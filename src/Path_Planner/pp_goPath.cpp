/*
ICORE LSU MECS
DATE:  APRIL 6th, 2022
AUTHOR: MASON PESSON
*/
// #include "/home/mason/catkin_ws/src/blue_rov_custom_integration/lib/PathPlanner/PlannerTemplates/_goPath.h"
// #include "/home/mason/catkin_ws/src/blue_rov_custom_integration/lib/PathPlanner/PlannerTemplates/_Astar.h"
// #include "/home/mason/catkin_ws/src/blue_rov_custom_integration/lib/PathPlanner/PlannerTemplates/_omplRRTConnect.h"

#include "/home/mason/catkin_ws/src/blue_rov_custom_integration/lib/PathPlanner/PathPlanners/goPath.h"

#include <Eigen/Dense>
#include <iostream>
#include <chrono>
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include "blue_rov_custom_integration/update_waypoint.h"
#include "blue_rov_custom_integration/control_pathplanner.h"
#include "blue_rov_custom_integration/pathplanner_update_waypoint.h"
#include <vector>
#include <fstream>


/*
pp_goPath

    Communicattion:
        1.) pp_goPath w/ pp_pid through service_line_waypoint servie
            - defines current and desired pose information

        2.) pp_goPath w/ ROSHUM through pp_system_control service
            - defines system config informations

        3.) pp_goPath w/ ROSHUM through pp_waypoint service
            - defines new trajectory information
*/

using namespace std;
using namespace root;


#define PI 3.14159265

// Waypoint / Final Goal Data
bool NEED_NEW_WAYPOINT = true;// indication if we need new trajectory
bool NEW_TRAJ = false; // boolian defining if new path is going to be generated

// ENUM DEFINING DEGREES OF FREEDOM -------------------
enum Cordinates {X, Y, Z, THX, THY, THZ}; // Position
enum Velocity {X_VEL, Y_VEL, Z_VEL, X_THVEL, Y_THVEL, Z_THVEL}; // Velocity

int STEP = 0; // Index of what step the pid is on. Used to index trajectory. 
int STEPS = 60;
const int DOF = 6;

Eigen::Matrix<float,61, 3> path_trans;
Eigen::Matrix<float,61, 1> path_anglu;

Eigen::Matrix<float,1,DOF> goal_pose;

float ojx {};
float ojy {};
float ojz {};



// Server response decloration -------------------------
bool send_waypoint(blue_rov_custom_integration::update_waypoint::Request &req, blue_rov_custom_integration::update_waypoint::Response &res);

// only gets called whenever the system byte has changed and communicated through the ROSHUM node
bool pp_control_action(blue_rov_custom_integration::control_pathplanner::Request &req, blue_rov_custom_integration::control_pathplanner::Response &res);

// Input from ROVHUM defining the goal waypoint
bool pp_waypoint_callback(blue_rov_custom_integration::pathplanner_update_waypoint::Request &req, blue_rov_custom_integration::pathplanner_update_waypoint::Response &res);


// ** MAIN ** ------------------------------------------ *********
int main(int argc,char** argv) {

    ros::init(argc,argv,"Path_Planner");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // Server SERVICE -------------------------------------------------------
    ros::ServiceServer pp_server = n.advertiseService("pp_system_control",pp_control_action);
    ros::ServiceServer pp_server_waypoint = n.advertiseService("pp_waypoint",pp_waypoint_callback);
    ros::ServiceServer server = n.advertiseService("service_line_waypoint",send_waypoint);

    ros::spin();

    return 0;
}


// WILL NOT GET CALLED UNTIL PID GET CALLED FROM ROSHUM
bool send_waypoint(blue_rov_custom_integration::update_waypoint::Request &req, blue_rov_custom_integration::update_waypoint::Response &res) {
    
    /* When a trajectory needs to be created it will be created before the first step and before the first step only.  Once the final step of the generated trajectory has been reached then another first step will be needed and another trajectory will be created to the new goal position.  All subsequent steps will just be feeding the pid the next step needed defined by the overall trajectory. */

    // Inorder for a path to be generated there needs to be a handshake between the pid and the pp... this is accomplished by the system byte messgae in the ROSHUM node..
    // first the byte must change then second the pp must declare that is needs a path then then third the pid must be turned on
    // pid dirrectly communicates with the path planner so it will be able to indicate that it needs a new traj within the service call
    if (NEW_TRAJ && req.NewTraj) { // HANDSHAKE between the pid and the path planner on if a new path needs to be generated

        //""" Do not generate new trajectory until needed again. Just feed waypoints """;
        NEW_TRAJ = false;

        float start_x = req.x;
        float start_y = req.y;
        float start_z = req.z;
        float start_thz = req.thz;

        cout << "INIT POSE: " << start_x << endl;
        cout << "INIT POSE: " << start_y << endl;
        cout << "INIT POSE: " << start_z << endl;
        cout << "INIT POSE: " << start_thz << endl;

        float final_x = goal_pose(X);
        float final_y = goal_pose(Y);
        float final_z = goal_pose(Z);
        float final_thz = goal_pose(THZ);

        cout << "GOAL POSE: " << final_x << endl;
        cout << "GOAL POSE: " << final_y << endl;
        cout << "GOAL POSE: " << final_z << endl;
        cout << "GOAL POSE: " << final_thz << endl;

        //Timer time;
        MeanTraj BlueRov(6,60,3); // # of DOF , steps, # of ocv
        // .2, -.7 init velos
        BlueRov.set_sensitivity(50,25,1); // whole, ability, power
        BlueRov.add_DOF(.1, 0 ,start_x,final_x,0,true); // acceleration, init velocity, init poition, final position, indx, boolian defining ocv  0.0016325 00143862  00329014
        BlueRov.add_DOF(.1, 0 ,start_y,final_y,1,true);
        BlueRov.add_DOF(.1, 0 ,start_z,final_z,2,true);
        BlueRov.add_DOF(.1, 0,0,0,3,false);
        BlueRov.add_DOF(.1, 0,0,0,4,false);
        BlueRov.add_DOF(.1, 0 ,start_thz,final_thz,5,false); // 0.5652

        //*/
        std::vector<float> objx;

        objx.push_back(ojx);

        //*/
        std::vector<float> objy;

        objy.push_back(ojy);

        //*/
        std::vector<float> objz;

        objz.push_back(ojz);


        BlueRov.optimize(objx,objy,objz);

        BlueRov.traj(1); // print path

        path_trans = BlueRov.trajectory_translational(); // 61 x 3
        path_anglu = BlueRov.trajectory_orientation(); // 61 x 1


        // pp::PathPlanner<DOF>* planner = new _goPath<DOF>();

        // planner->init_pose(X) = req.x;
        // planner->init_pose(Y) = req.y;
        // planner->init_pose(Z) = req.z;
        // planner->init_pose(THX) = 0;
        // planner->init_pose(THY) = 0;
        // planner->init_pose(THZ) = req.thz;

        // planner->init_vel(X) = req.x_vel;
        // planner->init_vel(Y) = req.y_vel;
        // planner->init_vel(Z) = req.z_vel;
        // planner->init_vel(THZ) = req.thz_vel;

        // cout << "INIT POSE: " << planner->init_pose(X) << endl;
        // cout << "INIT POSE: " << planner->init_pose(Y) << endl;
        // cout << "INIT POSE: " << planner->init_pose(Z) << endl;
        // cout << "INIT POSE: " << planner->init_pose(THZ) << endl;

        // planner->goal_pose = goal_pose;

        // cout << "GOAL POSE: " << planner->goal_pose(X) << endl;
        // cout << "GOAL POSE: " << planner->goal_pose(Y) << endl;
        // cout << "GOAL POSE: " << planner->goal_pose(Z) << endl;
        // cout << "GOAL POSE: " << planner->goal_pose(THZ) << endl;

        // planner->add_object(ojx,ojy,ojz);

        // cout << "Object x: " << ojx << endl;
        // cout << "Object y: " << ojy << endl;
        // cout << "Object z: " << ojz << endl;

        // planner->setup();
        
        // planner->compute();

        // planner->store_results();

        // path_trans = planner->path_trans;
        // path_anglu = planner->path_angular;


        // delete planner;

    }

    // Increment trajectory index -------------------------
    STEP = STEP + 1;

    
    // std::cout << "Sending desired pose from pp\n";
    res.x_way = path_trans(STEP,0); 
    res.y_way = path_trans(STEP,1);
    res.z_way = path_trans(STEP,2);
    res.thx_way = 0;
    res.thy_way = 0;
    res.thz_way = path_anglu(STEP,0);
    
    // std::cout << "Desired Angle: " << res.thz_way << std::endl;
    // std::cout << "Desired X: " << res.x_way << std::endl;
    // std::cout << "Desired Y: " << res.y_way << std::endl;

    if (STEP >= STEPS) {
        std::cout << "\033[1;47m PATHPLANNER FINISHED SENDING FOR CURRENT WAYPOINT \033[m \n";
        NEED_NEW_WAYPOINT = true;
        res.pathcomplete = true;
        STEP = 0;
    }

    return true;
}


// only gets called whenever the system byte has changed and communicated through the ROSHUM node
bool pp_control_action(blue_rov_custom_integration::control_pathplanner::Request &req, blue_rov_custom_integration::control_pathplanner::Response &res) {
    if (req.going_to_need_new_wp) {
        NEED_NEW_WAYPOINT = true;
    }
    if (req.cv_said_new_path) {
        NEW_TRAJ = true;
        res.cv_enforced = true;
        res.need_new_path = true;
    }
    else if (req.ask_if_new_path_needed && NEED_NEW_WAYPOINT) {
            NEW_TRAJ = true;
            res.cv_enforced = false;
            res.need_new_path = true;
    }
    else {
        res.need_new_path = false;
    }
    NEED_NEW_WAYPOINT = false;
    res.pp_healthy = true;

    return true;
}


bool pp_waypoint_callback(blue_rov_custom_integration::pathplanner_update_waypoint::Request &req, blue_rov_custom_integration::pathplanner_update_waypoint::Response &res) {

    Eigen::Matrix<float,1,DOF> m;

    if (DOF == 6) {
        m << req.x,req.y,req.z,0,0,req.yaw;
    }
    else if (DOF == 3)
    {
        m << req.x,req.y,req.z;
    }
    
    goal_pose = m;


    if (req.ox == 0 && req.oy == 0 && req.oz == 0) {
        
        cout << " \n";
        // need a way of not adding objects whenever cv does not add new object otherwise this will add an object to location 0
    }
    else {
        ojx = req.ox;
        ojy = req.oy;
        ojz = req.oz;
    }

    return true;
}
