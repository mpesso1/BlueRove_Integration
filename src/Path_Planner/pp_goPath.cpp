/*
ICORE LSU MECS
DATE:  APRIL 6th, 2022
AUTHOR: MASON PESSON
*/

#include "goPath.h"
#include <Eigen/Dense>
#include <iostream>
#include <chrono>
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include "blue_rov_custom_integration/update_waypoint.h"
#include "blue_rov_custom_integration/control_pathplanner.h"
#include "blue_rov_custom_integration/pathplanner_update_waypoint.h"


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

void add_object_xyz(float x, float y, float z); // Defined under main

// Waypoint / Final Goal Data
bool NEED_NEW_WAYPOINT = true;// indication if we need new trajectory
bool NEW_TRAJ; // boolian defining if new path is going to be generated
float x_goal_wp;
float y_goal_wp;
float z_goal_wp;
float yaw_goal_wp;

// ENUM DEFINING DEGREES OF FREEDOM -------------------
enum Cordinates {X, Y, Z, THX, THY, THZ}; // Position
enum Velocity {X_VEL, Y_VEL, Z_VEL, X_THVEL, Y_THVEL, Z_THVEL}; // Velocity


// CONTROL VARIABLES ---------------
const float MAX_ACCEL = .1; // maximum acceleration of robot defined
int STEP = 0; // Index of what step the pid is on. Used to index trajectory. 

const int DOF = 6; // Degrees of freedom
const int STEPS = 60; // steps within trajectory
const int OCV = 3; // object concering DOF

const float WHOLE_SENSITIVITY = 5.0;
const float ABILITY_SENSITIVITY = 2.0;
const float POWER_SENSITIVITY = 2.0;


// Path Planning Object -------------------------------
MeanTraj BlueRov(DOF, STEPS, OCV);


// Object Cordinates ----------------------------------
std::vector<float> objx;
std::vector<float> objy;
std::vector<float> objz;


// Trajectory -----------------------------------------
/* Will store the entire trajectories for both translational and rotational movement */;
Eigen::Matrix<float,60+1, 3> path_trans;
Eigen::Matrix<float,60+1, 3> path_angular; // currently need the amount of steps to be set to 60


// Server response decloration -------------------------
bool send_waypoint(blue_rov_custom_integration::update_waypoint::Request &req, blue_rov_custom_integration::update_waypoint::Response &res);


// only gets called whenever the system byte has changed and communicated through the ROSHUM node
bool pp_control_action(blue_rov_custom_integration::control_pathplanner::Request &req, blue_rov_custom_integration::control_pathplanner::Response &res) {
    if (req.cv_said_new_path) {
        NEW_TRAJ = true;
        res.cv_enforced = true;
    }
    else if (req.ask_if_new_path_needed) {
        if (NEED_NEW_WAYPOINT) {
            NEW_TRAJ = true;
        }
    }
    else {
        NEW_TRAJ = false;
    }

    if (NEW_TRAJ) {
        res.need_new_path = true;
    }
    else {
        res.need_new_path = false;
    }

    res.pp_healthy = true;
}


bool pp_waypoint_callback(blue_rov_custom_integration::pathplanner_update_waypoint::Request &req, blue_rov_custom_integration::pathplanner_update_waypoint::Response &res) {
    x_goal_wp = req.x;
    y_goal_wp = req.y;
    z_goal_wp = req.z;
    yaw_goal_wp = req.yaw;

    add_object_xyz(req.ox,req.oy,req.oz);
    // need a way of not adding objects whenever cv does not add new object otherwise this will add an object to location 0
}

// ** MAIN ** ------------------------------------------ *********
int main(int argc,char** argv) {

    // Control path planner sensitivity parameters -------------------------
    BlueRov.set_sensitivity(WHOLE_SENSITIVITY,ABILITY_SENSITIVITY,POWER_SENSITIVITY);

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

// Create pseudo objects function
void add_object_xyz(float x, float y, float z) {
    objx.push_back(x);
    objy.push_back(y);
    objz.push_back(z);
}

// WILL NOT GET CALLED UNTIL PID GET CALLED FROM ROSHUM
bool send_waypoint(blue_rov_custom_integration::update_waypoint::Request &req, blue_rov_custom_integration::update_waypoint::Response &res) {
    
    /* When a trajectory needs to be created it will be created before the first step and before the first step only.  Once the final step of the generated trajectory has been reached then another first step will be needed and another trajectory will be created to the new goal position.  All subsequent steps will just be feeding the pid the next step needed defined by the overall trajectory. */

    // Inorder for a path to be generated there needs to be a handshake between the pid and the pp... this is accomplished by the system byte messgae in the ROSHUM node..
    // first the byte must change then second the pp must declare that is needs a path then then third the pid must be turned on
    // pid dirrectly communicates with the path planner so it will be able to indicate that it needs a new traj within the service call
    if (NEW_TRAJ && req.NewTraj) { // HANDSHAKE between the pid and the path planner on if a new path needs to be generated
        // Initialize prior trajectories ----------------------
        /* acceleration, init velocity, init poition, final position, indx of DOF (used for algorithm), boolian defining ocv */

        BlueRov.add_DOF(MAX_ACCEL, req.x_vel, req.x, x_goal_wp, 0, true); 
        BlueRov.add_DOF(MAX_ACCEL, req.y_vel, req.y, y_goal_wp, 1, true); 
        BlueRov.add_DOF(MAX_ACCEL, req.z_vel, req.z, z_goal_wp, 2, true);
        BlueRov.add_DOF(MAX_ACCEL, req.thx_vel, req.thx, 0, 3, false);
        BlueRov.add_DOF(MAX_ACCEL, req.thy_vel, req.thy, 0, 4, false);
        BlueRov.add_DOF(MAX_ACCEL, req.thz_vel, req.thz, yaw_goal_wp, 5, false);

        /*//::::: TESTING
        BlueRov.add_DOF(MAX_ACCEL, 2, 0, 5, 0, true); 
        BlueRov.add_DOF(-MAX_ACCEL, 2, 0, 5, 1, true); 
        BlueRov.add_DOF(-MAX_ACCEL, 2, 0, 10, 2, true);
        BlueRov.add_DOF(MAX_ACCEL, 0, 0, 0, 3, false);
        BlueRov.add_DOF(MAX_ACCEL, 0, 0, 0, 4, false);
        BlueRov.add_DOF(MAX_ACCEL, 0, 0, 0, 5, false);
        // ----------------------------------------------------*/

        // Pseudo objects cordinates
        /*add_object_xyz(2.59,2.86,3.31);
        add_object_xyz(3.75,5.46,8.68);
        add_object_xyz(2.2,3.8,6.5);
        add_object_xyz(3.0,4.0,7.0);
        add_object_xyz(4.024,4.71,6.27);
        // ----------------------------------------------------*/

        // Optimize trajectory based on object cordinates -----
        BlueRov.optimize(objx,objy,objz); 

        // Store trajectory -----------------------------------
        path_trans = BlueRov.trajectory_translational();
        //path_angular = BlueRov.trajectory_angular();

        // Display trajectory ---------------------------------
        //""" Used for storing results in file system. Must use linux >> operator to redirect to results to desired file """ ; 
        BlueRov.traj(1);

        // ----------------------------------------------------
        //""" Do not generate new trajectory until needed again. Just feed waypoints """;
        NEW_TRAJ = false;
        NEED_NEW_WAYPOINT = false;
    }


    // Send trajectory waypoint ---------------------------
    res.x_way = path_trans(STEP,0);
    res.y_way = path_trans(STEP,1);
    res.z_way = path_trans(STEP,2);
    res.thx_way = 0;
    res.thy_way = 0;
    res.thz_way = 0; // NOTE THAT yaw is not desired yaw is not being updated


    // Once the requested path point is equal to the initial goal set point the n we can declare that the path planners job is finished and we need a new goal waypoint
    if (res.x_way == x_goal_wp && res.y_way == y_goal_wp && res.z_way == z_goal_wp){
        NEED_NEW_WAYPOINT = true;
    }


    // Increment trajectory index -------------------------
    STEP = STEP + 1;
    // ----------------------------------------------------

    return true;
}
