#include "goPath.h"
#include <Eigen/Dense>
#include <iostream>
#include <chrono>
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include "blue_rov_custom_integration/update_waypoint.h"
//access token == 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

using namespace std;
using namespace root;


// Gloabal Variables ----------
const float MAX_ACCEL = .1; // maximum acceleration of robt defined
bool NEW_TRAJ = true; // boolian defining if new path needs to be generated
int STEP = 0; // index of what step the pid is on

const int DOF = 6; // Degrees of freedom
const int STEPS = 60; // steps within trajectory
const int OCV = 3; // object concering DOF

const float WHOLE_SENSITIVITY = 5.0; // UNIQUE TO PATH PLANNER --> check goPath.h for explination
const float ABILITY_SENSITIVITY = 2.0;
const float POWER_SENSITIVITY = 2.0;
// ----------------------------


// Path Planning Object --------------------
MeanTraj BlueRov(DOF, STEPS, OCV);
// -----------------------------------------


// Object Cordinates ----------------------- // The way these object cordinates will get called will be the next phase of project
std::vector<float> objx;
std::vector<float> objy;
std::vector<float> objz;
void add_object_xyz(float x_local, float y_local, float z_local);
// ----------------------------------------- *********** Next Project.  Will need to work with vision system


// ----------- Trajectory --------------------
Eigen::Matrix<float,60+1, 3> path_trans; //******** Need to figure out how to put variables in Eigen -- messy!!
Eigen::Matrix<float,60+1, 3> path_angular; // not getting used right now
// -------------------------------------------



bool send_waypoint(blue_rov_custom_integration::update_waypoint::Request &req, blue_rov_custom_integration::update_waypoint::Response &res) {
    if (NEW_TRAJ) {
        BlueRov.add_DOF(MAX_ACCEL, req.x_vel, req.x, 4., 0, true); // acceleration, init velocity, init poition, final position, indx, boolian defining ocv
        BlueRov.add_DOF(MAX_ACCEL+.1, req.y_vel, req.y, 7., 1, true); 
        BlueRov.add_DOF(MAX_ACCEL, req.z_vel, req.z, 12., 2, true);
        BlueRov.add_DOF(MAX_ACCEL, req.thx_vel, req.thx, 4., 3, false);
        BlueRov.add_DOF(MAX_ACCEL, req.thy_vel, req.thy, 4., 4, false);
        BlueRov.add_DOF(MAX_ACCEL, req.thz_vel, req.thz, 5., 5, false);


        BlueRov.optimize(objx,objy,objz); 
        BlueRov.traj(1);
        path_trans = BlueRov.trajectory_translational();
        //path_angular = BlueRov.trajectory_angular();
        NEW_TRAJ = false;
    }
    res.x_way = path_trans(STEP,0);
    res.y_way = path_trans(STEP,1);
    res.z_way = path_trans(STEP,2);
    res.thx_way = 0; //  Need to figure out if path planner will affect the non ocv when optimizing path
    res.thy_way = 0;
    res.thz_way = 0; //std::atan2(res.y_way,res.x_way); // --> Need to define this better!

    STEP = STEP + 1;

    return true;
}


int main(int argc,char** argv) {

    ros::init(argc,argv,"Path_Planner");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    BlueRov.set_sensitivity(WHOLE_SENSITIVITY,ABILITY_SENSITIVITY,POWER_SENSITIVITY);

    add_object_xyz(2.59,2.86,3.31);
    add_object_xyz(3.75,5.46,8.68);
    add_object_xyz(2.2,3.8,6.5);
    add_object_xyz(3.0,4.0,7.0);
    add_object_xyz(4.024,4.71,6.27);


    ros::Publisher publish = n.advertise<geometry_msgs::Pose>("waypoint",1000);

    ros::ServiceServer server = n.advertiseService("service_line_waypoint",send_waypoint);

    ros::spin();
    return 0;
}


void add_object_xyz(float x, float y, float z) {
    objx.push_back(x);
    objy.push_back(y);
    objz.push_back(z);
}



