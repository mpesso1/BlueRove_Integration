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


using namespace std;
using namespace root;

void add_object_xyz(float x, float y, float z);

// ENUM DEFINING DEGREES OF FREEDOM -------------------
enum Cordinates {X, Y, Z, THX, THY, THZ}; // Position
enum Velocity {X_VEL, Y_VEL, Z_VEL, X_THVEL, Y_THVEL, Z_THVEL}; // Velocity
// ----------------------------------------------------


// GLOBAL VARIABLES / CONTROL VARIABLES ---------------
const float MAX_ACCEL = .1; // maximum acceleration of robot defined
bool NEW_TRAJ = true; // boolian defining if new path needs to be generated
int STEP = 0; // Index of what step the pid is on. Used to index trajectory. 

const int DOF = 6; // Degrees of freedom
const int STEPS = 60; // steps within trajectory
const int OCV = 3; // object concering DOF

const float WHOLE_SENSITIVITY = 5.0;
const float ABILITY_SENSITIVITY = 2.0;
const float POWER_SENSITIVITY = 2.0;
// ----------------------------------------------------


// Path Planning Object -------------------------------
MeanTraj BlueRov(DOF, STEPS, OCV);
// ----------------------------------------------------


// Object Cordinates ----------------------------------
std::vector<float> objx;
std::vector<float> objy;
std::vector<float> objz;
// ----------------------------------------------------


// Trajectory -----------------------------------------
/* Will store the entire trajectories for both translational and rotational movement */;
Eigen::Matrix<float,60+1, 3> path_trans;
Eigen::Matrix<float,60+1, 3> path_angular;
// ----------------------------------------------------


// Server response decloration -------------------------
bool send_waypoint(blue_rov_custom_integration::update_waypoint::Request &req, blue_rov_custom_integration::update_waypoint::Response &res);
// -----------------------------------------------------


// ** MAIN ** ------------------------------------------ *********
int main(int argc,char** argv) {

    ros::init(argc,argv,"Path_Planner");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);


    // Control path planner sensitivity parameters -------------------------
    BlueRov.set_sensitivity(WHOLE_SENSITIVITY,ABILITY_SENSITIVITY,POWER_SENSITIVITY);
    // ---------------------------------------------------------------------

    // Pseudo objects cordinates
    add_object_xyz(2.59,2.86,3.31);
    add_object_xyz(3.75,5.46,8.68);
    add_object_xyz(2.2,3.8,6.5);
    add_object_xyz(3.0,4.0,7.0);
    add_object_xyz(4.024,4.71,6.27);
    // ---------------------------------------------------------------------

    // Server SERVICE -------------------------------------------------------
    ros::ServiceServer server = n.advertiseService("service_line_waypoint",send_waypoint);
    // ---------------------------------------------------------------------

    ros::spin();

    return 0;
}

// Create pseudo objects function
void add_object_xyz(float x, float y, float z) {
    objx.push_back(x);
    objy.push_back(y);
    objz.push_back(z);
}

// Server response 
bool send_waypoint(blue_rov_custom_integration::update_waypoint::Request &req, blue_rov_custom_integration::update_waypoint::Response &res) {
    
    /* When a trajectory needs to be created it will be created before the first step and before the first step only.  Once the final step of the generated trajectory has been reached then another first step will be needed and another trajectory will be created to the new goal position.  All subsequent steps will just be feeding the pid the next step needed defined by the overall trajectory. */

    // Initial trajectory action -----------------------------------------------------------------
    if (NEW_TRAJ) {
        // Initialize prior trajectories ----------------------
        /* acceleration, init velocity, init poition, final position, indx (used for algorithm), boolian defining ocv */

        /*BlueRov.add_DOF(MAX_ACCEL, req.x_vel, req.x, 4., 0, true); 
        BlueRov.add_DOF(MAX_ACCEL, req.y_vel, req.y, 7., 1, true); 
        BlueRov.add_DOF(MAX_ACCEL, req.z_vel, req.z, 12., 2, true);
        BlueRov.add_DOF(MAX_ACCEL, req.thx_vel, req.thx, 4., 3, false);
        BlueRov.add_DOF(MAX_ACCEL, req.thy_vel, req.thy, 4., 4, false);
        BlueRov.add_DOF(MAX_ACCEL, req.thz_vel, req.thz, 5., 5, false);*/

        //::::: TESTING
        BlueRov.add_DOF(MAX_ACCEL, 2, 0, 5, 0, true); 
        BlueRov.add_DOF(-MAX_ACCEL, 2, 0, 5, 1, true); 
        BlueRov.add_DOF(-MAX_ACCEL, 2, 0, 10, 2, true);
        BlueRov.add_DOF(MAX_ACCEL, 0, 0, 0, 3, false);
        BlueRov.add_DOF(MAX_ACCEL, 0, 0, 0, 4, false);
        BlueRov.add_DOF(MAX_ACCEL, 0, 0, 0, 5, false);
        // ----------------------------------------------------


        // Optimize trajectory based on object cordinates -----
        BlueRov.optimize(objx,objy,objz); 
        // ----------------------------------------------------


        // Store trajectory -----------------------------------
        path_trans = BlueRov.trajectory_translational();
        //path_angular = BlueRov.trajectory_angular();
        // ----------------------------------------------------


        // Display trajectory ---------------------------------
        //""" Used for storing results in file system. Must use linux >> operator to redirect to results to desired file """ ; 
        BlueRov.traj(1);
        // ----------------------------------------------------


        // ----------------------------------------------------
        //""" Do not generate new trajectory until needed again. Just feed waypoints """;
        NEW_TRAJ = false;
        // ----------------------------------------------------
    }
    // -------------------------------------------------------------------------------------------


    // Send trajectory waypoint ---------------------------
    res.x_way = path_trans(STEP,0);
    res.y_way = path_trans(STEP,1);
    res.z_way = path_trans(STEP,2);
    res.thx_way = 0; 
    res.thy_way = 0;
    res.thz_way = 0;
    // ----------------------------------------------------


    // Increment trajectory index -------------------------
    STEP = STEP + 1;
    // ----------------------------------------------------

    return true;
}
