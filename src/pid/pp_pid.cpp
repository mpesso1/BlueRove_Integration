/*
ICORE LSU MECS
DATE:  APRIL 6th, 2022
AUTHOR: MASON PESSON
*/

#include <iostream>
#include <string>
#include "ros/ros.h"
#include "ros_pid.h"
#include <Eigen/Dense>
#include <chrono>
#include "std_msgs/UInt16.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "blue_rov_custom_integration/update_waypoint.h"
#include "blue_rov_custom_integration/control_pid.h"


// COMMUNICATION VARIABLES / STATE VARIABLES ----------
bool PID_ON = false;
bool PID_OFF = true;
bool PID_HEALTHY = false;
std::string ROVMAV_HEALTH = "UNHEALTHY";
bool NEW_TRAJ = false;

// ENUM DEFINING DEGREES OF FREEDOM -------------------
enum Cordinates {X, Y, Z, THX, THY, THZ}; // Position
enum Velocity {X_VEL, Y_VEL, Z_VEL, X_THVEL, Y_THVEL, Z_THVEL}; // Velocity

// CONTROL VARIABLES ---------------
const int DOF = 6; // Degrees of Freedom
const float TOLERANCE = .1; // UNITS: m --> defines how close the desired_pose and current_pose states need to be to one another
float dt; // UNITS: s --> time difference defined by chrono library

// PID Gains Initialized ------------------------------
Eigen::Matrix<float,1,6> pose_gain;
Eigen::Matrix<float,1,6> inte_gain;
Eigen::Matrix<float,1,6> deriv_gain;

// ----- States Initialized ----------------------------
Eigen::Matrix<float,6,1> desired_pose; // from path planner
Eigen::Matrix<float,6,1> current_pose; // from bluerov / mavlink interface
Eigen::Matrix<float,6,1> current_pose_local; // from bluerov / mavlink interface
Eigen::Matrix<float,6,1> current_velo; // from bluerov / mavlink interface

float BtoLTHz = 0;

// ROSHUM to PID --> System information
// defined under main
bool pid_control_action(blue_rov_custom_integration::control_pid::Request &req, blue_rov_custom_integration::control_pid::Response &res);

// ROSMAV to PID  --> Odometry information
// defined under main
void odom_callback(const nav_msgs::Odometry& cs);

// ** MAIN ** ------------------------------------------ *********                          <<
int main(int argc, char** argv) {

    ros::init(argc,argv,"Controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(10); // communication frequency [hz]

    // Publishing TOPICS -----------------> To: Robot <----------------------
    ros::Publisher rc_commands = n.advertise<nav_msgs::Odometry>("ROV_RC_COMMANDS",1000);

    ros::Publisher desiredPlot = n.advertise<nav_msgs::Odometry>("PLOT_DESIRED",1000);
    ros::Publisher currentPlot = n.advertise<nav_msgs::Odometry>("PLOT_CURRENT",1000);

    // Subscribing TOPICS -----------------> From: ROSMAV <-----------------
    ros::Subscriber odom = n.subscribe("ROV_ODOMETRY",1000,odom_callback); // nav_msgs/Odometry  :  Defines current_* 

    // Client SERVICE ---------------------> Server: pp_goPath <-----------
    blue_rov_custom_integration::update_waypoint waypoint;
    ros::ServiceClient client1 = n.serviceClient<blue_rov_custom_integration::update_waypoint>("service_line_waypoint");

    // Server SERVICE --------------------> Server: ROSHUM <---------------
    ros::ServiceServer pid_server = n.advertiseService("pid_system_control", pid_control_action);

    // PID GAIN -- (x, y, z, thx, thy, thz) ---------------------------------
    // pose_gain << 160, 160, 60, .01, .01, 40;
    // inte_gain << 9000000, 9000000, 10000000, .0, .0, 9000000;
    // deriv_gain << .0, .0, .0, .0, .0, .0;


    // pose_gain << 170, 170, 135, .01, .01, 160; // z p --> 310
    // inte_gain << 900000, 900000, 900000, .0, .0, 900000;
    // deriv_gain << .0, .0, .0000, .0, .0, .0;

    pose_gain << 170, 170, 135, .01, .01, 140; // z p --> 310
    inte_gain << 900000, 900000, 900000, .0, .0, 70000;
    deriv_gain << .0, .0, .0000, .0, .0, .000007;

    // PID controller object ------------------------------------------------
    pid::rosPID controller(DOF, pose_gain, inte_gain, deriv_gain);
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    nav_msgs::Odometry thrust;

    nav_msgs::Odometry plotDesired;
    nav_msgs::Odometry plotCurrent;

    // ROS loop -------------------------------------------------------------
    /* dt included in loop for controller (chrono library) */
    while (ros::ok()) {
        
         // Time differential per PID step -----------------------------------------------
        dt= std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() *.000000001; // ellapsed time in seconds
        // ------------------------------------------------------------------------------

        // Begin timmer -----------------------------------------------------------------
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        // ------------------------------------------------------------------------------
        // std::cout << dt << std::endl;
        
        // ROVMAV_HEALTH gets communicated over ROV_ODOMETRY and PID_ON / NEW_TRAJ gets communicated through ROVHUM... communication with the path planner leads up to this
        if (ROVMAV_HEALTH=="HEALTHY" && PID_ON==true) { // ROVMAV_HEALTH does not become healthy until pid pp and ROVMAV health in system byte all get set set high... Need system byte to be set to 15 or greater
            if (NEW_TRAJ) { 

                //controller.integral_error = 0; // insure intergral error is set to 0 to avoid there being and value still stored from whenever the object was initiated

                // Request to server current pose ----------------------------------------------
                waypoint.request.NewTraj = true;
                waypoint.request.x = current_pose(X); 
                waypoint.request.y = current_pose(Y);
                waypoint.request.z = current_pose(Z);
                waypoint.request.thx = current_pose(THX);
                waypoint.request.thy = current_pose(THY); // ** The reason we are sending the path planner our current position is because it will need it to generate the trajectory
                waypoint.request.thz = current_pose(THZ);
                waypoint.request.x_vel = current_velo(X_VEL);
                waypoint.request.y_vel = current_velo(Y_VEL);
                waypoint.request.z_vel = current_velo(Z_VEL);
                waypoint.request.thx_vel = current_velo(X_THVEL);
                waypoint.request.thy_vel = current_velo(Y_THVEL);
                waypoint.request.thz_vel = current_velo(Z_THVEL);

                // Response from server ---------------------------------------------------------
                if (client1.call(waypoint)) {
                    desired_pose(X) = waypoint.response.x_way;
                    desired_pose(Y) = waypoint.response.y_way;
                    desired_pose(Z) = waypoint.response.z_way;
                    desired_pose(THX) = waypoint.response.thx_way;
                    desired_pose(THY) = waypoint.response.thy_way;
                    desired_pose(THZ) = waypoint.response.thz_way;

                    BtoLTHz = desired_pose(THZ);
                    // std::cout << "DESIRED X: " << desired_pose(X) << std::endl;
                    // std::cout << "DESIRED Y: " << desired_pose(Y) << std::endl;
                    // std::cout << "DESIRED Z: " << desired_pose(Z) << std::endl;
                }
                else {
                    std::cout << "\033[1;31m \n PID cannot communicate w/ PathPlanner \n Waypoint Server Not Responding \033[m \n\n";
                }
                NEW_TRAJ = false;
            }

            else {

                // Call PID step ----------------------------------------------------------------
                bool state_reached = controller.run_pid(dt,desired_pose,current_pose, TOLERANCE, false);

                // std::cout << "X LOCAL POSITION: " << current_pose_local(X) << std::endl;
                // std::cout << "X Global: " << current_pose(X) << std::endl;
                // std::cout << "Y LOCAL POSITION: " << current_pose_local(Y) << std::endl;
                // std::cout << "Y Global: " << current_pose(Y) << std::endl;

                if (state_reached) {
                    controller.reset_pid(true, true);
                    waypoint.request.NewTraj = false;
                    // Response from server ---------------------------------------------------------
                    if (client1.call(waypoint)) {
                        desired_pose(X) = waypoint.response.x_way;
                        desired_pose(Y) = waypoint.response.y_way;
                        desired_pose(Z) = waypoint.response.z_way;
                        desired_pose(THX) = waypoint.response.thx_way;
                        desired_pose(THY) = waypoint.response.thy_way;
                        desired_pose(THZ) = waypoint.response.thz_way;

                        if (waypoint.response.pathcomplete) {
                            controller.reset_pid(true, true);
                            thrust.pose.pose.orientation.x = 1;
                            PID_ON = false;
                            PID_OFF = true;
                            std::cout << "\033[1;47m PID FINISHED EXECUTING FOR CURRENT WAYPOINT \033[m \n\n";
                            rc_commands.publish(thrust);
                        }
                    }
                    else {
                        std::cout << "\033[1;31m \n PID cannot communicate w/ PathPlanner \n Waypoint Server Not Responding \033[m \n\n";
                    }
                }
                else {
                    thrust.header.frame_id = "STABILIZE";
                    thrust.pose.pose.position.x = controller.controller_output(X)*cos(-current_pose(THZ)) - controller.controller_output(Y)*sin(-current_pose(THZ));
                    thrust.pose.pose.position.y = controller.controller_output(Y)*cos(-current_pose(THZ)) + controller.controller_output(X)*sin(-current_pose(THZ));
                    thrust.pose.pose.position.z = -controller.controller_output(Z);
                    thrust.pose.pose.orientation.z = controller.controller_output(THZ);

                    rc_commands.publish(thrust);



                    // std::cout << "X xontroller output: " << controller.controller_output(X) << std::endl;
                    // std::cout << "Y xontroller output: " << controller.controller_output(Y) << std::endl;
                    // std::cout << "Z xontroller output: " << controller.controller_output(Z) << std::endl;
                    // std::cout << "YAW xontroller output: " << controller.controller_output(THZ) << std::endl;
                }   
            }        
        }
        else { // system healthy but PID is not on
            
            thrust.header.frame_id = "POSHOLD";
            thrust.pose.pose.position.x = 0;
            thrust.pose.pose.position.y = 0;
            thrust.pose.pose.position.z = 0;
            thrust.pose.pose.orientation.z = 0;

            thrust.pose.pose.orientation.x = 0;
            rc_commands.publish(thrust);
        }

        // End timmer -------------------------------------------------------------------
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    
        plotCurrent.pose.pose.position.x = current_pose(X);
        plotCurrent.pose.pose.position.y = current_pose(Y);
        plotCurrent.pose.pose.position.z = current_pose(Z);
        plotCurrent.pose.pose.orientation.z = current_pose(THZ);

        plotCurrent.header.stamp = ros::Time::now();
        plotDesired.header.stamp = ros::Time::now();

        currentPlot.publish(plotCurrent);

        plotDesired.pose.pose.position.x = desired_pose(X);
        plotDesired.pose.pose.position.y = desired_pose(Y);
        plotDesired.pose.pose.position.z = desired_pose(Z);
        plotDesired.pose.pose.orientation.z = desired_pose(THZ);

        desiredPlot.publish(plotDesired);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

// Odometry callback function
void odom_callback(const nav_msgs::Odometry& cs) { // cs := current_pose state from bluerov
    
    // Will not begin recieving data until odom data is being recieved from ROV and the system is armed

    // system status data
    ROVMAV_HEALTH = cs.header.frame_id; 

    // LINEAR odom data
    current_pose(X) = cs.pose.pose.position.x; // UNITS: m
    current_pose(Y) = cs.pose.pose.position.y; // UNITS: m
    current_pose(Z) = cs.pose.pose.position.z; // UNITS: m ----------------------------________________KEKFFK#LE_#LD#)PKCEINVO#RJVojebnrgownj r2[ltbk ]   LOOK AT MEEEEEE <<<<<<<<<<<<<<<<<<<<<<<<<<<<_------------------

    current_velo(X_VEL) = cs.twist.twist.linear.x; // UNTIS: m/s
    current_velo(Y_VEL) = cs.twist.twist.linear.y; // UNTIS: m/s
    current_velo(Z_VEL) = cs.twist.twist.linear.z; // UNTIS: m/s


    // ANGULAR odom data
    current_pose(THX) = cs.pose.pose.orientation.x; // UNITS: deg
    current_pose(THY) = cs.pose.pose.orientation.y; // UNITS: deg

    if (abs(desired_pose(THZ) - cs.pose.pose.orientation.z) < abs(desired_pose(THZ) - cs.pose.pose.orientation.w)) {
        current_pose(THZ) = cs.pose.pose.orientation.z; // UNITS: deg
    }
    else {
        current_pose(THZ) = cs.pose.pose.orientation.w; // UNITS: deg
    }

    current_velo(X_THVEL) = cs.twist.twist.angular.x; // UNTIS: deg/s
    current_velo(Y_THVEL) = cs.twist.twist.angular.y; // UNTIS: deg/s
    current_velo(Z_THVEL) = cs.twist.twist.angular.z; // UNTIS: deg/s

    current_pose_local(X) = current_pose(X)*cos(BtoLTHz) + current_pose(Y)*sin(BtoLTHz);
    current_pose_local(Y) = current_pose(Y)*cos(BtoLTHz) + current_pose(X)*sin(BtoLTHz);
    current_pose_local(Z) = current_pose(Z);
    current_pose_local(THX) =  current_pose(THX);
    current_pose_local(THY) = current_pose(THY); 
    current_pose_local(THZ) = current_pose(THZ);

    // std::cout << "ODOM X: " << current_pose(X) << std::endl;
    // std::cout << "ODOM Y: " << current_pose(Y) << std::endl;
    // std::cout << "ODOM Z: " << current_pose(Z) << std::endl;
    // std::cout << "ODOM THZ: " << current_pose(THZ) << std::endl;
    //std::cout << "ODOM THW: " << cs.pose.pose.orientation.w << std::endl;

    // std::cout << "DESIRED X: " << desired_pose(X) << std::endl;
    // std::cout << "DESIRED Y: " << desired_pose(Y) << std::endl;
    // std::cout << "DESIRED Z: " << desired_pose(Z) << std::endl;
    
}

// only get called from ROSHUM node on system byte change
bool pid_control_action(blue_rov_custom_integration::control_pid::Request &req, blue_rov_custom_integration::control_pid::Response &res) {
    PID_ON = req.PID_on;
    PID_OFF = req.PID_off;
    NEW_TRAJ = req.New_Traj;

    res.pid_healthy = true;

    return true;
}







