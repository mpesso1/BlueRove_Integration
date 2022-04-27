#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include "ros_pid.h"
#include <chrono>
#include <Eigen/Dense>
#include <math.h>

using namespace std;


// ENUM DEFINING DEGREES OF FREEDOM -------------------
enum Cordinates {X, Y, Z, THX, THY, THZ}; // Position
enum Velocity {X_VEL, Y_VEL, Z_VEL, X_THVEL, Y_THVEL, Z_THVEL}; // Velocity

// PID Gains Initialized ------------------------------
Eigen::Matrix<float,1,6> pose_gain;
Eigen::Matrix<float,1,6> inte_gain;
Eigen::Matrix<float,1,6> deriv_gain;


Eigen::Matrix<float,6,1> desired_pose;

Eigen::Matrix<float,6,1> current_pose; // from bluerov / mavlink interface

float TOLERANCE = .001;

int DOF = 6;

float dt;


bool good = false;

int choosen_yaw = 20;

bool pidFinished;

void odom_callback(const nav_msgs::Odometry& cs) {
    
    if (cs.header.seq == 1) {
        good = false;
    }
    else {
        good = true;
    }

    // odom data
    current_pose(X) = cs.pose.pose.position.x; // UNITS: m
    current_pose(Y) = cs.pose.pose.position.y; // UNITS: m
    current_pose(Z) = cs.pose.pose.position.z; // UNITS: m

/*    current_velo(X_VEL) = cs.twist.twist.linear.x; // UNTIS: m/s
    current_velo(Y_VEL) = cs.twist.twist.linear.y; // UNTIS: m/s
    current_velo(Z_VEL) = cs.twist.twist.linear.z; // UNTIS: m/s
*/
    current_pose(THX) = cs.pose.pose.orientation.x; // UNITS: deg
    current_pose(THY) = cs.pose.pose.orientation.y; // UNITS: deg

    if (abs(desired_pose(THZ) - cs.pose.pose.orientation.z) < abs(desired_pose(THZ) - cs.pose.pose.orientation.w)) {
        current_pose(THZ) = cs.pose.pose.orientation.z; // UNITS: deg
    }
    else {
        current_pose(THZ) = cs.pose.pose.orientation.w; // UNITS: deg
    }
    

 /*   current_velo(X_THVEL) = cs.twist.twist.angular.x; // UNTIS: deg/s
    current_velo(Y_THVEL) = cs.twist.twist.angular.y; // UNTIS: deg/s
    current_velo(Z_THVEL) = cs.twist.twist.angular.z; // UNTIS: deg/s*/
}


int main(int argc, char** argv) {

    // PID GAIN -- (x, y, z, thx, thy, thz) ---------------------------------
    pose_gain << 40, 40, 60, .01, .01, 40;
    inte_gain << 9000000, 9000000, 10000000, .0, .0, 9000000;
    deriv_gain << .0, .0, .0, .0, .0, .0;

    

    desired_pose(X) = 0;
    desired_pose(Y) = 0;
    desired_pose(Z) = 0;
    desired_pose(THX) = 0;
    desired_pose(THY) = 0;
    desired_pose(THZ) = 0;

    pid::rosPID controller(DOF, pose_gain, inte_gain, deriv_gain);

    ros::init(argc,argv,"PID_TEST");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    ros::Publisher rc_commands = n.advertise<nav_msgs::Odometry>("thrust_commands",1000);
    ros::Publisher desired = n.advertise<nav_msgs::Odometry>("desired",1000);   // for plotting data

    ros::Subscriber odom = n.subscribe("odom",1000,odom_callback);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();


    nav_msgs::Odometry thrust;
    nav_msgs::Odometry desired_pose_plot;

    desired_pose_plot.pose.pose.position.x = desired_pose(X);
    desired_pose_plot.pose.pose.position.y = desired_pose(Y);
    desired_pose_plot.pose.pose.position.z = desired_pose(Z);
    desired_pose_plot.pose.pose.orientation.z = desired_pose(THZ);



    

    while (ros::ok()) {


        desired_pose_plot.header.stamp = ros::Time::now();

        // Time differential per PID step -----------------------------------------------
        dt= std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() *.000000001; // ellapsed time in seconds
        // ------------------------------------------------------------------------------

        // Begin timmer -----------------------------------------------------------------
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        // ------------------------------------------------------------------------------


        

        if (good) {
            pidFinished = controller.run_pid(dt,desired_pose,current_pose,TOLERANCE,false);
            thrust.pose.pose.position.x = controller.controller_output(X)*cos(current_pose(THZ)) + controller.controller_output(Y)*sin(current_pose(THZ));
            thrust.pose.pose.position.y = controller.controller_output(Y)*cos(current_pose(THZ)) + controller.controller_output(X)*sin(current_pose(THZ));
            thrust.pose.pose.position.z = -controller.controller_output(Z);
            thrust.pose.pose.orientation.z = controller.controller_output(THZ);
        }

        if (pidFinished) {
            std::cout << "PID did its job \n";
        }


        thrust.header.stamp = ros::Time::now();

        rc_commands.publish(thrust);
        desired.publish(desired_pose_plot);
        ros::spinOnce();
        loop_rate.sleep();


        // End timmer -------------------------------------------------------------------
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    }
    return 0;
}