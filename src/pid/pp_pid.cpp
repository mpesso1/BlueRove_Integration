#include <iostream>
#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include "ros_pid.h"
#include <Eigen/Dense>
#include <chrono>

int DOF = 6; // 
int count = 0;
float td;


int main(int argc, char** argv) {

    ros::init(argc,argv,"Controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(10); // communication frequency

    // ------------------------ Publishing Topics ------------------> To: Robot <--------------------
    ros::Publish pitch = n.advertise<std_msgs::UInt16>("/BlueRov2/rc_channel1/set_pwn",1000);
    ros::Publish roll = n.advertise<std_msgs::UInt16>("/BlueRov2/rc_channel2/set_pwn",1000);
    ros::Publish z = n.advertise<std_msgs::UInt16>("/BlueRov2/rc_channel3/set_pwn",1000);
    ros::Publish yaw = n.advertise<std_msgs::UInt16>("/BlueRov2/rc_channel4/set_pwn",1000);
    ros::Publish x = n.advertise<std_msgs::UInt16>("/BlueRov2/rc_channel5/set_pwn",1000);
    ros::Publish y = n.advertise<std_msgs::UInt16>("/BlueRov2/rc_channel6/set_pwn",1000);
    // -----------------------------------------------------------------------------------


    // ------------------------ Subscribing Topics ----------------> From: Path Planner, Odometry <--
    ros::Subscribe pp = n.subscribe("waypoints",1000,pp_callback);
    ros::SUbscribe odom = n.subscribe("/BlueRov2/Odometry",1000,odom_callback);
    // -----------------------------------------------------------------------------------


    // ----  PID Gains -------------------------
    Eigen::Matrix<float,1,6> pose_gain;
    Eigen::Matrix<float,1,6> inte_gain;
    Eigen::Matrix<float,1,6> deriv_gain;
    // x, y, z, thx, thy, thz
    pose_gain << 1, 1, 1, 1, 1, 1;
    inte_gain << 1, 1, 1, 1, 1, 1;
    deriv_gain << 1, 1, 1, 1, 1, 1;
    // -----------------------------------------


    // Instance PID controller
    pid::rosPID controller(6, pose_gain, inte_gain, deriv_gain);

    // Ros loop ----  dt included in loop for controller (chrono library) ----
    while (ros::ok()) {
        if (count == 0) {
            td = 0.0;
        }
        else {
            td = std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() *.000000001; // ellapsed time in seconds
        }
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        controller.run_pid(td,desired,current); //************ do not have desired or current yet..

        ros::spinOnce();

        loop_rate.sleep();
        
        count = 1;

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    }



    return 0;
}