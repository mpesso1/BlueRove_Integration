#include <iostream>
#include "ros/ros.h"
#include "ros_pid.h"
#include <Eigen/Dense>
#include <chrono>
#include "std_msgs/UInt16.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "blue_rov_custom_integration/update_waypoint.h"

// GLOBAL VARIABLES --------------------
const int DOF = 6; // Degrees of Freedom
int COUNT = 0; // couting variable used in script, only needed for first iteration
const float TOLERANCE = .0001; // defines how close the desired and current states need to be to one another
float td; // "time difference" defined by chrono library
// -------------------------------------

/*  ***** Server Used Instead *****
void pp_callback(const geometry_msgs::Pose& ds) {
    std::cout << "Hey" << std::endl;
}
*/


// ----  PID Gains -------------------------
Eigen::Matrix<float,1,6> pose_gain;
Eigen::Matrix<float,1,6> inte_gain;
Eigen::Matrix<float,1,6> deriv_gain;
// -----------------------------------------


// ----- States ----------------------------
Eigen::Matrix<float,6,1> desired;
Eigen::Matrix<float,6,1> current;
Eigen::Matrix<float,6,1> velo;
// -----------------------------------------


void odom_callback(const nav_msgs::Odometry& cs) {
    current(0) = cs.pose.pose.position.x;
    current(1) = cs.pose.pose.position.y;
    current(2) = cs.pose.pose.position.z;
    Eigen::Quaternionf q(cs.pose.pose.orientation.w, cs.pose.pose.orientation.x, cs.pose.pose.orientation.y, cs.pose.pose.orientation.z);
    auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    current(3) = euler(0);
    current(4) = euler(1);
    current(5) = euler(2);

    velo(0) = cs.twist.twist.linear.x;
    velo(1) = cs.twist.twist.linear.y;
    velo(2) = cs.twist.twist.linear.z;
    velo(3) = cs.twist.twist.angular.x;
    velo(4) = cs.twist.twist.angular.y;
    velo(5) = cs.twist.twist.angular.z;
}


int main(int argc, char** argv) {

    ros::init(argc,argv,"Controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(10); // communication frequency
    
    // x, y, z, thx, thy, thz
    pose_gain << 1, 1, 1, 1, 1, 1;
    inte_gain << 1, 1, 1, 1, 1, 1;
    deriv_gain << 1, 1, 1, 1, 1, 1;

    // ------------------------ Publishing Topics ------------------> To: Robot <--------------------
    ros::Publisher pitch = n.advertise<std_msgs::UInt16>("/BlueRov2/rc_channel1/set_pwn",1000);
    ros::Publisher roll = n.advertise<std_msgs::UInt16>("/BlueRov2/rc_channel2/set_pwn",1000);
    ros::Publisher z = n.advertise<std_msgs::UInt16>("/BlueRov2/rc_channel3/set_pwn",1000);
    ros::Publisher yaw = n.advertise<std_msgs::UInt16>("/BlueRov2/rc_channel4/set_pwn",1000);
    ros::Publisher x = n.advertise<std_msgs::UInt16>("/BlueRov2/rc_channel5/set_pwn",1000);
    ros::Publisher y = n.advertise<std_msgs::UInt16>("/BlueRov2/rc_channel6/set_pwn",1000);
    // ----------------------------------------------------------------------------------------------


    // ------------------------ Subscribing Topics ----------------> From: Path Planner, Odometry <-- {respectivly}
    //ros::Subscriber pp = n.subscribe("waypoint",1000,pp_callback); // geometry_msgs/Pose ***** Server Used Instead *****
    ros::Subscriber odom = n.subscribe("/BlueRov2/odometry",1000,odom_callback); // nav_msgs/Odometry
    // ----------------------------------------------------------------------------------------------


    // ----------------------- Client -----------------------------> Server: Path Planner <----------
    ros::ServiceClient client1 = n.serviceClient<blue_rov_custom_integration::update_waypoint>("service_line_waypoint");
    blue_rov_custom_integration::update_waypoint waypoint;
    // ----------------------------------------------------------------------------------------------


    // Setup before Loop -----------------------
    waypoint.request.x = current(0);
    waypoint.request.y = current(1);
    waypoint.request.z = current(2);
    waypoint.request.thx = current(3);
    waypoint.request.thy = current(4);
    waypoint.request.thy = current(5);

    waypoint.request.x_vel = velo(0);
    waypoint.request.y_vel = velo(1);
    waypoint.request.z_vel = velo(2);
    waypoint.request.thx_vel = velo(3);
    waypoint.request.thy_vel = velo(4);
    waypoint.request.thz_vel = velo(5);

    if (client1.call(waypoint)) {
        desired(0) = waypoint.response.x_way;
        desired(1) = waypoint.response.y_way;
        desired(2) = waypoint.response.z_way;
        desired(3) = waypoint.response.thx_way;
        desired(4) = waypoint.response.thy_way;
        desired(5) = waypoint.response.thz_way;

    }
    else {
        std::cout << "Waypoint Server Not Responding\n";
        return 1;
    }
    // -----------------------------------------


    // ------------------------ PID controller Object -----------------------------------------------
    pid::rosPID controller(6, pose_gain, inte_gain, deriv_gain);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    // ----------------------------------------------------------------------------------------------


    // Ros loop ----  dt included in loop for controller (chrono library) ----
    while (ros::ok()) {
        if (COUNT == 0) {
            td = 0.0;
        }
        else {
            td = std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() *.000000001; // ellapsed time in seconds
            COUNT = 1;
        }
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        controller.run_pid(td,desired,current); //************ do not have desired or current yet..
        
        if (current.isApprox(desired,TOLERANCE)) {
            waypoint.request.x = current(0);
            waypoint.request.y = current(1);
            waypoint.request.z = current(2);
            waypoint.request.thx = current(3);
            waypoint.request.thy = current(4);
            waypoint.request.thy = current(5);
            
            if (client1.call(waypoint)) {
                desired(0) = waypoint.response.x_way;
                desired(1) = waypoint.response.y_way;
                desired(2) = waypoint.response.z_way;
                desired(3) = waypoint.response.thx_way;
                desired(4) = waypoint.response.thy_way;
                desired(5) = waypoint.response.thz_way;
            }
        }

        ros::spinOnce();

        loop_rate.sleep();

        COUNT = 1;

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    }

    return 0;
}







