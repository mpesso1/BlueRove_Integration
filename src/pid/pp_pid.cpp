#include <iostream>
#include "ros/ros.h"
#include "ros_pid.h"
#include <Eigen/Dense>
#include <chrono>
#include "std_msgs/UInt16.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "blue_rov_custom_integration/update_waypoint.h"


// ENUM DEFINING DEGREES OF FREEDOM -------------------
enum Cordinates {X, Y, Z, THX, THY, THZ}; // Position
enum Velocity {X_VEL, Y_VEL, Z_VEL, X_THVEL, Y_THVEL, Z_THVEL}; // Velocity
// ----------------------------------------------------


// GLOBAL VARIABLES / CONTROL VARIABLES ---------------
const int DOF = 6; // Degrees of Freedom
const float TOLERANCE = .0001; // defines how close the desired_pose and current_pose states need to be to one another
float dt; // "time difference" defined by chrono library
// ----------------------------------------------------


// PID Gains Initialized ------------------------------
Eigen::Matrix<float,1,6> pose_gain;
Eigen::Matrix<float,1,6> inte_gain;
Eigen::Matrix<float,1,6> deriv_gain;
// ----------------------------------------------------


// ----- States Initialized ----------------------------
Eigen::Matrix<float,6,1> desired_pose; // from path planner
Eigen::Matrix<float,6,1> current_pose; // from bluerov
Eigen::Matrix<float,6,1> current_velo; // current_velocity
// -----------------------------------------------------


// Odometry Callback -----------------------------------
/* Callback function used to get odometry data coming from bluerov.  Defined under main. */
void odom_callback(const nav_msgs::Odometry& cs);
// -----------------------------------------------------


// ** MAIN ** ------------------------------------------ *********
int main(int argc, char** argv) {

    ros::init(argc,argv,"Controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(10); // communication frequency [hz]

    // Publishing TOPICS -----------------> To: Robot <----------------------
    ros::Publisher pitch = n.advertise<std_msgs::UInt16>("/BlueRov2/rc_channel1/set_pwn",1000);
    ros::Publisher roll = n.advertise<std_msgs::UInt16>("/BlueRov2/rc_channel2/set_pwn",1000);
    ros::Publisher z = n.advertise<std_msgs::UInt16>("/BlueRov2/rc_channel3/set_pwn",1000);
    ros::Publisher yaw = n.advertise<std_msgs::UInt16>("/BlueRov2/rc_channel4/set_pwn",1000);
    ros::Publisher x = n.advertise<std_msgs::UInt16>("/BlueRov2/rc_channel5/set_pwn",1000);
    ros::Publisher y = n.advertise<std_msgs::UInt16>("/BlueRov2/rc_channel6/set_pwn",1000);
    // ----------------------------------------------------------------------


    // Subscribing TOPICS -----------------> From: Odometry <-----------------
    ros::Subscriber odom = n.subscribe("/BlueRov2/odometry",1000,odom_callback); // nav_msgs/Odometry  :  Defines current_* 
    // ----------------------------------------------------------------------


    // Client SERVICE ---------------------> Server: Path Planner <-----------
    blue_rov_custom_integration::update_waypoint waypoint;
    ros::ServiceClient client1 = n.serviceClient<blue_rov_custom_integration::update_waypoint>("service_line_waypoint");
    // ----------------------------------------------------------------------


    // PID GAIN -- (x, y, z, thx, thy, thz) ---------------------------------
    pose_gain << .01, .01, .01, .01, .01, .01;
    inte_gain << .01, .01, .01, .01, .01, .01;
    deriv_gain << .01, .01, .01, .01, .01, .01;
    // ----------------------------------------------------------------------


    // Initial trajectory setup on startup --------------------------------------------------------------------------------------
    /* Will be further updated in the bluerov callback function. */
    /* Should initially all be approximatly zero. */
    // Request to server ----------------------------------------------------
    waypoint.request.x = current_pose(X);
    waypoint.request.y = current_pose(Y);
    waypoint.request.z = current_pose(Z);
    waypoint.request.thx = current_pose(THX);
    waypoint.request.thy = current_pose(THY);
    waypoint.request.thy = current_pose(THZ);
    waypoint.request.x_vel = current_velo(X_VEL);
    waypoint.request.y_vel = current_velo(Y_VEL);
    waypoint.request.z_vel = current_velo(Z_VEL);
    waypoint.request.thx_vel = current_velo(X_THVEL);
    waypoint.request.thy_vel = current_velo(Y_THVEL);
    waypoint.request.thz_vel = current_velo(Z_THVEL);
    // ----------------------------------------------------------------------

    // Response from server -------------------------------------------------
    if (client1.call(waypoint)) { // call from path planer
        desired_pose(X) = waypoint.response.x_way;
        desired_pose(Y) = waypoint.response.y_way;
        desired_pose(Z) = waypoint.response.z_way;
        desired_pose(THX) = waypoint.response.thx_way;
        desired_pose(THY) = waypoint.response.thy_way;
        desired_pose(THZ) = waypoint.response.thz_way;
    }
    else {
        std::cout << "Waypoint Server Not Responding\n";
    }
    // ----------------------------------------------------------------------
    // ----------------------------------------------------------------------------------------------------------------------------


    // PID controller object ------------------------------------------------
    pid::rosPID controller(DOF, pose_gain, inte_gain, deriv_gain);
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // ----------------------------------------------------------------------


    // Output to BlueRov ----------------------------------------------------
    std_msgs::UInt16 x_contoller_output;
    std_msgs::UInt16 y_contoller_output;
    std_msgs::UInt16 z_contoller_output;
    std_msgs::UInt16 thx_contoller_output;
    std_msgs::UInt16 thy_contoller_output;
    std_msgs::UInt16 thz_contoller_output;
    // ----------------------------------------------------------------------


    // ROS loop -------------------------------------------------------------
    /* dt included in loop for controller (chrono library) */
    while (ros::ok()) {

        // Time differential per PID step -----------------------------------------------
        dt= std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() *.000000001; // ellapsed time in seconds
        // ------------------------------------------------------------------------------


        // Begin timmer -----------------------------------------------------------------
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        // ------------------------------------------------------------------------------


        // Call PID step ----------------------------------------------------------------
        controller.run_pid(dt,desired_pose,current_pose);
        // ------------------------------------------------------------------------------


        // Store data to send to BlueRov -------------> From: PID <----------------------
        x_contoller_output.data = controller.controller_output(X);
        y_contoller_output.data = controller.controller_output(Y);
        z_contoller_output.data = controller.controller_output(Z);
        thx_contoller_output.data = controller.controller_output(THX);
        thy_contoller_output.data = controller.controller_output(THY);
        thz_contoller_output.data = controller.controller_output(THZ);
        // ------------------------------------------------------------------------------


        // Publish BlueRov output data ---------------> To: BlueRov <--------------------
        x.publish(x_contoller_output);
        y.publish(y_contoller_output);
        z.publish(z_contoller_output);
        roll.publish(thx_contoller_output);
        pitch.publish(thy_contoller_output);
        yaw.publish(thz_contoller_output);
        // ------------------------------------------------------------------------------


        // Determine convergence between current and desired ----------------------------
        // IF CONVERGED -----------------------------------------------------------------
        if (current_pose.isApprox(desired_pose,TOLERANCE)) {

            // Request to server current pose ----------------------------------------------
            waypoint.request.x = current_pose(X);
            waypoint.request.y = current_pose(Y);
            waypoint.request.z = current_pose(Z);
            waypoint.request.thx = current_pose(THX);
            waypoint.request.thy = current_pose(THY);
            waypoint.request.thy = current_pose(THZ);
            waypoint.request.x_vel = current_velo(X_VEL);
            waypoint.request.y_vel = current_velo(Y_VEL);
            waypoint.request.z_vel = current_velo(Z_VEL);
            waypoint.request.thx_vel = current_velo(X_THVEL);
            waypoint.request.thy_vel = current_velo(Y_THVEL);
            waypoint.request.thz_vel = current_velo(Z_THVEL);
            // ------------------------------------------------------------------------------
            

            // Response from server ---------------------------------------------------------
            if (client1.call(waypoint)) {
                desired_pose(X) = waypoint.response.x_way;
                desired_pose(Y) = waypoint.response.y_way;
                desired_pose(Z) = waypoint.response.z_way;
                desired_pose(THX) = waypoint.response.thx_way;
                desired_pose(THY) = waypoint.response.thy_way;
                desired_pose(THZ) = waypoint.response.thz_way;
            }
            else {
                std::cout << "Waypoint Server Not Responding\n";
            }
            // ------------------------------------------------------------------------------
        }
        // -------------------------------------------------------------------------------

        ros::spinOnce();
        loop_rate.sleep();


        // End timmer -------------------------------------------------------------------
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        // ------------------------------------------------------------------------------
    }

    return 0;
}

// Odometry callback function
void odom_callback(const nav_msgs::Odometry& cs) { // cs := current_pose state from bluerov
    current_pose(X) = cs.pose.pose.position.x;
    current_pose(Y) = cs.pose.pose.position.y;
    current_pose(Z) = cs.pose.pose.position.z;
    Eigen::Quaternionf q(cs.pose.pose.orientation.w, cs.pose.pose.orientation.x, cs.pose.pose.orientation.y, cs.pose.pose.orientation.z);
    auto euler = q.toRotationMatrix().eulerAngles(X, Y, Z);
    current_pose(THX) = euler(0);
    current_pose(THY) = euler(1);
    current_pose(THZ) = euler(2);

    current_velo(X_VEL) = cs.twist.twist.linear.x;
    current_velo(Y_VEL) = cs.twist.twist.linear.y;
    current_velo(Z_VEL) = cs.twist.twist.linear.z;
    current_velo(X_THVEL) = cs.twist.twist.angular.x;
    current_velo(Y_THVEL) = cs.twist.twist.angular.y;
    current_velo(Z_THVEL) = cs.twist.twist.angular.z;
}






