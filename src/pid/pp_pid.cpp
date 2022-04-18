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

/* 
pp_node
    Communication w/ ROSMAV 
        input: ROV_ODOMETRY topic
        output: ROV_RC_COMMANDS topic 
            --> both defined in ROSMAV node

    Communication w/ ROSHUM
        in/out: pid_system_control
            data:
                - PID_ON
                - PID_OFF
                - NEWTRAJ

            where data?: Data gets configured in the ROSHUM node and then sent here for controlling the pid

        NOTE: pid becomes healthy as long as it can communicate with the ROSHUM node
        NOTE: ROSMAV_HEALTHY determines 
            - if the pid can accept odom data coming in
            - if the pid can run
        NOTE: PID_ON / PID_OFF determines if the pid can run and defined in ROSHUM
        NOTE: NEW_TRAJ determines if path can be generated and initiates pid sequence

    Communication w/ pp_goPath through service_line_waypoint Service
        data: 
                float64 x
                float64 y
                float64 z           Request defines the current pose coming from ROV_ODOMETRY
                float64 thx
                float64 thy
                float64 thz
                float64 x_vel
                float64 y_vel
                float64 z_vel
                float64 thx_vel
                float64 thy_vel
                float64 thz_vel
                ---
                float64 x_way
                float64 y_way
                float64 z_way       Response defines the desired pose defined by pp
                float64 thx_way
                float64 thy_way
                float64 thz_way

        

*/

// COMMUNICATION VARIABLES / STATE VARIABLES ----------
bool PID_ON = false;
bool PID_OFF = true;
bool PID_HEALTHY = false;
std::string ROVMAV_HEALTHY = "UNHEALTHY";
bool NEW_TRAJ = false;
bool cv_object_detected_reset = false;
int thrust_nominal = 1500;


// ENUM DEFINING DEGREES OF FREEDOM -------------------
enum Cordinates {X, Y, Z, THX, THY, THZ}; // Position
enum Velocity {X_VEL, Y_VEL, Z_VEL, X_THVEL, Y_THVEL, Z_THVEL}; // Velocity



// CONTROL VARIABLES ---------------
const int DOF = 6; // Degrees of Freedom
const float TOLERANCE = .0001; // UNITS: m --> defines how close the desired_pose and current_pose states need to be to one another
float dt; // UNITS: s --> time difference defined by chrono library



// PID Gains Initialized ------------------------------
Eigen::Matrix<float,1,6> pose_gain;
Eigen::Matrix<float,1,6> inte_gain;
Eigen::Matrix<float,1,6> deriv_gain;


// only get called from ROSHUM node on system byte change
bool pid_control_action(blue_rov_custom_integration::control_pid::Request &req, blue_rov_custom_integration::control_pid::Response &res) {
    PID_ON = req.PID_on;
    PID_OFF = req.PID_off;
    NEW_TRAJ = req.New_Traj;

    if (req.cv_initiate_reset){
        cv_object_detected_reset = true;
    }
    res.pid_healthy = true;
}


// ----- States Initialized ----------------------------
Eigen::Matrix<float,6,1> desired_pose; // from path planner
Eigen::Matrix<float,6,1> current_pose; // from bluerov / mavlink interface
Eigen::Matrix<float,6,1> current_velo; // from bluerov / mavlink interface


// Odometry Callback -----------------------------------
// gets called from ROSMAV
void odom_callback(const nav_msgs::Odometry& cs);


// ** MAIN ** ------------------------------------------ *********
int main(int argc, char** argv) {

    ros::init(argc,argv,"Controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(10); // communication frequency [hz]

    // Publishing TOPICS -----------------> To: Robot <----------------------
    ros::Publisher rc_commands = n.advertise<nav_msgs::Odometry>("ROV_RC_COMMANDS",1000);

    // Subscribing TOPICS -----------------> From: ROSMAV <-----------------
    ros::Subscriber odom = n.subscribe("ROV_ODOMETRY",1000,odom_callback); // nav_msgs/Odometry  :  Defines current_* 

    // Client SERVICE ---------------------> Server: pp_goPath <-----------
    blue_rov_custom_integration::update_waypoint waypoint;
    ros::ServiceClient client1 = n.serviceClient<blue_rov_custom_integration::update_waypoint>("service_line_waypoint");

    // Server SERVICE --------------------> Server: ROSHUM <---------------
    ros::ServiceServer pid_server = n.advertiseService("pid_system_control", &pid_control_action);

    // PID GAIN -- (x, y, z, thx, thy, thz) ---------------------------------
    pose_gain << .01, .01, .01, .01, .01, .01;
    inte_gain << .01, .01, .01, .01, .01, .01;
    deriv_gain << .01, .01, .01, .01, .01, .01;

    // PID controller object ------------------------------------------------
    pid::rosPID controller(DOF, pose_gain, inte_gain, deriv_gain);
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // ----------------------------------------------------------------------

    nav_msgs::Odometry thrust;

    // ROS loop -------------------------------------------------------------
    /* dt included in loop for controller (chrono library) */
    while (ros::ok()) {
        
         // Time differential per PID step -----------------------------------------------
        dt= std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() *.000000001; // ellapsed time in seconds
        // ------------------------------------------------------------------------------

        // Begin timmer -----------------------------------------------------------------
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        // ------------------------------------------------------------------------------

        /*
        ROVMAV_HEALTHY gets communicated over ROV_ODOMETRY and PID_ON gets communicated over through ROVHUM... additionally NEW_TRAJ will get 
        turned on from service between this node and ROSHUM.. communication with the path planner leads up to this
        */
        if (ROVMAV_HEALTHY=="HEALTHY" && PID_ON==true) { // ROVMAV_HEALTHY does not become healthy until pid pp and ROVMAV health in system byte all get set set high... Need system byte to be set to 15 or greater
            if (NEW_TRAJ) { 
                // Request to server current pose ----------------------------------------------
                waypoint.request.NewTraj = true;
                waypoint.request.x = current_pose(X); 
                waypoint.request.y = current_pose(Y);
                waypoint.request.z = current_pose(Z);
                waypoint.request.thx = current_pose(THX);
                waypoint.request.thy = current_pose(THY); // ** The reason we are sending the path planner our current position is because it will need it to generate the trajectory
                waypoint.request.thy = current_pose(THZ);
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
                }
                else {
                    std::cout << "Waypoint Server Not Responding\n";
                }
                NEW_TRAJ = false;
            }

            // Call PID step ----------------------------------------------------------------
            controller.run_pid(dt,desired_pose,current_pose);


            thrust.header.frame_id = "STABILITY";
            thrust.pose.pose.position.x = controller.controller_output(X);
            thrust.pose.pose.position.y = controller.controller_output(Y);
            thrust.pose.pose.position.z = controller.controller_output(Z);
            thrust.pose.pose.orientation.z = controller.controller_output(THZ);
            rc_commands.publish(thrust);

            if (current_pose.isApprox(desired_pose,TOLERANCE)) {
                waypoint.request.NewTraj = false;
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
            }           
        }
        else { // system healthy but PID is not on
            thrust.header.frame_id = "POSHOLD";
            thrust.pose.pose.position.x = thrust_nominal;
            thrust.pose.pose.position.y = thrust_nominal;
            thrust.pose.pose.position.z = thrust_nominal;
            thrust.pose.pose.orientation.z = thrust_nominal;
            rc_commands.publish(thrust);
        }

        ros::spinOnce();
        loop_rate.sleep();


        // End timmer -------------------------------------------------------------------
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    }

    return 0;
}

// Odometry callback function
void odom_callback(const nav_msgs::Odometry& cs) { // cs := current_pose state from bluerov
    ROVMAV_HEALTHY = cs.header.frame_id; 

    if (ROVMAV_HEALTHY == "HEALTHY") {
        current_pose(X) = cs.pose.pose.position.x; // UNITS: m
        current_pose(Y) = cs.pose.pose.position.y; // UNITS: m
        current_pose(Z) = cs.pose.pose.position.z; // UNITS: m

        current_velo(X_VEL) = cs.twist.twist.linear.x; // UNTIS: m/s
        current_velo(Y_VEL) = cs.twist.twist.linear.y; // UNTIS: m/s
        current_velo(Z_VEL) = cs.twist.twist.linear.z; // UNTIS: m/s

        current_pose(THX) = cs.pose.pose.orientation.x; // UNITS: deg
        current_pose(THY) = cs.pose.pose.orientation.y; // UNITS: deg
        current_pose(THZ) = cs.pose.pose.orientation.z; // UNITS: deg

        current_velo(X_THVEL) = cs.twist.twist.angular.x; // UNTIS: deg/s
        current_velo(Y_THVEL) = cs.twist.twist.angular.y; // UNTIS: deg/s
        current_velo(Z_THVEL) = cs.twist.twist.angular.z; // UNTIS: deg/s
    }
}






