#!/usr/bin/env python3

from os import system
import rospy
from pymavlink_interface import ROVMAV, keyboard, mavutil
from nav_msgs.msg import Odometry

PID_THRUST_X=0
PID_THRUST_Y=0
PID_THRUST_Z=0
PID_THRUST_YAW=0

OFFSET_X = 0
OFFSET_Y = 0
OFFSET_Z = 0
OFFSET_YAW = 0

INIT_OFFSET_LINEAR_ONS = True
INIT_OFFSET_ANGULAR_ONS = True


def rc_callback(data):
    global PID_THRUST_Z,PID_THRUST_X,PID_THRUST_Y,PID_THRUST_YAW

    PID_THRUST_X = data.pose.pose.position.x
    PID_THRUST_Y = data.pose.pose.position.y
    PID_THRUST_Z = data.pose.pose.position.z
    PID_THRUST_YAW = data.pose.pose.orientation.z


rospy.init_node("pid_interface")
sub_rc = rospy.Subscriber("thrust_commands",Odometry,rc_callback)
pud_odom = rospy.Publisher("odom",Odometry,queue_size=10)
pub_odom_plot = rospy.Publisher("odom_plot",Odometry,queue_size=10)
rate = rospy.Rate(10)  


rovmav = ROVMAV()

odomdata = Odometry()   # odom message object


def linear_offset(linear_data,offset):      # Linear offset definition
    return linear_data - offset

def yaw_offset(yaw_data):        # Yaw offset definition
    if yaw_data != abs(yaw_data):
        return (6.28318530718+yaw_data)
    else:
        return yaw_data

def update_linear_offset(offset_x, offset_y, offset_z):     # Store offset data into global offset variables
    global OFFSET_X, OFFSET_Y, OFFSET_Z
    
    OFFSET_X = offset_x
    OFFSET_Y = offset_y
    OFFSET_Z = offset_z

def update_angular_offset(offset_yaw):      # Store yaw offset data into global angular offset variables
    global OFFSET_YAW

    OFFSET_YAW = offset_yaw


system_armed = False
rovmav.set_mode('STABILIZE') # initially set mode to STABILIZE
hold_pos = False

begin = False
begin_ONS = False


while not rospy.is_shutdown():

    time = rospy.get_rostime()

    if keyboard.is_pressed('0') and not system_armed:
        rovmav.arm_rov()

        rovmav.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,1000,display_interval=False)        # Initial requenst to the ROV for ATTITUDE data to be sent at a spicific rate
        rovmav.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,1000,display_interval=False)      # Initial reqiuest to the ROV for LOCAL_POSITION_NED data to be sent at a specific rate

        system_armed = True

    if keyboard.is_pressed('o') and system_armed:
        rovmav.disarm_rov()
        system_armed = False

    if keyboard.is_pressed('9') and hold_pos:
        rovmav.set_mode('STABILIZE')
        hold_pos = False

    if keyboard.is_pressed('8') and not hold_pos:
        rovmav.set_mode('POSHOLD')
        hold_pos = True

    if keyboard.is_pressed('1') and not begin_ONS:
        print("start")

        update_linear_offset(linear_pos['x'],linear_pos['y'],linear_pos['z'])
        
        begin = True
        begin_ONS = True

    if keyboard.is_pressed('2') and begin_ONS:
        print('stop')
        begin = False
        begin_ONS = False

        
    linear_pos = rovmav.view_message('LOCAL_POSITION_NED',returndata=True,display_output=False)     # Store incoming LOCAL_POITION_NED data coming in from ROV
    angular_pose = rovmav.view_message('ATTITUDE',returndata=True,display_output=False)     # Store incoming ATTITUDE data coming in from the ROV

    if linear_pos != None:      # While loop loops at a faster rate than than the rate of incoming messages from ROV. Only take in date if it is other that None
        if rovmav.INIT_OFFSET_withkey('h') or INIT_OFFSET_LINEAR_ONS:     # Initiate linear offset sequence with desired key.  Will take the current pose of the robot and set that linear data as the origin.

            update_linear_offset(linear_pos['x'],linear_pos['y'],linear_pos['z'])
            INIT_OFFSET_LINEAR_ONS = False

        # Store ROS data
        odomdata.pose.pose.position.x = linear_offset(linear_pos["x"],OFFSET_X)   # UNITS: m
        odomdata.pose.pose.position.y = linear_offset(linear_pos["y"],OFFSET_Y)   # UNITS: m
        odomdata.pose.pose.position.z = linear_offset(linear_pos["z"],OFFSET_Z)   # UNITS: m
        odomdata.twist.twist.linear.x = linear_pos["vx"]  # UNITS: m/s
        odomdata.twist.twist.linear.y = linear_pos["vy"]  # UNITS: m/s
        odomdata.twist.twist.linear.z = linear_pos["vz"]  # UNITS: m/s
        odomdata.header.stamp = time

    if angular_pose != None:        # While loop loops at a faster rate than than the rate of incoming messages from ROV. Only take in date if it is other that None
                
        if rovmav.INIT_OFFSET_withkey('j') or INIT_OFFSET_ANGULAR_ONS:     # Initiate angular offset sequence with desires key.  Will zero the current yaw of the robot.

            update_angular_offset(angular_pose['yaw'])
            INIT_OFFSET_ANGULAR_ONS = False

        # Store ROS data
        odomdata.pose.pose.orientation.x = angular_pose["roll"]   # UNITS: rad
        odomdata.pose.pose.orientation.y = angular_pose["pitch"]   # UNITS: rad
        odomdata.pose.pose.orientation.z = yaw_offset(angular_pose["yaw"])  # UNITS: rad  [0 - 2pi]
        odomdata.pose.pose.orientation.w = angular_pose["yaw"] # UNITS: rad [-pi - pi]
        odomdata.header.stamp = time

        #print(odomdata.pose.pose.orientation.z)

    pub_odom_plot.publish(odomdata)

    # else:                                             # If angular data starts acting up may want to uncomment this
    #     odomdata.header.seq = 1

    if system_armed and begin:
        pud_odom.publish(odomdata)
        rovmav.keyboard_controlls(pid_thrust_x=PID_THRUST_X,pid_thrust_y=PID_THRUST_Y,pid_thrust_z=PID_THRUST_Z,pid_thrust_yaw=PID_THRUST_YAW,letsride=True)

    else:
        rovmav.keyboard_controlls(letsride=True)

   
    rate.sleep()