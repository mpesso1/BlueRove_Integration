#!/usr/bin/env python

"""
ICORE LSU MECS
DATE:  APRIL 6th, 2022
AUTHOR: MASON PESSON
"""

from http import server
from tkinter import CURRENT
import rospy
import time

from nav_msgs.msg import Odometry
from std_msgs.msg import Byte

from pymavlink_interface import ROVMAV

from blue_rov_custom_integration.srv import byte_update, byte_updateResponse

"""
ROVMAV

    DESCRIPTION: Links the pymavlink interface that can communicate with the BlueRov to our ROS system. 

    COMMUNICATION:
        
        1.) _ROSMAV_ w/ _ROSHUM_ through SYSTEM_STATE topic
            data:
                - system byte status

            where data?:
                - the ROVMAV_healthy bit gets updated in the system byte status and defines if the messages are being recieved from the bluerov
                - the MODE bit gets updated through communication with the pid node
        
        2.) _ROSMAV_ publishes to pp_pid through ROV_ODOMETRY topic
            data: 
                - current state...
                - x, y, z, yaw, thx, thy, thz
                - vx, vy, vz

            where data?:
                - coming from pymavlink which is communicating a malink message contianing LOCAL_POSITION_NED cordinates

        3.) _ROSMAV_ subscribes to pp_pid through ROV_RC_COMMANDS topic
            data:
                - rc thrust commands
                - system MODE NOTE: system mode does not actually get set unitl pid has been allowed to therefore it is important for ROV to initially be set to POSHOLD mode

            where data?:
                - comes from computation done in pid

    NOTE: The ROVMAV node plays and important role in the working of the spread out state controler running the ROS ecosystem.
    It acts as a funnel of data coming through the path planner and the pid.  Its job is to take in this information and report it back 
    up to ROSHUM as a system byte.
    NOTE: ROVMAV also plays an important part in the start up sequence of the ROS system. This will become more clear after understandng the ROSHUM node.
    However, on start up the system byte should be set to 0, and in orer for it to come out of 0 the ROSMAV healthy bit must first turn 1.  Therfore before any other 
    process can start the BlueRov must be fully communicating with ROS.
    NOTE: For the the LOCAL_POSITION_NED message to send the rov must first be armed. Need to press the button 1 on keyboard
"""

"""
Dictionary used to link each index of bitmap value to its meaning
    - Use this as a reference to understand the meaning behind each bit
    - NOTE: ROSHUM should contain a more indepth description behind the use of each bit
"""
bitmap_enum = {
    "SYS_HEALTH": 0,        # 1: healthy, 0: fault NOTE: this bit it additionally used to trigger the state controler to reset... i.e. whenver the cv system detects an additional object in its trajectory
    "PP_HEALTH": 1,         # 1: healthy, 0: fault
    "PID_HEALTH": 2,        # 1: healthy, 0: fault
    "ROVMAV_HEALTH": 3,     # 1: healthy, 0: fault
    "MODE": 4,              # 1: STABILITY, 0: POSHOLD
    "MANUAL": 5,            # 1: Allow manual control while executing trajectory, 0: don't ** Considering manual control will be availible no mater what state the robot is in. It may be benificial to determine a new meaning for this bit
    "RESET": 6}             # 1: True, 0: False

# Binary value defining the state of the system
system_state_byte = '0000000'       # according to ROSMAV, system byte is initially set to 0

# Array storing each value from the bitmap
system_state_bit_array = []

# helper function to convert a byte string to array
def string_byte_to_array(string_byte=system_state_byte):
    a = []
    for d in str(string_byte): # for each index within the system byte, append that value onto the storage array
        a.append(int(d))
    return a

# helper function to convert the array of bit values to a number
def array_to_num(binary_array):
    return sum([j*(2**i) for i,j in list(enumerate(reversed(binary_array)))])

def int_to_binary(data):
    return '{0:08b}'.format(data)


system_state_bit_array = string_byte_to_array()
system_state_num = array_to_num(system_state_bit_array)

MODE='POSHOLD'              # initially set mode to POSHOLD on startup according to ROSMAV
CURRENT_MODE = MODE 

# Global variables that get filled through pid callback and used in ros while loop to send thrust commands
PID_THRUST_X = 0
PID_THRUST_Y = 0
PID_THRUST_Z = 0
PID_THRUST_YAW = 0

def rc_overide(data):
    global PID_THRUST_X
    global PID_THRUST_Y
    global PID_THRUST_Z
    global PID_THRUST_YAW
    global MODE

    """
    The PID node is responsible for determining the mode the robot should in 
    it is the job of both ROVMAV and the path planner node to feed it information to make this decision
    """
    # ROV mode
    MODE = data.header.frame_id
    if MODE == "POSHOLD":
        system_state_bit_array['MODE'] = 0
    if MODE == "STABILITY":
        system_state_bit_array['MODE'] = 1
    else:
        print("INCORRECT MODE SET")

    # PID thrust data
    # Regardless of the data being recevied and stored in these variable. This data will not be sent over to the Bluerov unless other system critia is met
    PID_THRUST_X = data.pose.pose.position.x
    PID_THRUST_Y = data.pose.pose.positoin.y
    PID_THRUST_Z = data.pose.pose.position.z
    PID_THRUST_YAW = data.pose.pose.orientation.z

def byte_update_callback(req):
    global system_state_num, system_state_byte, system_state_bit_array
    system_state_num = req.sys_byte
    system_state_byte = int_to_binary(system_state_num)
    system_state_bit_array = string_byte_to_array(system_state_byte)



# Create Node
rospy.init_node("ROSMAV")

# Publish
publish_odometry = rospy.Publish("ROV_ODOMETRY",Odometry,queue_size=10)
publish_state = rospy.Publish("SYSTEM_STATE",Byte,queue_size=10)

# Subscribe
sub = rospy.Subscriber("ROV_RC_COMMANDS",Odometry,rc_overide)

# Service
server = rospy.Service("byte_update",byte_update,byte_update_callback)

# Loop Rate
rate = rospy.Rate(10) # UNITS: hz

# odom message object
odomdata = Odometry()

# pymavlink class used to access the mavlink protocal for communicating with ROV
rovmav = ROVMAV()

while not rospy.is_shutdown():

    """
    Inorder for system byte to initiate the system must first be armed
    NOTE: By initiating startup sequence you are setting the ROV into initial MODE settings (POSHOLD) / arming the system / and setting the system_armed bit to true inorder to initiate system sequence
    """
    rovmav.INIT_STARTUP_withkey('0',MODE)
    rovmav.INIT_SHUTDOWN_withkey('-')


    """
    LOCAL_POSITION_NED is the fused local cordinates of the robot defined in NED cordinates. (z is in the downward direction)
    NOTE: As long as the robot is turned on and connected to the DVL the LOCAL_POSTION_NED message will be broadcated... does not matter if it is armed
    """
    linear_pos = rovmav.view_message('LOCAL_POSITION_NED',returndata=True)
    if linear_pos != None and rovmav.system_armed:

        # System must first be armed and recieving data
        system_state_bit_array["ROVMAV_HEALTH"] = 1
        system_state_num = array_to_num(system_state_bit_array)

        if system_state_num >= 15: # 0b00001111 or greater 

            # ROVMAV recieving data / is armed / and has sucsesfully changed system byte to indicate the system is healthy
            odomdata.header.frame_id = "HEALTHY" # ROSMAV communicates to pid that it is healthy

            # linear data
            odomdata.pose.pose.position.x = linear_pos["x"]   # UNITS: m
            odomdata.pose.pose.position.y = linear_pos["y"]   # UNITS: m
            odomdata.pose.pose.position.z = linear_pos["z"]   # UNITS: m
            odomdata.twist.twist.linear.x = linear_pos["vx"]  # UNITS: m/s
            odomdata.twist.twist.linear.y = linear_pos["vy"]  # UNITS: m/s
            odomdata.twist.twist.linear.z = linear_pos["vz"]  # UNITS: m/s

            # angular data
            angular_pose = rovmav.view_message('AAAA',returndata=True) # ** NOTE: need to find correct angular data
            odomdata.pose.pose.orientation.x = angular_pose["x"]   # UNITS: deg
            odomdata.pose.pose.orientation.y = angular_pose["y"]   # UNITS: deg
            odomdata.pose.pose.orientation.z = angular_pose["z"]   # UNITS: deg

        else:
            # Either not recieving data / not armed / or system is not healthy
            odomdata.header.frame_id = "UNHEALTHY"

        publish_odometry.publish(odomdata)


    # Incoming Data from PID
    if CURRENT_MODE != MODE:
        if system_state_bit_array['MODE'] == 0:
            rovmav.set_mode('POSHOLD')
        if system_state_bit_array['MODE'] == 1:
            rovmav.set_mode('STABILIZE')
    CURRENT_MODE = MODE

    # the only bit that can change is the ROVMAV_healthy bit
    system_state_num = array_to_num(system_state_bit_array)

    # following waypoints. still have manual interjections
    if system_state_num == 31: # 0011111 --> system must be healthy and in STABILIZE mode inorder to move
        rovmav.keyboard_controlls(pid_thrust_x=PID_THRUST_X,pid_thrust_y=PID_THRUST_Y,pid_thrust_z=PID_THRUST_Z,letsride=True)
    else:
        rovmav.keyboard_controlls() # can arm and disarm motors with keyboard number 1 and 2


    # Send data to ROSHUM
    publish_state.publish(array_to_num(system_state_bit_array))


    rospy.sleep()