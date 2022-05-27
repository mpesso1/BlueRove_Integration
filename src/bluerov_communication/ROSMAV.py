#!/usr/bin/env python3

"""
ICORE LSU MECS
DATE:  APRIL 6th, 2022
AUTHOR: MASON PESSON
"""

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Byte
from pymavlink_interface import ROVMAV, keyboard, mavutil
from blue_rov_custom_integration.srv import * #byte_update, byte_updateResponse
import time

bitmap_enum = {
    "RESET": 0,        # 1: healthy, 0: fault NOTE: this bit it additionally used to trigger the state controler to reset... i.e. whenver the cv system detects an additional object in its trajectory
    "MANUAL": 1,         # 1: healthy, 0: fault
    "MODE": 2,        # 1: healthy, 0: fault
    "ROVMAV_HEALTH": 3,     # 1: healthy, 0: fault
    "PID_HEALTH": 4,              # 1: STABILITY, 0: POSHOLD
    "PP_HEALTH": 5,            # 1: Allow manual control while executing trajectory, 0: don't ** Considering manual control will be availible no mater what state the robot is in. It may be benificial to determine a new meaning for this bit
    "SYS_HEALTH": 6}             # 1: True, 0: False


system_state_byte = '0000000'       # according to ROSMAV and subsequently all other system nodes, system byte is initially set to 0

system_state_bit_array = []     # Array storing each value from the bitmap


def string_byte_to_array(string_byte=system_state_byte):        # helper function to convert a byte string to array
    a = []
    for d in str(string_byte): # for each index within the system byte, append that value onto the storage array
        a.append(int(d))
    return a
def array_to_num(binary_array):     # helper function to convert the array of bit values to a number
    return sum([j*(2**i) for i,j in list(enumerate(reversed(binary_array)))])
def int_to_binary(data):        # Helper function to convert the sytem number to binary
    return '{0:07b}'.format(data)


system_state_bit_array = string_byte_to_array(system_state_byte)
system_state_num = array_to_num(system_state_bit_array)


MODE='POSHOLD'              # initially set mode to POSHOLD on startup according to ROSMAV
CURRENT_MODE = MODE         # do not need to set the mode on startup because it will be set to POSHOLD on shutdown


# PID thrust data
PID_THRUST_X = 0 #UNITS: none 
PID_THRUST_Y = 0 #UNITS: none
PID_THRUST_Z = 0 #UNITS: none
PID_THRUST_YAW = 0 #UNITS: none

# Offset incoming odom
OFFSET_X = 0 #UNITS: m
OFFSET_Y = 0 #UNITS: m
OFFSET_Z = 0 #UNITS: m
OFFSET_YAW = 0 #UNITS: rad

# Offset ONS on startup
INIT_OFFSET_LINEAR_ONS = True
INIT_OFFSET_ANGULAR_ONS = True


def byte_update_callback(req):
    '''
    subscriber callback to update system byte. data is coming from ROSHUM
    '''
    global system_state_num, system_state_byte, system_state_bit_array
    system_state_num = req.sys_byte_num
    system_state_byte = int_to_binary(system_state_num)
    system_state_bit_array = string_byte_to_array(system_state_byte)
    return byte_updateResponse(True)

def RESET_SYSTEM_withkey(key):
    '''
    reset system byte
    '''
    if keyboard.is_pressed(key):
        system_state_bit_array[bitmap_enum['RESET']]
def RESET_system_check():
    '''
    check if reset of system byta has been called
    '''
    global system_state_bit_array, system_state_num, system_state_byte, bitmap_enum
    if system_state_bit_array[bitmap_enum["RESET"]]:
        system_state_num = 0
        system_state_byte = int_to_binary(system_state_num)
        system_state_bit_array = string_byte_to_array(system_state_byte)
        system_state_bit_array[bitmap_enum["RESET"]] = 0
        

rovmav = ROVMAV()       # pymavlink class used to access the mavlink protocal for communicating with ROV

def resetTrigger_callback(req):
    '''
    service callback to reset system byte.  data is coming from lawn_mower node
    '''
    global reset_ONS
    if req.reset:
        reset_ONS = True
    
    return resetTriggerResponse(True)


def rc_overide(data):
    '''
    subscriber callback used to update thrust and mode data.  data coming from pid node
    '''
    global PID_THRUST_X, PID_THRUST_YAW, PID_THRUST_Y, PID_THRUST_Z, MODE
    global system_state_byte, system_state_num, system_state_bit_array
    global reset_ONS

    if not rovmav.Manual_overide: 

        # Mode to system byte mapping set by pid
        MODE = data.header.frame_id
        if MODE == "POSHOLD":
            MODE = "POSHOLD"
            system_state_bit_array[bitmap_enum['MODE']] = 0
        elif MODE == "STABILIZE":
            system_state_bit_array[bitmap_enum['MODE']] = 1
        else:
            print('\033[1;31m INCORRECT MODE SET \033[m \n\n')

        # system byte to thrust mapping
        if system_state_bit_array[bitmap_enum['MODE']] == 1:
            PID_THRUST_X = data.pose.pose.position.x
            PID_THRUST_Y = data.pose.pose.position.y
            PID_THRUST_Z = data.pose.pose.position.z
            PID_THRUST_YAW = data.pose.pose.orientation.z
        else:
            PID_THRUST_X = 0
            PID_THRUST_Y = 0
            PID_THRUST_Z = 0
            PID_THRUST_YAW = 0

    # true if pid is finished with all wps
    if data.pose.pose.orientation.x == 1:
        reset_ONS = True

    # print("x: ", PID_THRUST_X)
    # print("Y: ", PID_THRUST_Y)
    # print("Z: ", PID_THRUST_Z)
    # print("YAW: ", PID_THRUST_YAW)

reset_ONS = False # logic bit used to reset system byte

# ros
rospy.init_node("ROSMAV")   
publish_odometry = rospy.Publisher("ROV_ODOMETRY",Odometry,queue_size=10)
publish_state = rospy.Publisher("SYSTEM_STATE",Byte,queue_size=10)
sub = rospy.Subscriber("ROV_RC_COMMANDS",Odometry,rc_overide)
server = rospy.Service("byte_update",byte_update,byte_update_callback)
resetTriggerServer = rospy.Service("resetTrigger",resetTrigger,resetTrigger_callback)
rate = rospy.Rate(10)   # UNITS: hz
odomdata = Odometry()   # odom message object


def linear_offset(linear_data,offset):
    '''
    Linear offset definition
    '''
    return linear_data - offset
def yaw_offset(yaw_data):
    '''
    Yaw offset definition
    '''
    if yaw_data != abs(yaw_data):
        return (2*3.14+yaw_data)
    else:
        return yaw_data
def update_linear_offset(offset_x, offset_y, offset_z):
    '''
    Store offset data into global offset variables
    '''
    global OFFSET_X, OFFSET_Y, OFFSET_Z

    OFFSET_X = offset_x
    OFFSET_Y = offset_y
    OFFSET_Z = offset_z
def update_angular_offset(offset_yaw):
    '''
    Store yaw offset data into global angular offset variables
    '''
    global OFFSET_YAW

    OFFSET_YAW = offset_yaw


# initial request for odom message transfer speed
rovmav.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,11,display_interval=False)
rovmav.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,11,display_interval=False)
update_message_request = 0  # used to continuously remind bluerov of message speed.  updates around 50 iterations


while not rospy.is_shutdown():

    if reset_ONS:
        
        rovmav.keyboard_controlls(pid_thrust_x=0,pid_thrust_y=0,pid_thrust_z=0,pid_thrust_yaw=0,letsride=True)
        time.sleep(.5)
        rovmav.keyboard_controlls(pid_thrust_x=0,pid_thrust_y=0,pid_thrust_z=0,pid_thrust_yaw=0,letsride=True)
        system_state_bit_array[bitmap_enum["RESET"]] = 1
        reset_ONS = False

    RESET_system_check() # Check if system bytes RESET bit has gone high.  If so, continue with reset sequence... setting system byte to 0

    rovmav.INIT_STARTUP_withkey('0',MODE)    # Arming the system with whatever mode the system is currently in

    rovmav.INIT_SHUTDOWN_withkey('o')    # Disarming the system and putting the ROV into POSHOLD mode

    if rovmav.system_armed:     # Set high by the INIT_STARTUP key and set low with INIT_SHUTDOWN key

        system_state_bit_array[bitmap_enum['ROVMAV_HEALTH']] = 1       # Initiate startup sequence... let ROSHUM know that ROV is armed and recieved a heartbeat
        system_state_num = array_to_num(system_state_bit_array)

        if system_state_num >= 15: # 0b00001111 or greater      Return of the startup sequence... each node and therefore the system has been defined as healthy

            if update_message_request >= 50:

                rovmav.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,11,display_interval=False)        # Updated request for ATTITUDE message to be sent at a specific rate 
                #rovmav.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,1000,display_interval=False)      # Updated request for LOCAL_POSITION_NED to be sent at a specific rate
                update_message_request=0

            linear_pos = rovmav.view_message('LOCAL_POSITION_NED',returndata=True,display_output=False)     # Store incoming LOCAL_POITION_NED data coming in from ROV
            angular_pose = rovmav.view_message('ATTITUDE',returndata=True,display_output=False)     # Store incoming ATTITUDE data coming in from the ROV

            odomdata.header.frame_id = "HEALTHY" # ROSMAV communicates to pid that it is healthy.. NOTE: This data is used as a handshake between ROSMAV and the pid for the pid to initiate

            if linear_pos != None:      # While loop loops at a faster rate than than the rate of incoming messages from ROV. Only take in date if it is other that None
                
                if rovmav.INIT_OFFSET_withkey('h') or INIT_OFFSET_LINEAR_ONS:     # Initiate linear offset sequence with desired key.  Will take the current pose of the robot and set that linear data as the origin.

                    update_linear_offset(linear_pos['x'],linear_pos['y'],linear_pos['z']) #                         >>>>>>>>>>>>>        linear_pos['z']                          
                    INIT_OFFSET_LINEAR_ONS = False

                # Store ROS data
                odomdata.pose.pose.position.x = linear_offset(linear_pos["x"],OFFSET_X)   # UNITS: m
                odomdata.pose.pose.position.y = linear_offset(linear_pos["y"],OFFSET_Y)   # UNITS: m
                odomdata.pose.pose.position.z =  linear_offset(linear_pos["z"],OFFSET_Z)   # UNITS: m              >>>>>>>>               linear_pos["z"]
                odomdata.twist.twist.linear.x = linear_pos["vx"]  # UNITS: m/s
                odomdata.twist.twist.linear.y = linear_pos["vy"]  # UNITS: m/s
                odomdata.twist.twist.linear.z = linear_pos["vz"]  # UNITS: m/s             >>>>>>>>>>>>>>>            linear_pos["vz"]

                # print("X: ",odomdata.pose.pose.position.x)
                # print("Y: ",odomdata.pose.pose.position.y)
                #print("Z: ",odomdata.pose.pose.position.z)

                #print(odomdata.twist.twist.linear.x)

            if angular_pose != None:        # While loop loops at a faster rate than than the rate of incoming messages from ROV. Only take in date if it is other that None
                
                if rovmav.INIT_OFFSET_withkey('j') or INIT_OFFSET_ANGULAR_ONS:     # Initiate angular offset sequence with desires key.  Will zero the current yaw of the robot.

                    update_angular_offset(angular_pose['yaw'])
                    INIT_OFFSET_ANGULAR_ONS = False

                # Store ROS data
                odomdata.pose.pose.orientation.x = angular_pose["roll"]   # UNITS: deg
                odomdata.pose.pose.orientation.y = angular_pose["pitch"]   # UNITS: deg
                odomdata.pose.pose.orientation.z = yaw_offset(angular_pose["yaw"])  # UNITS: deg       rad  [0 - 2pi]
                odomdata.pose.pose.orientation.w = angular_pose["yaw"]  # UNITS: deg    rad [-pi - pi]

                # print("YAW_Z: ",odomdata.pose.pose.orientation.z)
                # print("YAW_W: ",odomdata.pose.pose.orientation.w)

            rovmav.MODE_CHANGE_ONS_withkey("9")     # Manually set the ROV into Stabilize mode blocking any output from the pid.  NOTE: Can only be set if system is healthy

            update_message_request += 1

            publish_odometry.publish(odomdata)


        else:
            odomdata.header.frame_id = "UNHEALTHY"      # ROSMAV communicates to pid that it is unhealthy.. NOTE: This data is used as a handshake between ROSMAV and the pid for the pid to initiate

    if not rovmav.system_armed and system_state_num > 0:        # Part of the disarm sequence check
        system_state_bit_array[bitmap_enum["RESET"]] = 1

    rovmav.MODE_CHANGE_RELEASE_ONS_withkey('8')     # Release the manual set made with MODE_CHANGE_ONS

    if rovmav.Manual_overide:       # Sequence if mode was set manually
        MODE = 'STABILIZE'
        system_state_bit_array[bitmap_enum['MODE']] = 1
        PID_THRUST_X = 0
        PID_THRUST_Y = 0
        PID_THRUST_Z = 0
        PID_THRUST_YAW = 0

    if CURRENT_MODE != MODE:        # Set mode only if mode has changed

        if system_state_bit_array[bitmap_enum['MODE']] == 0:
            rovmav.set_mode('POSHOLD')

        if system_state_bit_array[bitmap_enum['MODE']] == 1:
            rovmav.set_mode('STABILIZE')
            
    CURRENT_MODE = MODE

    system_state_num = array_to_num(system_state_bit_array)     # Update system number

    if system_state_num == 31: # 0011111 --> system must be healthy and in STABILIZE mode inorder to move  NOTE: Ca become 31 by either Manually setting of if pid initiates autonomous rc override
        rovmav.keyboard_controlls(pid_thrust_x=PID_THRUST_X,pid_thrust_y=PID_THRUST_Y,pid_thrust_z=PID_THRUST_Z,pid_thrust_yaw=PID_THRUST_YAW,letsride=True)
    else:
        rovmav.keyboard_controlls(pid_thrust_x=0,pid_thrust_y=0,pid_thrust_z=0,pid_thrust_yaw=0,letsride=False)
    
    
    publish_state.publish(array_to_num(system_state_bit_array))     # Communicate system changes to ROSHUM

    rate.sleep()