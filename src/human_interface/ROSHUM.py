#!/usr/bin/env python3

"""
ICORE LSU MECS
DATE:  APRIL 6th, 2022
AUTHOR: MASON PESSON
"""

import rospy
from std_msgs.msg import Byte
from blue_rov_custom_integration.srv import *
import keyboard

# bitmap used to link each index of bitmap value to its meaning... use this to understand what each bit does
bitmap_enum = {
    "RESET": 0,        # 1: healthy, 0: fault NOTE: this bit it additionally used to trigger the state controler to reset... i.e. whenver the cv system detects an additional object in its trajectory
    "MANUAL": 1,         # 1: healthy, 0: fault
    "MODE": 2,        # 1: healthy, 0: fault
    "ROVMAV_HEALTH": 3,     # 1: healthy, 0: fault
    "PID_HEALTH": 4,              # 1: STABILITY, 0: POSHOLD
    "PP_HEALTH": 5,            # 1: Allow manual control while executing trajectory, 0: don't ** Considering manual control will be availible no mater what state the robot is in. It may be benificial to determine a new meaning for this bit
    "SYS_HEALTH": 6}             # 1: True, 0: False


# INPUT FROM ROVMAV
system_binary_input = '0000000'       # Binary value defining the state of the system.  According to ROSHUM is initially set to 0.
system_binary_array_input = []        # Array storing each value from the bitmap

# CURRENT/LAST VALUES FROM LAST INPUT FROM ROVMAV
system_binary_current = '0000000'     # Binary value defining the state of the system.  According to ROSHUM is initially set to 0.
system_binary_array_current = []      # Array storing each value from the bitmap
system_num_current = 0                # int representing the binary value

system_num_last = 0                   # According to ROSHUM initially set to 0 to match what the initial ROSHUM binary value is

# helper function to convert a byte string to array
def string_byte_to_array(string_byte):
    a = []
    for d in str(string_byte):
        a.append(int(d))
    return a

# helper function to convert the bit array to a number
def array_to_num(binary_array):
    return sum([j*(2**i) for i,j in list(enumerate(reversed(binary_array)))])

def int_to_binary(data):
    return '{0:07b}'.format(data)


# Object Cordinates
Ox = 2.66194  # UNITS: m
Oy = -1.52111  # UNITS: m
Oz = 1.76056  # UNITS: m
# // 2.66194
# // -1.52111
# // 1.76056

# WPs
X_WP = []  # UNITS: m
Y_WP = []  # UNITS: m
Z_WP = []  # UNITS: m
YAW_WP = []   # UNTIS: rad

wpIDX = 0   # WP index

# system/wp logic
TOTAL_WAYPOINTS = 0  
MISSION_COMPLETE_FLAG = False 
NO_MORE_WAYPOINTS = False

reset_pathplanner_NEED_NEW_WP = False

# WP text manipulation
def WPfile_empty():
    '''
    empty wp text file
    '''
    with open('/home/mason/catkin_ws/src/blue_rov_custom_integration/src/human_interface/readwaypoint.txt', 'w') as f:
        f.write('')
def WPsys_reset():
    '''
    empty all wp data holders for incoming wp txt info
    '''
    global X_WP, Y_WP, Z_WP, YAW_WP
    global TOTAL_WAYPOINTS, wpIDX, MISSION_COMPLETE_FLAG, NO_MORE_WAYPOINTS
    global system_binary_input, system_binary_array_input, system_binary_current, system_binary_array_current, system_num_current, system_num_last


    # WPs
    X_WP = []  # UNITS: m
    Y_WP = []  # UNITS: m
    Z_WP = []  # UNITS: m
    YAW_WP = []   # UNTIS: rad

    wpIDX = 0   # WP index

    # system/wp logic
    TOTAL_WAYPOINTS = 0  
    MISSION_COMPLETE_FLAG = False 
    NO_MORE_WAYPOINTS = False

    # INPUT FROM ROVMAV
    system_binary_input = '0000000'       # Binary value defining the state of the system.  According to ROSHUM is initially set to 0.
    system_binary_array_input = []        # Array storing each value from the bitmap

    # CURRENT/LAST VALUES FROM LAST INPUT FROM ROVMAV
    system_binary_current = '0000000'     # Binary value defining the state of the system.  According to ROSHUM is initially set to 0.
    system_binary_array_current = []      # Array storing each value from the bitmap
    system_num_current = 0                # int representing the binary value

    system_num_last = 0                   # According to ROSHUM initially set to 0 to match what the initial ROSHUM binary value is
def WPfile_read():
    global X_WP, Y_WP, Z_WP, YAW_WP
    global TOTAL_WAYPOINTS, MISSION_COMPLETE_FLAG, NO_MORE_WAYPOINTS

    WPsys_reset() # reset wp configuration data

    # Upload waypoints from text file:   <x> <y> <z> <yaw>
    with open('/home/mason/catkin_ws/src/blue_rov_custom_integration/src/human_interface/readwaypoint.txt') as f:
        num_waypoints = 0
        for line in f:
            s = line.split()
            if len(s) == 4:
                X_WP.append(float(s[0])) # UNIT: m
                Y_WP.append(float(s[1])) # UNIT:m
                Z_WP.append(float(s[2])) # UNIT: m
                if abs(float(s[3])) != float(s[3]):
                    YAW_WP.append(2*3.14 + float(s[3])) # UNIT: deg
                else:
                    YAW_WP.append(float(s[3])) # UNIT: deg
                num_waypoints += 1
            else:
                print("\033[5;41m WAYPOINT text file must be formated as <x> <y> <z> <thz> or no input \033[m")
        TOTAL_WAYPOINTS = num_waypoints

    if TOTAL_WAYPOINTS == 0: 
        print("\033[7;30m NO MISSION SET --> waypoint text file was defined as empty. \033[m")
        NO_MORE_WAYPOINTS = True
        MISSION_COMPLETE_FLAG = True
    else:
        NO_MORE_WAYPOINTS = False
        MISSION_COMPLETE_FLAG = False

    print(MISSION_COMPLETE_FLAG)

    WPfile_empty()


with open('/home/mason/catkin_ws/src/blue_rov_custom_integration/src/human_interface/readwaypoint.txt') as f:
    num_waypoints = 0
    for line in f:
        s = line.split()
        if len(s) == 4:
            X_WP.append(float(s[0])) # UNIT: m
            Y_WP.append(float(s[1])) # UNIT:m
            Z_WP.append(float(s[2])) # UNIT: m
            YAW_WP.append(float(s[3])) # UNIT: deg
            num_waypoints += 1
        else:
            print("\033[5;41m WAYPOINT text file must be formated as <x> <y> <z> <thz> or no input \033[m")
    TOTAL_WAYPOINTS = num_waypoints

if TOTAL_WAYPOINTS == 0: 
    print("\033[7;30m NO MISSION SET --> waypoint text file was defined as empty. \033[m")
    NO_MORE_WAYPOINTS = True
    MISSION_COMPLETE_FLAG = True




# Global variable used to determine if the cv system has communicated that a new trajectory needs to be determined
cv_said_new_path = False


initial_message = """
           \033[109;46m      JKS (Just Keep Swimming)      \033[m


                \033[4;7m SYSTEM CONTROLS (Manual)\033[m
\033[1;34m   [0] --> Arm       [o] --> Disarm        [9] --> Stabilize Mode \033[m
\033[1;34m        [8] --> POsehold Mode \033[m


                \033[4;7m SYSTEM CONTROLS (Autonomous)\033[m
\033[1;34m   [h] --> Reset Linear Pose        [j] --> Set Rotational Offset \033[m
\033[1;34m   n --> Define Dy        [v] --> Define Dx, Dz \033[m
\033[1;34m   [b] --> Return to Origin        [r] --> Run System \033[m
\033[1;34m           [y] --> Reset configuration data \033[m


                \033[4;7m POSITION CONTROLS \033[m
\033[1;34m   [w],[s] --> Forward, Backward      [a],[d] --> CW Rotate, CCW Rotate \033[m
\033[1;34m   [l],[].] --> Up, Down               [,],[/] --> Left, Right \033[m



\033[2;33m    
    Mode: POSEHOLD
    Keyboard: Disabled until mode set to Stabilize [9] and armed [0]
    System: If waypoints defined then system will 
            opperate autonomously \033[m
"""

print(initial_message)

obj_ONS = True

def update_system_state(data):

    # Include global variables
    global obj_ONS
    global system_binary_input
    global system_binary_array_input
    global system_binary_array_current
    global system_num_current
    global bitmap_enum
    global cv_said_new_path
    global X_WP, Y_WP, Z_WP, YAW_WP, Ox, Oy, Oz
    global MISSION_COMPLETE_FLAG
    global wpIDX
    global system_num_last
    global NO_MORE_WAYPOINTS, TOTAL_WAYPOINTS
    global reset_pathplanner_NEED_NEW_WP

    # Convert system_num to binary string
    system_binary_input = int_to_binary(data.data)

    # Store binary value in an array
    system_binary_array_input = string_byte_to_array(system_binary_input)
    
    # Action on changed binary value
    if data.data != None: # if data is actually being recieved 
        
        if data.data != system_num_last and data.data <= 15: # if system number has changed and not in stabilize mode (0001111)
            
            print("looping")
            system_num_last = data.data

            # Initiate communication with Path Planner
            rospy.wait_for_service('pp_system_control')

            # Define object used to communicate through service
            pp_server = rospy.ServiceProxy("pp_system_control",control_pathplanner)

            ask_if_new_path_needed = True # always ask for new path if system byte has changed

            pp_response = pp_server(ask_if_new_path_needed,cv_said_new_path,reset_pathplanner_NEED_NEW_WP)  # cv_said_new_path coming from cv node and communicated through cv service
            reset_pathplanner_NEED_NEW_WP = False
            
            # Set path planner healthy bit true if path planner says so
            if pp_response.pp_healthy: 
                system_binary_array_input[bitmap_enum['PP_HEALTH']] = 1          # set to true if we can communicate
                # Response from path planner defining that 1 of the 3 new path posibilities are true
                if pp_response.need_new_path:
                    if not MISSION_COMPLETE_FLAG:
                        # Initiate communication with path planner over new service
                        rospy.wait_for_service('pp_waypoint')

                        # Define object used to communicate over service
                        waypoint_server = rospy.ServiceProxy("pp_waypoint",pathplanner_update_waypoint) 
                        
                        waypoint_response = waypoint_server(X_WP[wpIDX],Y_WP[wpIDX],Z_WP[wpIDX],YAW_WP[wpIDX],Ox,Oy,Oz) # see if possible.. includeother parameters in input of callback will possibly want to change these to arrays of values

                        if not pp_response.cv_enforced: # incrament the waypoint if the new traj is not because of new object defined by cv
                            wpIDX = wpIDX + 1
                            # print("Waypoint index incremented")
                            if wpIDX >= TOTAL_WAYPOINTS:
                                NO_MORE_WAYPOINTS = True

                        # Initiate communication with pid
                        rospy.wait_for_service('pid_system_control')

                        # Define object used to communicate over service
                        pid_server = rospy.ServiceProxy("pid_system_control",control_pid) 
                        turn_pid_on = False
                        turn_pid_off = not turn_pid_on
                        pid_response = pid_server(turn_pid_on, turn_pid_off, True) # last argument indicates if a new path is being generated
                        
                        # need to reset what cv said even if it didnt say anyhting in the first place.. will use the system health bit to denote that cv said something
                        cv_said_new_path = False 

                        # Set PID healthy bit true it pid says so
                        if pid_response.pid_healthy:
                            system_binary_array_input[bitmap_enum['PID_HEALTH']] = 1
                    else:
                        # Initiate communication with pid
                        rospy.wait_for_service('pid_system_control')

                        # Define object used to communicate over service
                        pid_server = rospy.ServiceProxy("pid_system_control",control_pid) 
                        turn_pid_on = False
                        turn_pid_off = not turn_pid_on
                        pid_response = pid_server(turn_pid_on, turn_pid_off, True) # last argument indicates if a new path is being generated
                        
                        # need to reset what cv said even if it didnt say anyhting in the first place.. will use the system health bit to denote that cv said something
                        cv_said_new_path = False 

                        # Set PID healthy bit true it pid says so
                        if pid_response.pid_healthy:
                            system_binary_array_input[bitmap_enum['PID_HEALTH']] = 1

                else: 
                    if not MISSION_COMPLETE_FLAG:
                        pid_server = rospy.ServiceProxy("pid_system_control",control_pid) 
                        turn_pid_on = True
                        turn_pid_off = not turn_pid_on
                        pid_response = pid_server(turn_pid_on,turn_pid_off, True) # last argument indicates if a new path is being generated

                        if pid_response.pid_healthy:
                            system_binary_array_input[bitmap_enum['PID_HEALTH']] = 1
                        if NO_MORE_WAYPOINTS:
                            MISSION_COMPLETE_FLAG = True
                            # print("Mission Complete flag set to true")

                    else:
                        # Initiate communication with pid
                        rospy.wait_for_service('pid_system_control')

                        # Define object used to communicate over service
                        pid_server = rospy.ServiceProxy("pid_system_control",control_pid) 
                        turn_pid_on = False
                        turn_pid_off = not turn_pid_on
                        pid_response = pid_server(turn_pid_on, turn_pid_off, True) # last argument indicates if a new path is being generated
                        
                        # need to reset what cv said even if it didnt say anyhting in the first place.. will use the system health bit to denote that cv said something
                        cv_said_new_path = False 

                        # Set PID healthy bit true it pid says so
                        if pid_response.pid_healthy:
                            system_binary_array_input[bitmap_enum['PID_HEALTH']] = 1

            # if all nodes are healthy then system is healthy
            if system_binary_array_input[bitmap_enum['PID_HEALTH']] == 1 and system_binary_array_input[bitmap_enum['PP_HEALTH']] == 1 and system_binary_array_input[bitmap_enum['ROVMAV_HEALTH']] == 1:
                system_binary_array_input[bitmap_enum['SYS_HEALTH']] = 1

            # Update the current state of the system byte
            system_binary_array_current = system_binary_array_input
            system_num_current = array_to_num(system_binary_array_current)

            rospy.wait_for_service('byte_update')
            ROV_client = rospy.ServiceProxy("byte_update",byte_update)
            ROVMAV_response = ROV_client(system_num_current)

            if ROVMAV_response.recieved:
                pass
                # print(system_num_current)
                # print(system_num_last)
                # print(system_binary_array_input)
                
    else:
        print("\033[1;33m WARNING: Not recieving messages from ROVMAV... value is None \033[m \n\n") 
        
    if keyboard.is_pressed('t') and obj_ONS:
        print("T WAS PRESSED")
        cv_said_new_path = True
        
# // 5.00971
# // -2.5467
# // 2.32233
        
        Ox = 5.00971
        Oy = -3 #-2.5467
        Oz = 1.85 #2.32233
        
        obj_ONS = False
        
        rospy.wait_for_service('pid_system_control')

        # Define object used to communicate over service
        pid_server = rospy.ServiceProxy("pid_system_control",control_pid) 
        turn_pid_on = False
        turn_pid_off = not turn_pid_on
        pid_response = pid_server(turn_pid_on, turn_pid_off, True) # last argument indicates if a new path is being generated
        
        wpIDX = wpIDX - 1
        MISSION_COMPLETE_FLAG = False 
        NO_MORE_WAYPOINTS = False

        rospy.wait_for_service('byte_update')
        ROV_client = rospy.ServiceProxy("byte_update",byte_update)
        ROVMAV_response = ROV_client(0)
        





def cv_action_callback(req):
    global cv_said_new_path
    # global Ox
    # global Oy
    # global Oz

    global system_binary_array_current
    system_binary_array_current[bitmap_enum['SYS_HEALTH']] = 0

    cv_said_new_path = True

    # Ox = req.x
    # Oy = req.y
    # Oz = req.z

    return cv_actionResponse(True)


rospy.init_node("ROSHUM")

def wpTrigger_callback(req):
    global reset_pathplanner_NEED_NEW_WP
    if req.checkWPtext:
        WPfile_read()

    reset_pathplanner_NEED_NEW_WP = True

    return wpTriggerResponse(True)

# communicating directly with ROVMAV node on the complete cycled output of the system.  NEED TO MAKE SURE I AM APPROPRIATLY UPDATING THE BITMASK
system_subscriber = rospy.Subscriber("SYSTEM_STATE",Byte,update_system_state) # need to test how these added parameters work

cv_server = rospy.Service("cv_action_service",cv_action,cv_action_callback) # server to cv system for if it detects additional object in path all it will do is set fault in system byte for it to recalculate new path with updates object waypoint

wpTriggerServer = rospy.Service("wpTrigger",wpTrigger,wpTrigger_callback)


rospy.spin()