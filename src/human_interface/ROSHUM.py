#!/usr/bin/env python

"""
ICORE LSU MECS
DATE:  APRIL 6th, 2022
AUTHOR: MASON PESSON
"""

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray

from blue_rov_custom_integration.srv import control_pid, control_pathplanner, cv_action, cv_actionResponse, pathplanner_update_waypoint, byte_update

"""
ROSHUM

    Contains the logic necessary for the ROV to act correctly upon start up, in normal operation, and if an error is ocuring the correct prcedure will occure.
    Also responsible for taking in the waypoints in waypoints.txt file to send to the path planner 

    COMMUNICATION:
        1.) ROSHUM (client) w/ pp_goPath (server) over pp_system_control Service
            data:
                bool ask_if_new_path_needed
                bool cv_said_new_path
                bool plot_traj              communicaiton necessary inorder to tell the pp that it needs to compute a new path and get feedback on why it is building a new path
                ---                         also used to define if the pid should be on or off
                bool need_new_path
                bool pp_healthy
                bool cv_enforced

        2.) ROSHUM (client) w/ pp_pid (server) over pid_system_control
            data:
                bool PID_on
                bool PID_off
                bool New_Traj               communication needed in order to turn on or block the pid from operating.  NOTE also that the pid is what turns the path planner on in the first place
                ---
                bool action_taken
                bool pid_healthy

        3.) ROSHUM (client) x/ pp_goPath (server) over pp_waypoint
            data: 
                float64 x
                float64 y
                float64 z
                float64 yaw
                float64 Ox                  necessary info inorder to communicate to the path planner the new waypoint to go to or the new objet to move around
                float64 Oy
                float64 Oz
                ---
                bool traj_ready

"""

"""Global variables to store object or pose waypoints"""
Ox = None  # x object cordinate UNITS: m
Oy = None  # y object cordinate UNITS: m
Oz = None  # z object cordinate UNITS: m

X_WP = []  # x cordinate waypoints UNITS: m
Y_WP = []  # y cordinate waypoints UNITS: m
Z_WP = []  # z cordinate waypoints UNITS: m
YAW_WP = []   # yaw cordinate waypoints UNTIS: deg

TOTAL_WAYPOINTS = 0   # index on num of waypoints to determine when path is finished
wpIDX = 0   # index to cycle through waypoints 

MISSION_COMPLETE_FLAG = False # used to denote when all goal waypoints have been met i.e. wpIDX = TOTAL_WAYPOINTS

"""
Upload waypoints from text file
<x> <y> <z> <yaw>
"""
with open('waypoints.txt') as f:
    num_waypoints = 0
    for line in f:
        s = line.split()
        if len(s) == 4:
            X_WP.append(s[0])
            Y_WP.append(s[1])
            Z_WP.append(s[2])
            YAW_WP.append(s[3])
        else:
            print("waypoint definition does not contain 4 inputs")
        num_waypoints += 1
    TOTAL_WAYPOINTS = num_waypoints

if TOTAL_WAYPOINTS == None:
    print("No mission set")
    MISSION_COMPLETE_FLAG = True


# bitmap used to link each index of bitmap value to its meaning... use this to understand what each bit does
bitmap_enum = {
    "SYS_HEALTH": 0,    # 1: healthy, 0: fault NOTE: this bit it additionally used to trigger the state controler to reset... i.e. whenver the cv system detects an additional object in its trajectory
    "PP_HEALTH": 1,     # 1: healthy, 0: fault
    "PID_HEALTH": 2,    # 1: healthy, 0: fault
    "ROVMAV_HEALTH": 3,  # 1: healthy, 0: fault 
    "MODE": 4,          # 1: STABILITY, 0: POSHOLD
    "MANUAL": 5,        # 1: Allow manual control while executing trajectory, 0: don't
    "RESET": 6}    # 1: True, 0: False

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
    return '{0:08b}'.format(data)


# Global variable used to determine if the cv system has communicated that a new trajectory needs to be determined
cv_said_new_path = False


"""
Callback concerning the SYSTEM_STATE topic used to communicate directly with ROSMAV node

NOTE: The ROSMAV mode is important because it is the last stop to the system... think of this Node as the Gulf of Mexico is related to the Mississippi River... erveryhting funnels down to the gulf
It contains all the feeback related to system performance.  Think of the ROSHUM node as being upstream and the ROSMAV node as being downstream... We use
downstram information upstream to make dicisions for the system. These decisions then effect downstream and we get feedback.

NOTE: Action only takes place if the bitmap defining the system has changed from its previous state.  There are few situations when the bitmap should change.
    1.) On startup of the system
    2.) Whenever CV determines that the path needs to be reconstructed
    3.) Any possible fault. --> faults get passed down through the system to ROVMAV
    4.) Possibly on shutdown... may not be an issue

NOTE: All communication that is taking place is through the use of ROS services and the SYSTEM_STATE topic that is streaming system information from ROSMAV.  
That is, all services are being controlled though the following callback function.  

List of Services: NOTE: All clients here

    1.) Service: pp_system_control
            - Client --> ROSHUM 
            - Server --> pp_goPath
            - srv: control_pathplanner

    2.) Service: pid_system_control
            - Client --> ROSHUM
            - Server --> pp_pid
            - srv: control_pid

    3.) Service: pp_waypoint
            - Client --> ROSHUM
            - Server --> pp_goPath

NOTE: All adjustments to the system byte are computed on the input variable and then transfered to current variable after completion of the callback
"""
def update_system_state(data):

    # Include global variables
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

    # Convert system_num to binary string
    system_binary_input = int_to_binary(data.data)
    
    # Store binary value in an array
    system_binary_array_input = string_byte_to_array(system_binary_input)
    
    # Action on changed binary value
    if data.data != None: # if data is actually being recieved 
        if data.data != system_num_last: # if system number has changed

            system_num_last = data.data

            # Initiate communication with Path Planner
            rospy.wait_for_service('pp_system_control')

            # Define object used to communicate through service
            pp_server = rospy.ServicePrOxy("pp_system_control",control_pathplanner)

            ask_if_new_path_needed = True # always ask for new path if system byte has changed

            """
            Reasons a new path may need to be genreated:
                1.) There is currently no waypoint set e.g. the system is in startup
                2.) The preexisting waypoint is exahsted meaning that the robot has reached it
                3.) The CV system has determined an unexpected object in its view and would like pp to generate a new path around it

            NOTE: possiblities 1 and 2 will need to be checked within the path planner and possibility 3 is a hard reset meaning that it will recompute the path no matter what
            """
            pp_response = pp_server(ask_if_new_path_needed,cv_said_new_path)  # cv_said_new_path coming from cv node and communicated through cv service

            # Set path planner healthy bit true if path planner says so
            if pp_response.pp_healthy: 

                system_binary_array_input['PP_HEALTH'] = 1          # set to true if we can communicate

                # Response from path planner defining that 1 of the 3 new path posibilities are true
                if pp_response.need_new_path:

                    # Initiate communication with path planner over new service
                    rospy.wait_for_service('pp_waypoint')

                    if not pp_response.cv_enforced: # incrament the waypoint if the new traj is not because of new object defined by cv
                        wpIDX = wpIDX + 1
                        if wpIDX >= TOTAL_WAYPOINTS:
                            MISSION_COMPLETE_FLAG = True

                    # Define object used to communicate over service
                    waypoint_server = rospy.ServicePrOxy("pp_waypoint",pathplanner_update_waypoint) 
                    
                    # Waypoints will be read from the file defining how the robot will travel area
                    # Object will be directly read from CV input
                    # Depending on whats values true the path planner will either use the xyzyaw values or the OxOyOz values
                    waypoint_response = waypoint_server(X_WP[wpIDX],Y_WP[wpIDX],Z_WP[wpIDX],YAW_WP[wpIDX],Ox,Oy,Oz) # see if possible.. includeother parameters in input of callback will possibly want to change these to arrays of values

                    """
                    Now that the path planner has been envoked we will intiate the PID to turn on
                    NOTE: The on and off of the PID is set as a push button meaning that if you turn it on you must then turn it off.
                    It will not turn off after you stop telling it to turn on
                    """
                    # Initiate communication with pid
                    rospy.wait_for_service('pid_system_control')

                    # Define object used to communicate over service
                    pid_server = rospy.ServicePrOxy("pid_system_control",control_pid) 
                    turn_pid_on = False
                    turn_pid_off = not turn_pid_on
                    pid_response = pid_server(turn_pid_on, turn_pid_off)
                    
                    # need to reset what cv said even if it didnt say anyhting in the first place.. will use the system health bit to denote that cv said something
                    cv_said_new_path = False 

                    # Set PID healthy bit true it pid says so
                    if pid_response.pid_healthy:
                        system_binary_array_input['PID_HEALTH'] = 1

                else: # No need to turn on the pid but still need to know the health of the pid.  
                    pid_server = rospy.ServicePrOxy("pid_system_control",control_pid) 
                    turn_pid_on = True
                    turn_pid_off = not turn_pid_on
                    pid_response = pid_server(turn_pid_on,turn_pid_off)

                    if pid_response.pid_healthy:
                        system_binary_array_input['PID_HEALTH'] = 1

            # if all nodes are healthy then system is healthy
            if system_binary_array_input['PID_HEALTH'] == 1 and system_binary_array_input['PP_HEALTH'] == 1 and system_binary_array_input['ROVMAV_HEALTH'] == 1:
                system_binary_array_input['SYS_HEALTH'] = 1

            # Update the current state of the system byte
            system_binary_array_current = system_binary_array_input
            system_num_current = array_to_num(system_binary_array_current)
        else:
            print('SYSTEM HEALTHY: ', system_binary_array_current)
    else:
        print("WARNING: Not recieving messages from ROVMAV... value is returning None") 

    rospy.wait_for_service('byte_update')
    ROV_client = rospy.ServiceProxy("byte_update",byte_update)
    ROVMAV_response = ROV_client(system_num_current) 
                    
def cv_action_callback(req):
    global cv_said_new_path
    global Ox
    global Oy
    global Oz

    global system_binary_array_current
    system_binary_array_current['SYS_HEALTH'] = 0

    cv_said_new_path = True

    Ox = req.x
    Oy = req.y
    Oz = req.z

    return cv_actionResponse(True)



rospy.init_node("ROSHUM")

# communicating directly with ROVMAV node on the complete cycled output of the system.  NEED TO MAKE SURE I AM APPROPRIATLY UPDATING THE BITMASK
system_subscriber = rospy.Subscriber("SYSTEM_STATE",String,update_system_state) # need to test how these added parameters work

cv_server = rospy.Service("cv_action_service",cv_action,cv_action_callback) # server to cv system for if it detects additional object in path all it will do is set fault in system byte for it to recalculate new path with updates object waypoint

rospy.spin()