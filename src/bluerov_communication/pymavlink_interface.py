"""
ICORE LSU MECS
DATE:  APRIL 6th, 2022
AUTHOR: MASON PESSON
"""

from pymavlink import mavutil,mavwp
import time
import sys
import keyboard
from pymavlink.quaternion import QuaternionBase
import math


"""
XML dialect includes: 
    - Standards:
        minimal.xml
        common.xml
    - Ardusub Specific
        ardupilotmega.xml
        uAvionix.xml
        icarous.xml
    NOTE: Can be found @ https://github.com/mavlink/mavlink/tree/master/message_definitions/v1.0
    ** When parsing the code it is useful to use these xml files to understand why certain functions are called the way they are

    Each xml file contains enums and messages.  Commands are stored in enums and called in messages

Pymavlink:
    Gets created from a code generater (mavgen) along with xml files stored in the mavlink build

NOTE: all calls constructed by mavutil.mavlink call into entries to enums in all included xml files.


"""

class ROVMAV(object):

    def __init__(self):
        """
        INITIATE COMMUNICATION
            Mavlink uses both udpin and udpout.  
                - udpin defines mavlink server... autopilot device i.e. pixhawk on bluerov
                - udpout defines mavlink client ... topside computer (computer you are probably running this from)

            NOTE: The udp port 14550 is the same port that QCG uses to communicate.  Therefore you will hear "communication lost" whenever any object declared with the same port is armed
                - Therefore if not set correctly you will need to set new port.
                IF using udpin:
                    1) Go to webpage: 192.168.2.2:2770/mavproxy
                    2) --out udpin:0.0.0.0:14660 (14660 is defined here because it is what I did, but for each his own)
                    3) Press "Restart MAVProxy"

                IF using udpout:
                    1) Go to webpage: 192.168.2.1:2770/mavproxy
                    2) --out udpin:192.168.2.1:14660 (14660 is defined here because it is what I did, but for each his own)
                    3) Press "Restart MAVProxy"
        """
        self.master = mavutil.mavlink_connection('udpin:192.168.2.1:14660',baudrate=115200)

        """
        INSURE COMMUNICATION
            Although it is not necessary (bcause the server is broadcasting information and does not care who is listening) it is still benificial.
                - If we are not getting heartbeat messages then we are not recieving any other messages
        """
        #master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0)
        self.master.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (self.master.target_system, self.master.target_component))


        self.rc_channel = {"pitch": 1,
                         "roll": 2,
                         "vz": 3,
                         "yaw": 4,
                         "vx": 5,
                         "vy": 6}

        self.boot_time = time.time()


    def acknowledge_command(self):
        """
        ACKNOWLEDGE COMMAND
            acknowledges that the comman that was just sent was recieved.
        """
        ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True).to_dict()
        print(ack_msg)
        print(mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE)
        print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)


    def view_message_loop(self,msg=None,forthismanyiterations=0,atthisrate=0):
        """
        VIEW MESSAGES   NOTE: will loop
            By default a server communicating on a Mavlink channel will broadcast a specific set of mssages.  This fuction will print those messages to the terminal.
                NOTE: Connection obviously required
                NOTE: That messages shown are only limited to only those being broadcasted.  Non default messages must be requested to be broadcasted, and then asked to be printed to terminal to be shown or checked.

            Messages found in XML files (dialect)
                Available messages can be found in the xml files of those used during code generation.  
                i.e. common.xml: https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml
                found between the <messages></messages> tags 

                Ever message is defined with a <message></message> (without the (s)) tag including an id and name attribute. Messages contain field tags (essentially a function parameter) with the folling attributed:    
                    - type
                    - name
                    - enum
                    - display
                    - print format


            USEFUL MESSAGES:
                ATTITUDE
                BATTERY_STATUS['voltages'][0]
                LOCAL_POSITION_NED
                SYSTEM_TIME
                RAW_IMU
                RC_CHANNELS
                MISSION_CURRENT --> gives what seq it is on
                SYS_STATUS['voltage_battery']

            INPUT: 
                msg     type: STRING
                    - set to None by default. This will broadcast all messages
                    - if defined specific message then only that message will be shown on terminal
                forthismanyiterations       type: INT
                    - defines how many loops the while loop will execute before exiting
                    - type: int
                    - if forthismanyiterations is set to 0 then the while loop will repeat forever. This is used for continuous printing it desired.
                atthisrate      type: INT
                    - by default set to 0.1
                    - NOTE: this default will cause the incoming messages to be delayed.  Needs to be set to 0 for there to be no delay
            OUTPUT: 
                - Print to terminal MAVLINK messages being 
                - tp
        """
        it = 0
        while True:
            self.view_message(msg)
            time.sleep(atthisrate) # NOTE: This call could lead to delay in gathering data
            it += 1
            if forthismanyiterations == it:
                break


    def view_message(self,msg=None):
        """
        VIEW MESSAGES   NOTE: will NOT loop
            By default a server communicating on a Mavlink channel will broadcast a specific set of mssages.  This fuction will print those messages to the terminal.
                NOTE: Connection obviously required
                NOTE: That messages shown are only limited to only those being broadcasted.  Non default messages must be requested to be broadcasted, and then asked to be printed to terminal to be shown or checked.

            Messages found in XML files (dialect)
                Available messages can be found in the xml files of those used during code generation.  
                i.e. common.xml: https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml
                found between the <messages></messages> tags 

                Ever message is defined with a <message></message> (without the (s)) tag including an id and name attribute. Messages contain field tags (essentially a function parameter) with the folling attributed:    
                    - type
                    - name
                    - enum
                    - display
                    - print format


            USEFUL MESSAGES:
                ATTITUDE
                BATTERY_STATUS['voltages'][0]
                LOCAL_POSITION_NED
                SYSTEM_TIME
                RAW_IMU
                RC_CHANNELS
                MISSION_CURRENT --> gives what seq it is on
                SYS_STATUS['voltage_battery']

            INPUT: 
                msg     type: STRING
                    - set to None by default. This will broadcast all messages
                    - if defined specific message then only that message will be shown on terminal
                forthismanyiterations       type: INT
                    - defines how many loops the while loop will execute before exiting
                    - type: int
                    - if forthismanyiterations is set to 0 then the while loop will repeat forever. This is used for continuous printing it desired.
                atthisrate      type: INT
                    - by default set to 0.1
                    - NOTE: this default will cause the incoming messages to be delayed.  Needs to be set to 0 for there to be no delay
            OUTPUT: 
                - Print to terminal MAVLINK messages being 
                - tp
        """
        try:
            print(self.master.recv_match(type=msg).to_dict())
        except:
            pass

    
    def request_message(self,msg,msgid,atthisfrequency=1):
        """
        REQUEST MESSAGE
            If message is not appearing when asking for it in view_message() then you me need to request autopilot to broadcast the message.

            NOTE: this functionality will always execute the command sucessfilly as long as the message requested is a valid message; however, this does entirly meean that the 
            message will then be visable.  This is an issue than has been unresolved.  
            
            INPUT:
                msg 
                    - used to call view message function so content requested can potentially be printed to the terminal
                msgid
                    - Enum found in mavlink repository that identifies the message
                    - NOTE: must be defined as MAVLINK_MSG_ID_{msg}
                atthisfrequency
                    - define the frequency at which the message should be brodacast at

        """
        self.master.mav.command_long_send( self.master.target_system, self.master.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msgid, # The MAVLink message ID
                1e6 / atthisfrequency, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
                0, 0, 0, 0, # Unused parameters
                0, ) # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.    
        self.acknowledge_command()
        self.view_message(msg)

    def arm_rov(self):
        """
        ARM ROV
            Current Understanding: Allows motors to move
            NOTE: Messages can still be sent with out arming the rov
            NOTE: No acknowledge function call is needed as  built in acknowledge is used
        """
        # Arm
        # master.arducopter_arm() or:
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1, 0, 0, 0, 0, 0, 0)

        # wait until arming confirmed (can manually check with master.motors_armed())
        print("Waiting for the vehicle to arm")
        self.master.motors_armed_wait()
        print('Armed!')


    def disarm_rov(self):
        """
        DISARM ROV
            Current Understanding: Disables motors from moving
            NOTE: Should be used for saftey precations
            NOTE: No acknowledge function call is needed as  built in acknowledge is used

        """
        # Disarm
        # master.arducopter_disarm() or:
        self.master.mav.command_long_send(self.master.target_system,self.master.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0, 0, 0, 0, 0, 0, 0, 0)

        # wait until disarming confirmed
        self.master.motors_disarmed_wait()
        print("Disarmed")


    def print_available_rov_modes(self):
        """
        PRINT ROV MODES
            Different Modes allow the ROV to opperate differently 
            This may be helpful when looking for modes
        """
        print(list(self.master.mode_mapping().keys()))


    def set_mode(self,mode='STABILIZE'):
        """
        SET MODE of BLUEROV
            Available Modes:
                - STABILIZE
                - ACRO
                - ALT_HOLD
                    - Yaw auto allowed
                - AUTO
                - GUIDED
                - CIRCLE
                - SURFACE
                - POSHOLD
                    - NO YAW auto
                - MANUAL

            NOTE: All modes contain MANUAL functionallity built on top of them except GUIDED mode

        """
        if mode in self.master.mode_mapping():
            # Get mode ID
            mode_id = self.master.mode_mapping()[mode]
            print('MODE ID',mode_id)
            # Set new mode
            """
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
            """
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id)
            
            # Wait for ACK command
            self.acknowledge_command()

        # Check if mode is available
        else:
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(self.master.mode_mapping().keys()))


    def request_parameters_list(self, paramid=None):
        """
        PRINT FULL LIST OF PARAMETERS
            Parameters are used to exchange configuration settings between MAVLink components
             - Each parameter is represented as a key/value pair. The key is a human readable name of the parameter.

            List of available parameters and there meaning can be found here: https://www.ardusub.com/developers/full-parameter-list.html
        """
        self.master.mav.param_request_list_send(
            self.master.target_system, self.master.target_component)
        while True:
            time.sleep(0.01)
            try:
                message = self.master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
                print(message)
                if paramid==None:
                    print('name: %s\tvalue: %d' % (message['param_id'], message['param_value']))
                else:
                    if message['param_id']==paramid:
                        print('name: %s\tvalue: %d' % (message['param_id'], message['param_value']))
            except Exception as error:
                print(error)
                sys.exit(0)


    def request_parameter_value(self,parameter='SURFACE_DEPTH',going_to_change=False):
        """
        PRINT DESIRED PARAMETER VALUE
            Parameters are used to exchange configuration settings between MAVLink components
             - Each parameter is represented as a key/value pair. The key is a human readable name of the parameter.

            List of available parameters and there meaning can be found here: https://www.ardusub.com/developers/full-parameter-list.html
        """
        self.master.mav.param_request_read_send(self.master.target_system, self.master.target_component, parameter.encode(), -1) # .encoded() string is used for char[] data type need (defined in xml)
        message = self.master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
        print('name: %s\tvalue: %d' % (message['param_id'], message['param_value']))
        if going_to_change:
            return message['param_type']


    def set_parameter_value(self,value,parameter='SURFACE_DEPTH'):
        """
        SET VALUE OF PARAMETER
            Parameters are used to exchange configuration settings between MAVLink components
             - Each parameter is represented as a key/value pair. The key is a human readable name of the parameter.

            List of available parameters and there meaning can be found here: https://www.ardusub.com/developers/full-parameter-list.html
        """
        param_type = self.request_parameter_value(parameter,going_to_change=True)
        print('Parameter Changing...')
        self.master.mav.param_set_send(self.master.target_system, self.master.target_component, parameter.encode(), value, param_type)


    ##
    def execute_trajectory(self,lat,lon,alt):
        wp = mavwp.MAVWPLoader()                                                    
        seq = 1
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        radius = .1
        for i in range(len(lat)):                  
                    wp.add(mavutil.mavlink.MAVLink_mission_item_message(self.master.target_system,
                                self.master.target_component,
                                seq,
                                frame,
                                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                0, 0, 0, radius, 0, 0,
                                lat[i],lon[i],alt[i]))
                    seq += 1                                                                       

        self.master.waypoint_clear_all_send()                                     
        self.master.waypoint_count_send(wp.count())                          

        for i in range(wp.count()):
                    msg = self.master.recv_match(type=['MISSION_REQUEST'],blocking=True)  
                    print(self.view_message('MISSION_CURRENT'))           
                    self.master.mav.send(wp.wp(msg.seq))                                                                      
                    print( 'Sending waypoint {0}'.format(msg.seq))                                                  

    ##
    def send_command_long(self):
        self.master.mav.command_long_send(self.master.target_system,self.master.target_component,mavutil.mavlink.MAV_CMD_NAV_PATHPLANNING,0, 0, 0, 0, 1, 1, 1, 1)

        self.acknowledge_command()

    
    def set_rc_channel_pwm(self,channel_id, pwm=1500):
        """ 
        SEND THRUST COMMAND

            WARNING: If not used correctly seriouse dammange can be caused to BLUEROV
            NOTE: Thrust commands range from 1100 - 1900 and are (0) at 1500.   

            INPUT:
                channel_id
                    - BLUEROV uses channels stored in self.rc_channels dictionary.  Mapping is declared there.
                pwm
                    - initially set to 1500... therefore no thrust
        """
        if channel_id < 1 or channel_id > 18:
            print("Channel does not exist.")
            return
        

        rc_channel_values = [65535 for _ in range(18)]
        rc_channel_values[channel_id - 1] = pwm
        self.master.mav.rc_channels_override_send(
            self.master.target_system,                # target_system
            self.master.target_component,             # target_component
            *rc_channel_values)                  # RC channel list, in microseconds.

    
    def manual_control_loop(self):
        while True:
            self.keyboard_controlls()

# Need to make it act as a stream.. or take into account each case... use switch statements
    def keyboard_controlls(self):
        """
        KEYBOARD TO THRUST MAPPING
            Mapping:
                (w s) --> longitudinal movement (x)
                (d a) --> lateral movement (y)

                (l .) --> vertical movement (z)
                (, /) --> yaw movement (omega z)
        """
        thruster_power = 55
        thruster_nominal = 1500

        if keyboard.read_key() == "w":
            self.set_rc_channel_pwm(self.rc_channel['vx'],thruster_nominal+thruster_power)
        elif keyboard.read_key() == "s":
            self.set_rc_channel_pwm(self.rc_channel['vx'],thruster_nominal+thruster_power*-1)
        else:
            print('working')
            self.set_rc_channel_pwm(self.rc_channel['vx'],thruster_nominal)
            
        if not keyboard.is_pressed('q'):
            print("hey")

        if keyboard.read_key() == "d":
            self.set_rc_channel_pwm(self.rc_channel['yaw'],thruster_nominal+thruster_power)
        elif keyboard.read_key() == "a":
            self.set_rc_channel_pwm(self.rc_channel['yaw'],thruster_nominal+thruster_power*-1)
        else:
            self.set_rc_channel_pwm(self.rc_channel['yaw'],thruster_nominal)

        if keyboard.read_key() == "l":
            self.set_rc_channel_pwm(self.rc_channel['vz'],thruster_nominal+thruster_power)
        elif keyboard.read_key() == ".":
            self.set_rc_channel_pwm(self.rc_channel['vz'],thruster_nominal+thruster_power*-1)
        else:
            self.set_rc_channel_pwm(self.rc_channel['vz'],thruster_nominal)

        if keyboard.read_key() == ",":
            self.set_rc_channel_pwm(self.rc_channel['vy'],thruster_nominal+thruster_power)
        elif keyboard.read_key() == "/":
            self.set_rc_channel_pwm(self.rc_channel['vy'],thruster_nominal+thruster_power*-1)
        else:
            self.set_rc_channel_pwm(self.rc_channel['vy'],thruster_nominal)
        

    def set_target_attitude(self,roll, pitch, yaw):
        """ Sets the target attitude while in depth-hold mode.

        'roll', 'pitch', and 'yaw' are angles in degrees.

        """
        self.master.mav.set_attitude_target_send(
            int(1e3 * (time.time() - self.boot_time)), # ms since boot
            self.master.target_system, self.master.target_component,
            # allow throttle to be controlled by depth_hold mode
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
            # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
            QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
            0, 0, 0, 0 # roll rate, pitch rate, yaw rate, thrust
        )

    def test_yaw(self):
        time.sleep(5)
        # Spin in one directioin
        roll_angle = pitch_angle = 0
        for yaw_angle in range(0, 500, 10):
            self.set_target_attitude(roll_angle, pitch_angle, yaw_angle)
            time.sleep(1) # wait for a second

        # spin the other way with 3x larger steps
        for yaw_angle in range(500, 0, -30):
            self.set_target_attitude(roll_angle, pitch_angle, yaw_angle)
            time.sleep(1)

    def set_target_depth(self,depth):
        """ Sets the target depth while in depth-hold mode.

        Uses https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT

        'depth' is technically an altitude, so set as negative meters below the surface
            -> set_target_depth(-1.5) # sets target to 1.5m below the water surface.

        """
        """
        self.master.mav.set_position_target_global_int_send(
            int(1e3 * (time.time() - self.boot_time)), # ms since boot
            self.master.target_system, self.master.target_component,
            coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            type_mask=( # ignore everything except z position
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |#
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            ), lat_int=0, lon_int=0, alt=depth, # (x, y WGS84 frame pos - not used), z [m]
            vx=0, vy=0, vz=0, # velocities in NED frame [m/s] (not used)
            afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
            # accelerations in NED frame [N], yaw, yaw_rate
            #  (all not supported yet, ignored in GCS Mavlink)
        )
        """

        self.master.mav.set_position_target_local_ned_send(
            int(1e3 * (time.time() - self.boot_time)),
            self.master.target_system, self.master.target_component,
            coordinate_frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask=( # ignore everything except z position
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                #mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |#
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            ), x=0,y=0,z=depth,vx=0,vy=0,vz=0,afx=0,afy=0,afz=0,yaw=0,yaw_rate=0
        )

    def mission_count(self):
        self.master.mav.mission_count_send(self.master.target_system, self.master.target_component,1)
        #
        self.view_message_loop('MISSION_ACK')
        # self.acknowledge_command()

if __name__ == "__main__":
    """
    Documentation of using code:

        Understanding how pymavlink is created:
            - Pymavlink is createed
    """

    rovmav = ROVMAV()
    rovmav.mission_count()

    #rovmav.request_message()
    #rovmav.view_message_loop('LOCAL_POSITION_NED',atthisrate=0)
    #rovmav.request_parameters_list()
    #rovmav.set_parameter_value(-10,'SURFACE_DEPTH')
    #rovmav.request_parameter_value('SURFACE_DEPTH')

    #rovmav.arm_rov()
    #rovmav.set_mode('GUIDED')

    #rovmav.set_target_depth(3)
    #print('mason')
    #rovmav.view_message_loop('COMMAND_ACK',atthisrate=0)
    #rovmav.test_yaw()
    #rovmav.disarm_rov()

    #rovmav.manual_control_loop()

    #rovmav.request_message('AHRS2',mavutil.mavlink.MAVLINK_MSG_ID_AHRS2)

    #rovmav.send_command_long()
    #rovmav.set_parameter_value(0.8,'PSC_VELXY_D')
    #rovmav.request_parameter_value('PSC_VELZ_P')

    #rovmav.disarm_rov()