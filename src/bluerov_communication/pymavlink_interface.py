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

        wp = mavwp.MAVWPLoader()

        self.system_armed = False

        self.startup_ONS = False

        self.MODE_ONS = True

        self.Manual_overide = False

        self.ARM_ONS = True

    def acknowledge_command(self):
        """
        ACKNOWLEDGE COMMAND
            acknowledges that the comman that was just sent was recieved.
        """
        ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True).to_dict()
        print("Command Successful")
        #print(ack_msg)
        #print(mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE)
        #print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)


    def view_message_loop(self,msg=None,forthismanyiterations=0,atthisrate=0,stopafterone=False,returndata=False):
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
        # Need to check added functionality... added stopafterone to see only the fist message that was sent.  This is for when messages are only expected to be sent one time and the while loop can end
        # also added return data to return the data gathered so further checks can be made depending on the message chooses.  Also may want to see if instead of using break can just use return
        output = None
        it = 0
        while True:
            try:
                output = self.master.recv_match(type=msg).to_dict()
                print(output)
                if stopafterone:
                    break
            except:
                pass
            time.sleep(atthisrate) # NOTE: This call could lead to delay in gathering data
            it += 1
            if forthismanyiterations == it:
                break

        if returndata:
            return output


    def view_message(self,msg=None,returndata=False,display_output=True):
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
        output = None
        try:
            output = self.master.recv_match(type=msg).to_dict()
            if display_output:
                print(output) # need t comment this out when actually implimenting
        except:
            pass
        if returndata and output != None:
            return output
    
    def request_message_interval(self,msgid,atthisfrequency=10,msg=None,display_interval=True): # Default is 10 messages per second
        """
        REQUEST MESSAGE --> NOTE: you are actually also setting the frequency at which the message gets sent... this is mportant
            If message is not appearing when asking for it in view_message() then you me need to request autopilot to broadcast the message.

            NOTE: this functionality will always execute the command sucessfilly as long as the message requested is a valid message; however, this does entirly meean that the 
            message will then be visable.  This is an issue than has been unresolved.  
            
            INPUT:
                msg 
                    - used to call view message function so content requested can potentially be printed to the terminal
                msgid
                    - Enum found in mavlink repository that identifies the message
                    - NOTE: must be defined as mavutil.mavlink.MAVLINK_MSG_ID_{msg} *** 
                atthisfrequency
                    - define the frequency at which the message should be brodacast at

        """
        self.master.mav.command_long_send( self.master.target_system, self.master.target_component, 
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 
                0,
                msgid, # The MAVLink message ID
                1e6 / atthisfrequency, # The interval between two messages in microseconds. data needs to be sent in micro(hz) hence the 1e6
                0, 
                0, 
                0, 
                0, # Unused parameters
                0) # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.    
        #self.acknowledge_command()
        #self.view_message(msg)
        if display_interval:
            self.get_message_interval(msgid)

        if msg != None:
            print('looking for message... if not appearing after sometime then message is unavailable now')
            self.view_message_loop(msg,stopafterone=True)


    def get_message_interval(self,msg):
        """
        NOTE: msg must be defned as must be defined as MAVLINK_MSG_ID_{message} *** cn be found in minimun.xml
        """
        self.master.mav.command_long_send( self.master.target_system, self.master.target_component, 
            mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL, 
            0,
            msg, # The MAVLink message ID
            0, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
            0, 
            0, 
            0, 
            0, # Unused parameters
            0) # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.    

        # NOTE: Do not call an acknowledge function is you are trying to see a particular message... Just look for the particuar message you are after
        # response should be found in a MESSAGE_INTERVAL message
        data = self.view_message_loop('MESSAGE_INTERVAL',stopafterone=True,returndata=True)
        print('Interval between sent messages [s]: ', data['interval_us']*1e-6)


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
        self.system_armed = True


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
        self.system_armed = False


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


            # try different way of setting guided mode: MASON
            #mavutil.mavfile.set_mode(self.master,'GUIDED',0,0)

            # Wait for ACK command
            print("mode change")
            self.acknowledge_command()

            self.current_mode = mode

        # Check if mode is available
        else:
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(self.master.mode_mapping().keys()))

    # result: 3
    def enable_guided_mode(self):
        self.master.mav.command_long_send(self.master.target_system,self.master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE,
        0, # may want to try changing this value to 1... do not dig into why
        1, # 1 := Enable , 0 := Disable
        0,
        0,
        0,
        0,
        0,
        0)
    # result: 3
    def set_guided_master(self):
        self.master.mav.command_long_send(self.master.target_system,self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_GUIDED_MASTER,
            0,
            self.master.target_system,
            self.master.target_component,
            0,
            0,
            0,
            0,
            0)

    # result: 3
    def set_guided_limits(self):
        self.master.mav.command_long_send(self.master.target_system,self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_GUIDED_LIMITS,
            0,
            1.5, # timeout in seconds
            0, # min altitude --> 0 means no min altitude
            0, # max altitude --> 0 means no max altitue
            0, # horizontal limit --> 0 means not horizontal limit
            0,
            0,
            0)


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
        """
        Control loop used to maualy control bluerov 
        """
        while True:
            self.keyboard_controlls()
            if keyboard.is_pressed("q"):
                break

    def ARMROV_withkey(self,key):
        if keyboard.is_pressed(key):
            self.arm_rov()

    def DISARMROV_withkey(self,key):
        if keyboard.is_pressed(key):
            self.disarm_rov()

    def INIT_STARTUP_withkey(self,key,mode):
        if keyboard.is_pressed(key) and not self.startup_ONS:
            self.set_mode(mode)
            self.arm_rov()
            self.startup_ONS = True
            self.system_armed = True
            time.sleep(.1)

    def INIT_SHUTDOWN_withkey(self,key):
        if keyboard.is_pressed(key) and self.startup_ONS:
            time.sleep(.5)
            self.set_mode('POSHOLD')
            self.disarm_rov()
            self.startup_ONS = False
            self.system_armed = False
            time.sleep(.1)

    def MODE_CHANGE_ONS_withkey(self,key):
        if keyboard.is_pressed(key) and self.MODE_ONS:
            self.Manual_overide = True
            self.MODE_ONS = False

    def MODE_CHANGE_RELEASE_ONS_withkey(self,key):
        if keyboard.is_pressed(key) and not self.MODE_ONS:
            self.Manual_overide = False
            self.MODE_ONS = True

    def MODE_ONS_RESET_withkey(self,key):
        if keyboard.is_pressed(key) and not self.MODE_ONS:
            self.MODE_ONS = True

    def INIT_OFFSET_withkey(self,key):
        if keyboard.is_pressed(key):
            print("mason")
            return True
        else: 
            return False

# Need to make it act as a stream.. or take into account each case... use switch statements
    def keyboard_controlls(self,pid_thrust_x=0,pid_thrust_y=0,pid_thrust_z=0,pid_thrust_yaw=0,letsride=True):
        """
        KEYBOARD TO THRUST MAPPING
            Mapping:
                (w s) --> longitudinal movement (x)
                (d a) --> yaw movement (omega z)

                (l .) --> vertical movement (z)
                (, /) --> lateral movement (y)

            INPUT:
                pid_thrust_{} TYPE: float
                    - output from the pid controlled. 
                    - used to autonomously control the bluerov without manually having to use controls
                letsride  TYPE: boolean
                    - defines if the bluerov will be completing a trajectory

        """
        pid_thrust_x = int(pid_thrust_x)
        pid_thrust_y = int(pid_thrust_y)
        pid_thrust_z = int(pid_thrust_z)
        pid_thrust_yaw = int(pid_thrust_yaw)

        thruster_power = 140
        thruster_nominal = 1500
        thruster_positive_deadband = 0
        thruster_negative_deadband = -0

        # Safety check on pid values not exceeding known maximum wanted thrust
        
        if pid_thrust_x > 150:
            pid_thrust_x = 150
        if pid_thrust_y > 150:
            pid_thrust_y = 150
        # if pid_thrust_z > 220:
        #     pid_thrust_z = 220
        if pid_thrust_yaw > 150:
            pid_thrust_yaw = 150

        if pid_thrust_x < -150:
            pid_thrust_x = -150
        if pid_thrust_y < -150:
            pid_thrust_y = -150
        # if pid_thrust_z < -220:
        #     pid_thrust_z = -220
        if pid_thrust_yaw < -150:
            pid_thrust_yaw = -150



        if keyboard.is_pressed("space") or letsride:

            if keyboard.is_pressed("w"):
                self.set_rc_channel_pwm(self.rc_channel['vx'],thruster_nominal+thruster_power)
            elif keyboard.is_pressed("s"):
                self.set_rc_channel_pwm(self.rc_channel['vx'],thruster_nominal+thruster_power*-1)
            else:
                if pid_thrust_x == abs(pid_thrust_x): # positive thrust, positive deadband
                    self.set_rc_channel_pwm(self.rc_channel['vx'],thruster_nominal + thruster_positive_deadband + pid_thrust_x)
                else:
                    self.set_rc_channel_pwm(self.rc_channel['vx'],thruster_nominal + thruster_negative_deadband + pid_thrust_x)                    
                
            if keyboard.is_pressed("d"):
                self.set_rc_channel_pwm(self.rc_channel['yaw'],thruster_nominal+thruster_power)
            elif keyboard.is_pressed("a"):
                self.set_rc_channel_pwm(self.rc_channel['yaw'],thruster_nominal+thruster_power*-1)
            else:
                if pid_thrust_yaw == abs(pid_thrust_yaw): # positive thrust, positive deadband
                    self.set_rc_channel_pwm(self.rc_channel['yaw'],thruster_nominal + 20 + pid_thrust_yaw)
                else: 
                    self.set_rc_channel_pwm(self.rc_channel['yaw'],thruster_nominal -20  + pid_thrust_yaw)                

            if keyboard.is_pressed("l"):
                self.set_rc_channel_pwm(self.rc_channel['vz'],thruster_nominal+thruster_power)
            elif keyboard.is_pressed("."):
                self.set_rc_channel_pwm(self.rc_channel['vz'],thruster_nominal+thruster_power*-1)
            else:
                if pid_thrust_z == abs(pid_thrust_z): # positive thrust, positive deadband
                    self.set_rc_channel_pwm(self.rc_channel['vz'],thruster_nominal + thruster_positive_deadband + pid_thrust_z)
                else:
                    self.set_rc_channel_pwm(self.rc_channel['vz'],thruster_nominal + thruster_negative_deadband + pid_thrust_z)                
                

            if keyboard.is_pressed("/"):
                self.set_rc_channel_pwm(self.rc_channel['vy'],thruster_nominal+thruster_power)
            elif keyboard.is_pressed(","):
                self.set_rc_channel_pwm(self.rc_channel['vy'],thruster_nominal+thruster_power*-1)
            else:
                if pid_thrust_y == abs(pid_thrust_y): # positive thrust, positive deadband
                    self.set_rc_channel_pwm(self.rc_channel['vy'],thruster_nominal + thruster_positive_deadband + pid_thrust_y)
                else:
                    self.set_rc_channel_pwm(self.rc_channel['vy'],thruster_nominal + thruster_negative_deadband + pid_thrust_y)

                    

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

    def set_target_depth(self,x,y,depth):
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
            type_mask=0b110111111100, #( # ignore everything except z position ---> 3580 (decimal)
                #mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                #mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                #mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                #mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                #mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                #mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                #mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                #mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                ##mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |#
                #mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                #mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE), 
            lat_int=0, lon_int=0, alt=depth, # (x, y WGS84 frame pos - not used), z [m]
            vx=0, vy=0, vz=0, # velocities in NED frame [m/s] (not used)
            afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
            # accelerations in NED frame [N], yaw, yaw_rate
            #  (all not supported yet, ignored in GCS Mavlink)
        )
        """

        self.master.mav.set_position_target_local_ned_send(
            int(1e3 * (time.time() - self.boot_time)),
            self.master.target_system, self.master.target_component,
            coordinate_frame=mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, # OFFSET is taken from current position
            type_mask=0b110111111100, #( # ignore everything except z position ---> 3580 (decimal)
                #mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                #mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
               # mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
               # mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
               # mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
               # mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
               # mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
               # mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
               # mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                #mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |#
               # mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
               # mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE), 
            x=x,y=y,z=depth,vx=0,vy=0,vz=0,afx=0,afy=0,afz=0,yaw=0,yaw_rate=0
        )


    """
    Below are functions needed in order to execute the mission protocal outlined in:
    https://mavlink.io/en/services/mission.html#mission_types
    """

    # first a the number of waypoints included in the mission must be defined and sent to the autopilot
    # NOTE: 3 different ways of calling the function are listed. Only execute one at a time when testing
    def mission_count(self,seq):
        """
        NOT currently being used instead using function already define in mavutil
        """
        # 1.)
        # mavlink10 call
        self.master.mav.mission_count_send(self.master.target_system, self.master.target_component,seq)

        # 2.)
        # extension off of message definition (sets the mission type... may be needed so try even if teh original works)
        #self.master.mav.mission_count_send(self.master.target_system, self.master.target_component,1,mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

        # 3.)
        # not mavlink10
        #self.master.mav.waypoint_count_send(self.master.target_system, self.master.target_component, seq)

        # 4.) 
        # using mavutil library
        #self.master.mav.waypoint_count_send((self.master.target_system, self.master.target_component,seq)

        message_request_data = self.view_message_loop('MISSION_REQUEST_INT',stopafterone=True,returndata=True)
        # self.acknowledge_command()

    """
    Format for saving missions in txt files
    QGC WPL <VERSION>
    <INDEX> <CURRENT WP> <COORD FRAME> <COMMAND> <PARAM1> <PARAM2> <PARAM3> <PARAM4> <PARAM5/X/LATITUDE> <PARAM6/Y/LONGITUDE> <PARAM7/Z/ALTITUDE> <AUTOCONTINUE>
    """

    def set_home_position(self,lat,long,altitude,startatcurrent=False):
        #print('--- ', self.master.target_system, ',', self.master.target_component)
        if not startatcurrent:
            self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            1, # set position
            0, # param1 # if this value is set to 1 then it should use the current position!
            0, # param2
            0, # param3
            0, # param4
            lat, # lat
            long, # lon
            altitude) 
        else:
            self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            1, # set position
            1, # param1 # if this value is set to 1 then it should use the current position!
            0, # param2
            0, # param3
            0, # param4
            0, # lat
            0, # lon
            0) 

        #acknoledge the command was sent
        self.acknowledge_command()

        # check that position was set correctly --> first need to check that this function is working properly 
        #self.get_home_position()

    # command is valid but no HOME_POSITION is showing up in portal
    def get_home_position(self):
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, 
        0, # May want to try setting this value to one... do not go deep into why this would work
        0,
        0,
        0,
        0,
        0,
        0,
        0)

        # acknowledge the command was sent
        #self.acknowledge_command()
            # commad: 410 result: 4
            # MEMINFO
        self.view_message_loop(atthisrate=1)
        # view message --> should be sent as HOME_POSITION message
        #self.view_message_loop('HOME_POSITION',stopafterone=True)

    def change_altitue_condition(self,altitude):
        self.master.mav.command_long_send(self.master.target_system,self.master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT, 
            0, # May want to try setting this value to one... do not go deep into why this would work
            1, # Rate at which of decent
            0,
            0,
            0,
            0,
            0,
            altitude) # altitude in meters (documentation does not specify the frame but assume global ralative)
        
        # acknowledge the command was sent
        self.acknowledge_command()

    def change_yaw_condition(self,yaw):
        self.master.mav.command_long_send(self.master.target_system,self.master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
            0, # May want to try setting this value to one... do not go deep into why this would work
            1, # Rate at which of decent
            0,
            0,
            0,
            0,
            0,
            yaw) # altitude in meters (documentation does not specify the frame but assume global ralative)

        # acknowledge the command was sent
        self.acknowledge_command()

    def change_altitude_DO(self,altitude):
        self.master.mav.command_long_send(self.master.target_system,self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE, 
            1, # May want to try setting this value to one... do not go deep into why this would work
            1, # Altitude
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # Frame to use
            0,
            0,
            0,
            0,
            altitude) # altitude in meters (documentation does not specify the frame but assume global ralative)
        
        # acknowledge the command was sent
        self.acknowledge_command()

    def goto_this_position(self,yaw,lat,long,alt):
        self.master.mav.command_long_send(self.master.target_system,self.master.target_component,
            master.mavutil.MAV_CMD_OVERRIDE_GOTO,
            0,
            mavutil.mavlink.MAV_GOTO_DO_HOLD, # pause mission
            mavutil.mavlink.MAV_GOTO_HOLD_AT_SPECIFIED_POSITION, # hold at specified position
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # frame
            yaw, # desired yaw
            lat, # desired latitude
            long, # desired longitude
            alt) # desired altitude

        # acknowledge the command was sent
        self.acknowledge_command()

    # NOTE: due to the params of this command I will try different methods of calling it
    def takeoff_hack(self, yaw, lat, long, alt):
        # 1.)
        # command_long method
        self.master.mav.command_long_send(self.master.target_system,self.master.target_system,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0,
            0, # pitch
            0,
            0,
            yaw, # yaw
            lat, # latitude position
            long, # longitude position
            alt) # altitude position

        # 2.)
        # command_int method
        """
        self.master.mav.mav.command_int_send(self.master.target_system,self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # command
            0,
            0, 
            0, # pitch
            0,
            0,
            yaw, # yaw
            lat, # latitude position
            long, # longitude position
            alt) # altitude position
        """

        # 3.)
        # misison method --> need other components of the mission protocal


    def define_mission_vanillia(self,seq,yaw,lat,long,alt,frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,current=0,autocontinue=1):
        p = mavutil.mavlink.MAVLink_mission_item_message(master.target_system, master.target_component, seq, frame,
                                                        command,
                                                        current, autocontinue,
                                                        0, # param 1 --> Hold time [s]
                                                        0, # param 2 --> accepted radius [m]
                                                        0, # param 3 --> pass radius [m]
                                                        yaw, # param 4 [deg]
                                                        lat, # param 5
                                                        long, # param 6
                                                        alt) # param 7
        self.wp.add(p)


    def define_mission_nav_waypoint(self,seq,yaw,lat,long,alt):
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        current= 0 # current waypoint that the mission is on
        autocontinue = 1 # continue to the next waypoint once the current waypoint has been reached
        p = mavutil.mavlink.MAVLink_mission_item_message(master.target_system, master.target_component, seq, frame,
                                                                command,
                                                                current, autocontinue,
                                                                0, # param 1 --> Hold time [s]
                                                                0, # param 2 --> accepted radius [m]
                                                                0, # param 3 --> pass radius [m]
                                                                yaw, # param 4 [deg]
                                                                lat, # param 5
                                                                long, # param 6
                                                                alt) # param 7
        
        # NOTE: there is disparity on whether how the parameters should be defined in reference to the frame.  
                # --> if _INT frame in global then the parameters should be scaled by 1e7 
                # --> if _INT frame in local then parameters should be scaled by 1e4
                # --> if non _INT then the parameters should not be scaled.

                # This is due to encoding the date for int values that get used in _INT frames and not having to encode data due to float types being used in non _INT frames

                # ADDITIONLLY: some packages are just expected to decode messages becasue they will expect them to be encoded int values.  Therefor it is a safe bet to use _INT frame encoded
                # HOWEVER: because this is a unique type of function call that I have not seen before I will not encode the numbers at first and see what happene
                # ** next test should be changing the frame, then change to frame back to _INT and scale the vales by 1e7

        self.wp.add(p)

    def mission_execute(self,yaw,lat,long,alt):
        # Set the home position to the initial posiition of the trajectory... all other waypoints will be defined as if this position is the origin
        # if you want to set the current position as the home position then set_home_posiiton definition needs to be altered
        self.set_home_position(lat[0],long[0],alt[0])

        # add all waypoints to the initiated wp class
        for i in range(len(lat)):
            self.define_mission_nav_waypoint(i,yaw[i],lat[i],long[i],alt[i])

        # clear all waypoints that could possibly be there now
        master.waypoint_clear_all_send()
        # send the MISSION_COUNT(n) command
        master.waypoint_count_send(self.wp.count())      

        # upload mission
        for i in range(self.wp.count()):
            msg = master.recv_match(type=['MISSION_REQUEST'],blocking=True)
            print(msg)
            master.mav.send(self.wp.wp(msg.seq))
            print('Sending waypoint {0}'.format(msg.seq))

        # confirmation of mission upload completion --> make sure to check message and see it it gets 0 for message completion type
        print("just waiting on mission complete acknowledgement")
        self.view_message_loop('MISSIN_ACK')

    def set_current_waypoiny(self,seq):
        self.master.waypoint_set_current_send(seq)

        # see if message was executed properly:
        # ---> change message type --> On failure, the Drone must broadcast a STATUSTEXT with a MAV_SEVERITY and a string stating the problem. This may be displayed in the UI of receiving systems.
        self.view_message_loop('MISSION_CURRENT',stopafterone=True)



if __name__ == "__main__":
    """
    Documentation of using code:

        Understanding how pymavlink is created:
            - Pymavlink is createed from code generator -> mavgen
    """
    """
    Testing:
    ** note that a possible way for calling command_int_send was found by using master.mav.mav.command_int_send()

    # Can be done in lab:
       * 1.) Test getting message intervals function: get_message_interval()
       **     - There is currently an issure with the frequency that the values are coming in... Can not get the values to go back to 0hz

       * 2.) Test setting message intervals function: request_message()
       **     - There is currently an issure with the frequency that the values are coming in... Can not get the values to go back to 0hz

       * 3.) Test if added functionality for seeing messages only once is working. function: view _message_loop(stopafterone=True)

       * 4.) Test if added functionality for returning message values is working. function: view _message_loop(returnvalue=True) NOTE: stopafterone must also be True

       DO NOT WANT TO RISK ACTUATNG MOTORS 5.) Test getting the home position with cmd: MAV_CMD_GET_HOME_POSITION . function: get_home_position()
        --> Appently can not set home position: Possible reasons: 1. dvl was disconnected 2. was not armed 3. mode was not set 4. 2&3

       DO NOT WANT TO RISK ACTUATNG MOTORS 6.) Test setting the home position with function: set_home_position()
            - Also try doing so by using current position

        ** The reason that the home position is so important is because it defines the cordinate system for GLOBAL_RELATIVE 
        NOTE: There is mention that mission protocal for ArduPilot and PX4 cannot compete mission in anyhting other than global, therefore this frame has to work... Actual Global will not be fesible

        7.) Need to determine where the local pose is initiated from

    # Done at pool
        1.) Test setting the bluerov to guided mode using new functionality: set_mode() ---> see what mode id is equal to 4 can be printed from function
            - Also want to see if there is any change in outcome of guided mode from functions: enable_guided_mode() , set_guided_master() , set_guided_limits()
            - will need to be tested if it fixes the weird action being taken be the bluerov... this will require bluerov to be armed
            - with these new funtions called see if it effects the orginal method for setting waypoints... set_target_depth()

        2.) Test using hacking waypoint movements:
            - condition altitude and yaw commands : function: change_yaw() and change_altitude()
            - DO commands. functions: change_altitude_DO() and goto_this_position()

        3.) Test mission upload  function: mission_execute()
            - NOTE: if everyhting works but the bluerov is still not moving then the autocontinue is not wokring and this may be a problem worth looking into
            --> Be sure to play with the frames, encoded values, and the type being used

        4.) Test mission execution --> by set_current_waypoint()
            - change seq to 1 and see what happens... if then to 2 and see what happens

       * 5.) Test updated manual control
    """

    rovmav = ROVMAV()

    rovmav.set_mode('MANUAL')

    # rovmav.arm_rov()
    # rovmav.disarm_rov()
    # rovmav.request_parameters_list()

    rovmav.request_parameter_value('GND_EXT_BUS')
    rovmav.set_parameter_value(1,'GND_EXT_BUS')
    rovmav.request_parameter_value('GND_EXT_BUS')

    # rovmav.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,1000)
    # rovmav.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,1000)
    # rovmav.view_message_loop('SCALED_PRESSURE2')
    # update_req = 0
    # while True:
        
    #     # linear_pos = rovmav.view_message('LOCAL_POSITION_NED',returndata=True,display_output=False)
    #     # angular_pose = rovmav.view_message('ATTITUDE',returndata=True,display_output=False)
    #     pressure = rovmav.view_message('SCALED_PRESSURE',returndata=True,display_output=False)
    #     #if linear_pos != None:
    #         #print(linear_pos["x"])
    #     if pressure != None:
    #        print(pressure["press_abs"]*100/1000/9.81 - 14.7)
    #     rovmav.keyboard_controlls()
    #     if keyboard.is_pressed('q'):
    #         break
    #     update_req += 1
    #
    # time.sleep(1)
    # rovmav.disarm_rov()
    #rovmav.set_mode('POSHOLD')
"""
    while True:
        rovmav.view_message('ATTITUDE')
        rovmav.keyboard_controlls()
        if keyboard.is_pressed('q'):
            break


    time.sleep(1)
    rovmav.disarm_rov()
"""
    #rovmav.manual_control_loop()


    #rovmav.set_mode('GUIDED')
    #rovmav.view_message_loop('HEARTBEAT')
    #rovmav.set_home_position(0,0,76,startatcurrent=False)
    #rovmav.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,msg='GLOBAL_POSITION_INT')
    #rovmav.get_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SYSTEM_TIME)
    #rovmav.request_message(mavutil.mavlink.MAVLINK_MSG_ID_SYSTEM_TIME,atthisfrequency=10)
    #rovmav.mission_count()

    #rovmav.request_message()
    #rovmav.view_message_loop('LOCAL_POSITION_NED',atthisrate=0)
    #rovmav.set_parameter_value(-10,'SURFACE_DEPTH')
    #rovmav.request_parameter_value('SURFACE_DEPTH')


    #rovmav.set_mode()
    #rovmav.view_message_loop('HEARTBEAT')
    #rovmav.set_mode('GUIDED')
    #rovmav.set_mode('STABILIZE')
    #rovmav.set_mode('POSHOLD')
    #rovmav.disarm_rov()
    #rovmav.arm_rov()
    #rovmav.manual_control_loop()
    #rovmav.disarm_rov()
