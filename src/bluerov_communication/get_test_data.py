#!/usr/bin/env python3


import rospy
from nav_msgs.msg import Odometry
from pymavlink_interface import ROVMAV, mavutil

rospy.init_node("TEST_DATA")   
publish_scaled_imu = rospy.Publisher("SCALED_IMU2",Odometry,queue_size=10)
publish_raw_pressure = rospy.Publisher("RAW_PRESSURE",Odometry,queue_size=10)
publish_attitude = rospy.Publisher("ATTITUDE",Odometry,queue_size=10)

rate = rospy.Rate(10)   # UNITS: hz
rovmav = ROVMAV() 

scaled_imu = Odometry()
raw_pressure = Odometry()
attitude = Odometry()

rovmav.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,11,display_interval=False)

if __name__ == '__main__':
    
    while not rospy.is_shutdown():
        
        mav_scaled_imu = rovmav.view_message('SCALED_IMU2',returndata=True,display_output=False)    
        if mav_scaled_imu != None:
            # print(mav_scaled_imu)
            scaled_imu.pose.pose.position.x = mav_scaled_imu["xacc"]
            scaled_imu.pose.pose.position.y = mav_scaled_imu["yacc"]
            scaled_imu.pose.pose.position.z = mav_scaled_imu["zacc"]
            scaled_imu.twist.twist.linear.x = mav_scaled_imu["xgyro"]
            scaled_imu.twist.twist.linear.y = mav_scaled_imu["ygyro"]
            scaled_imu.twist.twist.linear.z = mav_scaled_imu["zgyro"]
            publish_scaled_imu.publish(scaled_imu)
        
        
        mav_raw_pressure = rovmav.view_message('SCALED_PRESSURE',returndata=True,display_output=False)
        if mav_raw_pressure != None:
            # print(mav_raw_pressure)
            raw_pressure.pose.pose.position.z = mav_raw_pressure["press_abs"]
            publish_raw_pressure.publish(raw_pressure)
        
        
        mav_attitude = rovmav.view_message('ATTITUDE',returndata=True,display_output=False) 
        if mav_attitude != None:
            print(mav_attitude)
            attitude.pose.pose.position.x = mav_attitude['roll']
            attitude.pose.pose.position.y = mav_attitude['pitch']
            attitude.pose.pose.position.z = mav_attitude['yaw']
            attitude.twist.twist.linear.x = mav_attitude['rollspeed']
            attitude.twist.twist.linear.y = mav_attitude['pitchspeed']
            attitude.twist.twist.linear.z = mav_attitude['yawspeed']
            publish_attitude.publish(attitude)
        
        mav_pos_ned = rovmav.view_message('LOCAL_POSITION_NED',returndata=True,display_output=False)
        if mav_pos_ned != None:
            print(mav_pos_ned)
        
        rate.sleep()
    
    

