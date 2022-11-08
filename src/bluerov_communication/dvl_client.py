#!/usr/bin/env python3

import socket
import json
import rospy
from nav_msgs.msg import Odometry

from time import sleep

class ROS():
    def __init__(self, client):
        rospy.init_node('DVL_DATA')
        self.odom = rospy.Publisher("/DVL_ODOM", Odometry, queue_size=100)
        self.doppler = rospy.Publisher("/DVL_DOPPLER", Odometry, queue_size=100)
        
        self.dead_reckon = Odometry()
        self.doppler_data = Odometry()

        self.rate = rospy.Rate(10) # 10hz
        
        self.dvl = client
        
    def run(self):
        while not rospy.is_shutdown():
            data = json.loads(self.dvl.get().decode("utf-8"))
            
            self.dead_reckon.header.stamp = rospy.Time.now()
            self.doppler_data.header.stamp = rospy.Time.now()
            if (data['type'] == 'velocity'):
                self.doppler_data.twist.twist.linear.x = data['vx']
                self.doppler_data.twist.twist.linear.y = data['vy']
                self.doppler_data.twist.twist.linear.z = data['vz']
                
                self.doppler.publish(self.doppler_data)
                
            elif (data['type'] == 'position_local'):
                self.dead_reckon.pose.pose.position.x = data['x']
                self.dead_reckon.pose.pose.position.y = data['y']
                self.dead_reckon.pose.pose.position.z = data['z']
                self.dead_reckon.pose.pose.orientation.x = data['x']
                self.dead_reckon.pose.pose.orientation.y = data['y']
                self.dead_reckon.pose.pose.orientation.z = data['z']
                
                self.odom.publish(self.dead_reckon)

            self.rate.sleep()
        
    

class DVLclient():
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        
    def connect(self):
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect((TCP_IP, TCP_PORT))
            self.s.settimeout(1)
        except socket.error as err:
            sleep(1)
            self.connect()

    def get(self):
        raw_data = b''

        while True:
            rec = self.s.recv(1)
            # print(rec)
            if rec == b'\n':
                break
            raw_data += rec
        return (raw_data)

    def run(self):
        while True:
            self.get()
        

if __name__ == '__main__':
    
    TCP_IP =  "192.168.2.95" 
    TCP_PORT =  16171
    
    client = DVLclient(TCP_IP, TCP_PORT)
    client.connect()
    
    ros = ROS(client)
    ros.run()