#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Image, Imu, LaserScan
import cv_bridge
import time
import cv2
import numpy as np
import pandas as pd

from time import sleep
import os

class DataExtractor(object):
    def __init__(self):
        self.camera_bridge = cv_bridge.CvBridge()
        
        self.laser = rospy.Subscriber('/angelshark/scan', LaserScan, self.laser_callback, queue_size=1)
        self.cmd_vel = rospy.Subscriber("/angelshark/ackermann_cmd", AckermannDriveStamped, self.cmd_vel_callback)
        self.camera = rospy.Subscriber('/angelshark/camera/image_raw', Image, self.camera_callback, queue_size=1) 
        #self.imu = rospy.Subscriber('/angelshark/imu', Imu, self.imu_callback, queue_size=1)
        
        self.camera_data = None
        self.laser_data = None
        self.steering_angle = None
        self.speed = None
        #self.imu_x = None
        #self.imu_y = None
        #self.imu_z = None

        self.rate = rospy.Rate(10)
        
        self.list_laser=[]
        self.list_steering_angle=[]
        self.list_speed=[]
        #self.list_imu_x=[]
        #self.list_imu_y=[]
        #self.list_imu_z=[]
        self.data_len = 0
        
        self.home_dir = os.getenv("HOME")
        self.last_file_num = 0

        self.rate.sleep()
            
    def cmd_vel_callback(self, msg):
        self.steering_angle = msg.drive.steering_angle
        self.speed = msg.drive.speed

    def laser_callback(self, msg):
        self.laser_data = msg.ranges
        
    #def imu_callback(self, msg):
    #    self.imu_x = msg.magnetic_field.x
    #    self.imu_y = msg.magnetic_field.y
    #    self.imu_z = msg.magnetic_field.z

    def camera_callback(self, msg):
        try:
            self.camera_data = self.camera_bridge.imgmsg_to_cv2(msg, 'bgr8')
        except cv_bridge.CvBridgeError:
            return

    def camera_image_saver(self, img, data_len):
        if not os.path.exists(self.home_dir + "/Desktop/DataPacket/Images"): # We create all nested folders here
            os.makedirs(self.home_dir + "/Desktop/DataPacket/Images")
        
        target_file = os.path.expanduser(self.home_dir + '/Desktop/DataPacket/Images/1.jpg') # If there is old images in Images
        exists = os.path.isfile(target_file)
        if exists and self.last_file_num == 0: # We get last image num in this if block
            images_list = os.listdir(self.home_dir + '/Desktop/DataPacket/Images')
            images_list = [int(i.split('/')[-1].split('.')[0]) for i in images_list]
            self.last_file_num = sorted(images_list)[-1]

        # We add last image num to data_len so If there is old image in folder we append our new images not replace them.
        image_num = int(data_len) + self.last_file_num
        cv2.imwrite(self.home_dir + '/Desktop/DataPacket/Images/%s.jpg' % image_num, img)

    def extract(self):
        while not rospy.is_shutdown():
            
            #self.list_imu_x.append(self.imu_x)
            #self.list_imu_y.append(self.imu_y)
            #self.list_imu_z.append(self.imu_z)
            self.list_laser.append(self.laser_data)
            self.camera_image_saver(self.camera_data, self.data_len+1) # First image name will be 1.jpg not 0.jpg
            self.list_steering_angle.append(self.steering_angle)
            self.list_speed.append(self.speed)

            self.data_len += 1
            print "Data Length:",self.data_len
            self.rate.sleep()

    def create_csv(self):
        column_names = ["laser"+str(i) for i in range(len(self.laser_data))]

        df_laser = pd.DataFrame(self.list_laser, columns=column_names)

        df = pd.DataFrame(data={"steering_angle":self.list_steering_angle,
                                "speed":self.list_speed})

        df = pd.concat([df_laser, df], axis=1)

        target_file = os.path.expanduser(self.home_dir + '/Desktop/DataPacket/data.csv')
        exists = os.path.isfile(target_file)
        if exists:
            df.to_csv(target_file, sep=',',index=False, header=False, mode = 'a')
            rospy.logwarn("appended to %s file", target_file)
        else:
            df.to_csv(target_file, sep=',',index=False, header=True, mode = 'a')
            rospy.logwarn("created %s file", target_file)
        

def main():
    rospy.init_node("data_extractor", disable_signals=True)

    data_extractor = DataExtractor()
    sleep(1) # Time in seconds.
    while True:
        try:
            data_extractor.extract()
        except KeyboardInterrupt:
            break
    data_extractor.create_csv()
            
if __name__ == '__main__':
    main()