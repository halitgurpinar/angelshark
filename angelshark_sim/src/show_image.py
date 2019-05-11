#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class image_shower(object):
    def __init__(self):
        
        self.cvbridge = CvBridge()
        self.camera_subscriber = rospy.Subscriber('/angelshark/camera/image_raw', Image, self.camera_callback, queue_size=1)
        
        self.cam_data = None

        self.rate = rospy.Rate(10)
        self.rate.sleep()


    def camera_callback(self, msg):
        try:
            self.cam_data = self.cvbridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print("camera err:",e)

    def show_img(self):
        while not rospy.is_shutdown():
            cv2.imshow("Angelshark Camera",self.cam_data)
            
            k = cv2.waitKey(1)
            if k == 27:
                break
            
            self.rate.sleep()

def main():
    rospy.init_node("angelshark_img_show",disable_signals=True)

    camera = image_shower()
    try:
        camera.show_img()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
