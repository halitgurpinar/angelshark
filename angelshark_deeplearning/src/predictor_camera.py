#!/usr/bin/python
# -*- coding: utf-8 -*-

from sensor_msgs.msg import  Image
from ackermann_msgs.msg import AckermannDriveStamped

import rospy
import rospkg
import cv_bridge
import cv2
import numpy as np

from time import sleep
import json
from keras.models import model_from_json

class PredictAngle():
    def __init__(self):
        self.rate = rospy.Rate(20)

        self.rospack = rospkg.RosPack()
        self.package_path = self.rospack.get_path('angelshark_deeplearning')
        self.model_name = self.package_path + '/model/model'

        self.jstr = json.loads(open(self.model_name+'.json').read())
        self.model = model_from_json(self.jstr)
        self.model.load_weights(self.model_name+'.h5')

        self.pub = rospy.Publisher('/angelshark/ackermann_cmd', AckermannDriveStamped, queue_size=1)

        self.camera = rospy.Subscriber('/angelshark/camera/image_raw', Image, self.camera_callback, queue_size=1) 
        self.camera_data = None
        self.camera_bridge = cv_bridge.CvBridge()
        
        self.rate = rospy.Rate(10)
        self.rate.sleep()

    
    def camera_callback(self, img_data):
        try:
            self.camera_data = self.camera_bridge.imgmsg_to_cv2(img_data, 'bgr8')
        except cv_bridge.CvBridgeError:
            rospy.logerr("Cv Bridge Err")
            return

    def predict(self, image):
        # speed = 0.5
        
        out = self.model.predict(image, batch_size=1)
        steering = out[0][0]
        speed = out[0][1]

        #print("Steering := %f" % steering)
        #print("Speed := %f" % speed)
        rospy.logwarn("Steering := %f , Speed := %f", steering,speed)

        return {'steering': steering, 'speed': speed}

    def cmd_publisher(self):
        while not rospy.is_shutdown():
            if self.camera_data is None:
                rospy.logerr("No Image!")
                continue
            
            h = 240
            w = 320
            
            image = self.camera_data
            image = cv2.resize(image, (w, h))
            image = image.reshape(1, h, w, 3)
            image = np.asarray(image).astype(np.float32) / 255.0
            
            prediction = self.predict(image)
            
            msg = AckermannDriveStamped()
            msg.drive.speed = prediction['speed']
            msg.drive.steering_angle = prediction['steering']
                    
            self.pub.publish(msg)
            
            self.rate.sleep()

def main():
    rospy.init_node('predictor_camera', anonymous=True)
    
    sleep(30)
    
    predict = PredictAngle()
    predict.cmd_publisher()

if __name__ == '__main__':
    main()  
