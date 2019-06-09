#!/usr/bin/python
# -*- coding: utf-8 -*-

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

import rospy
import rospkg
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
        rospy.logwarn(self.model.summary())

        self.pub = rospy.Publisher('/angelshark/ackermann_cmd', AckermannDriveStamped, queue_size=1)
        self.laser = rospy.Subscriber('/angelshark/scan', LaserScan, self.laser_callback, queue_size=1)
        self.laser_data = None
        
        self.maximum_laser_range = 12.0
        
        self.rate = rospy.Rate(10)
        self.rate.sleep()

    def laser_callback(self, msg):
        self.laser_data = msg.ranges

    def predict(self, laser_arr):
        # speed = 0.5
        out = self.model.predict(laser_arr, batch_size=1)
        steering = out[0][0]
        speed = out[0][1]
        
        #print("Steering := %f" % steering)
        #print("Speed := %f" % speed)
        rospy.logwarn("Steering := %f , Speed := %f", steering,speed)
        return {'steering': steering, 'speed': speed}

    def cmd_publisher(self):
        while not rospy.is_shutdown():
            if self.laser_data is None:
                rospy.logerr("No laser!")
                continue
            
            laser_arr = np.asarray(self.laser_data)
            
            for i in range(len(laser_arr)):
                if np.isinf(laser_arr[i]):
                    laser_arr[i] = self.maximum_laser_range
            laser_arr = laser_arr.reshape(1, laser_arr.shape[0]).astype(np.float64) / self.maximum_laser_range
            
            prediction = self.predict(laser_arr)
            
            msg = AckermannDriveStamped()
            msg.drive.speed = prediction['speed']
            msg.drive.steering_angle = prediction['steering']
            
            self.pub.publish(msg)
            
            self.rate.sleep()

def main():
    rospy.init_node('predictor_laser', anonymous=True)
    
    sleep(30)
    
    predict = PredictAngle()
    predict.cmd_publisher()

if __name__ == '__main__':
    main()  
