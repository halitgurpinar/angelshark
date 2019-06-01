#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
import sys

class AckermannDriveJoyop:

    def __init__(self):
        self.max_speed = 5.0            # m/s
        self.max_steering_angle = 0.524  # radian
        cmd_topic = 'angelshark/ackermann_cmd'

        self.speed = 0
        self.steering_angle = 0
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.drive_pub = rospy.Publisher(cmd_topic, AckermannDriveStamped,
                                         queue_size=1)
        rospy.Timer(rospy.Duration(1.0/5.0), self.pub_callback, oneshot=False)
        rospy.loginfo('ackermann_drive_joyop_node initialized')

    def joy_callback(self, joy_msg): # left triger = joy_msg.axes[2]   ,   right triger = joy_msg.axes[5]   ,   right joypad = joy_msg.axes[3]
        self.speed = joy_msg.axes[4] * self.max_speed;
        self.steering_angle = -joy_msg.axes[0] * self.max_steering_angle;


    def pub_callback(self, event):
        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.drive.speed = self.speed
        ackermann_cmd_msg.drive.steering_angle = self.steering_angle
        self.drive_pub.publish(ackermann_cmd_msg)
        #self.print_state()

    def print_state(self):
        sys.stderr.write('\x1b[2J\x1b[H')
        rospy.loginfo('\x1b[1M\r'
                      '\033[34;1mSpeed: \033[32;1m%0.2f m/s, '
                      '\033[34;1mSteering Angle: \033[32;1m%0.2f rad\033[0m',
                      self.speed, self.steering_angle)

    def finalize(self):
        rospy.loginfo('Halting motors, aligning wheels and exiting...')
        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.drive.speed = 0
        ackermann_cmd_msg.drive.steering_angle = 0
        self.drive_pub.publish(ackermann_cmd_msg)
        sys.exit()

if __name__ == '__main__':
    rospy.init_node('ackermann_drive_joyop_node')
    joyop = AckermannDriveJoyop()
    rospy.spin()
