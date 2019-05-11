#!/usr/bin/env python

"""
This script makes Gazebo less fail by translating gazebo status messages to odometry data.
Since Gazebo also publishes data faster than normal odom data, this script caps the update to 20hz.
Winter Guerra
"""

import sys
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import tf2_ros

RATE = 20


class OdometryNode(object):
    def __init__(self, allow_odom_reset=False):
        # init internals
        self.last_received_pose = Pose()
        self.last_received_twist = Twist()
        self.last_recieved_stamp = None
        self.allow_odom_reset = allow_odom_reset

        # Set the update rate
        self.rate = rospy.Rate(RATE)
        self.pub_odom = rospy.Publisher('/angelshark/odom', Odometry, queue_size=1)

        self.tf_pub = tf2_ros.TransformBroadcaster()

        # Set subscribers
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.sub_robot_pose_update)

    def sub_robot_pose_update(self, msg):
        # Find the index of the racecar
        try:
            array_index = msg.name.index('angelshark::pars__base_link')
        except ValueError:
            # Wait for Gazebo to startup
            pass
        else:
            # Extract our current position information
            self.last_received_pose = msg.pose[array_index]
            self.last_received_twist = msg.twist[array_index]

        self.last_recieved_stamp = rospy.Time.now()

    def run(self):
        while not rospy.is_shutdown():
            self.send_transform()
            self.step()

    def send_transform(self):
        if self.last_recieved_stamp is None:
            return

        cmd = Odometry()
        cmd.header.stamp = self.last_recieved_stamp
        cmd.header.frame_id = 'map'
        cmd.child_frame_id = 'odom'
        cmd.pose.pose = self.last_received_pose
        cmd.twist.twist = self.last_received_twist
        self.pub_odom.publish(cmd)

        transform = TransformStamped(
            header=Header(
                frame_id=cmd.header.frame_id,
                stamp=cmd.header.stamp
            ),
            child_frame_id=cmd.child_frame_id,
            transform=Transform(
                translation=cmd.pose.pose.position,
                rotation=cmd.pose.pose.orientation
            )
        )

        self.tf_pub.sendTransform(transform)

    def step(self):
        try:
            self.rate.sleep()
        except rospy.ROSTimeMovedBackwardsException as exception:
            if not self.allow_odom_reset:
                raise exception
        except rospy.ROSInterruptException:
            rospy.signal_shutdown("Odometry node is interrupted!")


def main():
    rospy.init_node("gazebo_odometry_node")

    allow_odom_reset = len(sys.argv) >= 2 and sys.argv[1] == '--allow-odom-reset'

    odom_node = OdometryNode(allow_odom_reset)
    odom_node.run()


if __name__ == '__main__':
    main()