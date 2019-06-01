#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import trollius
from trollius import From

import pygazebo
import pygazebo.msg.int_pb2

import random

import time

first = [-1,1]
second = [-2,2]
third = [-3,3]
fourth = [-4,4]

message1 = pygazebo.msg.int_pb2.Int()
message2 = pygazebo.msg.int_pb2.Int()
message3 = pygazebo.msg.int_pb2.Int()
message4 = pygazebo.msg.int_pb2.Int()


@trollius.coroutine
def publish_loop():
    while not rospy.is_shutdown():
        manager = yield From(pygazebo.connect())
        
        publisher = yield From(
            manager.advertise('/gazebo/default/angelshark_joy',
                            'gazebo.msgs.Int'))
        
        message1.data = random.choice(first)
        message2.data = random.choice(second)
        message3.data = random.choice(third)
        message4.data = random.choice(fourth)
        
        for i in range(20):
            yield From(publisher.publish(message1))
            yield From(publisher.publish(message2))
            yield From(publisher.publish(message3))
            yield From(publisher.publish(message4))
            yield From(trollius.sleep(1.0))


def main():
    rospy.init_node("traffic_lights_control",disable_signals=True)
    
    try:
        loop = trollius.get_event_loop()
        loop.run_until_complete(publish_loop())
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
