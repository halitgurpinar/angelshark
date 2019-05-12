#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped
import math

car_length = 144    #the distance between front and rear wheel in cm
car_width = 110     #the distance between two front wheel in cm
half_of_car_width = car_width/2
flag_move = 0

def set_throttle_steer(data):

    global flag_move

    pub_vel_left_rear_wheel = rospy.Publisher('/angelshark/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    #pub_vel_right_rear_wheel = rospy.Publisher('/pars/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    #pub_vel_left_front_wheel = rospy.Publisher('/pars/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
    #pub_vel_right_front_wheel = rospy.Publisher('/pars/right_front_wheel_velocity_controller/command', Float64, queue_size=1)

    pub_pos_left_steering_hinge = rospy.Publisher('/angelshark/left_steering_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher('/angelshark/right_steering_hinge_position_controller/command', Float64, queue_size=1)

    throttle = data.drive.speed/0.1
    steer = data.drive.steering_angle
    if steer != 0:
        R = car_length / math.tan(steer)
        right_steer = math.atan(car_length / (R - half_of_car_width))
        left_steer  = math.atan(car_length / (R + half_of_car_width))
        #rospy.loginfo('\x1b[1M\r'
                      #'\033[34;1mSteer: \033[32;1m%0.2f, '
                      #'\033[34;1mSteering right: \033[32;1m%0.2f rad\033[0m, '
                      #'\033[34;1mSteering left: \033[32;1m%0.2f rad\033[0m',
                      #steer, right_steer, left_steer)
    else:
        right_steer = left_steer = 0

    pub_vel_left_rear_wheel.publish(throttle)
    #pub_vel_right_rear_wheel.publish(throttle)
    #pub_vel_left_front_wheel.publish(throttle)
    #pub_vel_right_front_wheel.publish(throttle)
    pub_pos_left_steering_hinge.publish(left_steer)
    pub_pos_right_steering_hinge.publish(right_steer)

def servo_commands():

    rospy.init_node('servo_commands', anonymous=True)

    rospy.Subscriber("angelshark/ackermann_cmd", AckermannDriveStamped, set_throttle_steer)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass