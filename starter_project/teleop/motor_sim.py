#!/usr/bin/env python3
"""
Node for simulating receiving joystick Twist input and publishing velocity commands to the respective motors
"""

from typing import Any
import rospy
from std_msts
from mrover.msg import WheelCmd
from geometry_msgs.msg import Twist

wheel_pub = rospy.Publisher("/wheel_cmd", WheelCmd, queue_size=1)

def calculate_wheel_velocities(msg):
    forward_back = msg.linear.x
    left_right = msg.angular.z

    #Scaling multiplier to adjust values if needed
    K = 1
    
    left = K * (forward_back + left_right)
    right = K * (forward_back - left_right)

    #Ensure values are [-1,1] for each motor
    if abs(left) > 1:
        left = left/abs(left)
    if abs(right) > 1:
        right = right/abs(right)

    wheel_cmd = WheelCmd()
    wheel_cmd.left = left
    wheel_cmd.right = right
    wheel_pub.publish(wheel_cmd)

def main():
    rospy.init_node("motor_sim_node")

    rospy.Subscriber("/joystick_cmd_vel", Twist, calculate_wheel_velocities)

    rospy.spin()


if __name__ == "__main__":
    main()