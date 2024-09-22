#!/usr/bin/env python3
"""
Node for simulating receiving joystick Twist input and publishing velocity commands to the respective motors
"""

from typing import Any
import rclpy
from mrover.msg import WheelCmd
from geometry_msgs.msg import Twist

wheel_pub = None

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
    rclpy.init()
    node = rclpy.create_node("motor_sim_node")

    wheel_pub = node.create_publisher(WheelCmd, "/wheel_cmd", qos_profile=1)

    node.create_subscription(Twist, "/joystick_cmd_vel", calculate_wheel_velocities, qos_profile=1)

    rclpy.spin(node)


if __name__ == "__main__":
    main()