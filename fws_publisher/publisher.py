#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math


class Kinematics(Node):

    def __init__(self):
        super().__init__('kinematics')

        # Initialize Joint Command Publishers
        self.steering_pub = self.create_publisher(
            Float64MultiArray,
            '/steering_position_controller/commands',
            10)
        self.wheel_pub = self.create_publisher(
            Float64MultiArray,
            '/wheel_velocity_controller/commands',
            10)

        # Initialize Twist Subscriber
        self.twist_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10)

    def twist_callback(self, msg):

        # Parameters
        R = 0.065
        L = 0.2

        # Init Output
        delta = 0.0
        v_wheel = 0.0

        # Twist Messages
        x_dot = msg.linear.x
        omega_z = msg.angular.z

        # Twist to Model
        V = x_dot
        Omega = omega_z

        # Vehicle Model
        if V != 0:
            delta = math.atan((Omega * L) / (2 * V))
            v_wheel = V / math.cos(delta)

        # m/s to rad/s
        omega_wheel = v_wheel / R

        # Publish Commands
        steering_msg = Float64MultiArray(
            data=[delta, delta, -delta, -delta])
        wheel_msg = Float64MultiArray(
            data=[omega_wheel, omega_wheel, omega_wheel, omega_wheel])
        
        self.steering_pub.publish(steering_msg)
        self.wheel_pub.publish(wheel_msg)


def main(args=None):
    rclpy.init(args=args)

    node_kinematics = Kinematics()

    rclpy.spin(node_kinematics)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
