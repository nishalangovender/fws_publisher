#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math


class KinematicsCalculator(Node):

    def __init__(self):
        super().__init__('kinematics_calculator')

        # Initialize subscribers for Twist messages from teleop_twist_keyboard
        self.twist_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10)

        # Initialize publishers for joint commands
        self.steering_pub = self.create_publisher(
            Float64MultiArray,
            '/steering_position_controller/commands',
            10)
        self.wheel_pub = self.create_publisher(
            Float64MultiArray,
            '/wheel_velocity_controller/commands',
            10)

    def twist_callback(self, msg):

        # Parameters
        R = 0.065
        L = 0.2

        # Twist Messages
        x_dot = msg.linear.x
        beta = msg.linear.y
        psi_dot = msg.angular.z

        # Initialize delta to zero
        delta = 0

        # Initialize v to a default value
        v = 0

        if x_dot != 0:
            if beta == 0:
                # Zero Side-Slip

                # Steering Angle
                delta = math.atan(0.5 * psi_dot * L / x_dot)

                # Wheel Velocity
                v = x_dot / math.cos(delta)

            else:
                # Parallel Steering

                # Steering Angle
                delta = beta

                # Wheel Velocity
                if psi_dot == 0:  # Given x_dot only
                    v = x_dot / (2 * math.cos(beta))
                else:  # Given both x_dot and psi_dot -> use psi_dot
                    v = psi_dot * L / math.sin(beta)

        v_A = v
        v_B = v

        # Calculate wheel angular velocities (ω_A and ω_B)
        omega_A = v_A / R
        omega_B = v_B / R

        # Publish the calculated values to the command topics
        steering_msg = Float64MultiArray(
            data=[delta, delta, -delta, -delta])
        wheel_msg = Float64MultiArray(
            data=[omega_A, omega_A, omega_B, omega_B])

        self.steering_pub.publish(steering_msg)
        self.wheel_pub.publish(wheel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsCalculator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
