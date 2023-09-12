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
            '/cmd_vel',  # Update to match the teleop_twist_keyboard topic
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

        # Define r as calss attribute
        self.r = 0.065

        # Define the given value for L as a class attribute
        self.L = 0.2

        # Initialize delta_A and delta_B as class attributes
        self.delta_A = 0.0
        self.delta_B = 0.0

    def twist_callback(self, msg):
        # Extract linear and angular velocity from Twist message
        V_x = msg.linear.x
        V_y = msg.linear.y
        V = math.sqrt(V_x ** 2 + V_y ** 2)
        psi_dot = msg.angular.z

        # Access the L, delta_A, and delta_B attributes from the class
        L = self.L
        delta_A = self.delta_A
        delta_B = self.delta_B

        # Handle the case when linear velocity (V) is zero
        if V == 0:
            cos_beta = 1.0  # Set a default value for cos_beta
            cos_delta_A = 1.0  # Set default value for cos_delta_A
            cos_delta_B = 1.0  # Set default value for cos_delta_B
            tan_delta_A = 0.0
            tan_delta_B = 0.0
        else:
            # Calculate cos(β), cos(δ_A), cos(δ_B), tan(δ_A), and tan(δ_B)
            denominator = 1 + (2 * L * psi_dot / V) ** 2
            cos_beta = 1 / math.sqrt(denominator)
            cos_delta_A = math.cos(math.atan(2 * L * psi_dot - V * (math.tan(
                delta_B) - math.tan(delta_A)) / V * (math.tan(delta_B) + math.tan(delta_A))))
            cos_delta_B = math.cos(math.atan(2 * L * psi_dot + V * (math.tan(
                delta_B) - math.tan(delta_A)) / V * (math.tan(delta_B) + math.tan(delta_A))))
            tan_delta_A = math.tan(delta_A)
            tan_delta_B = math.tan(delta_B)

        # Calculate steering angles (delta_A and delta_B)
        self.delta_A = math.atan2(
            2 * L * psi_dot - V * (tan_delta_B - tan_delta_A), V * (tan_delta_B + tan_delta_A))
        self.delta_B = math.atan2(
            2 * L * psi_dot + V * (tan_delta_B - tan_delta_A), V * (tan_delta_B + tan_delta_A))

        # Calculate wheel velocities (v_A and v_B)
        v_A = V * cos_beta / cos_delta_A
        v_B = V * cos_beta / cos_delta_B

        # Calculate wheel angular velocities (ω_A and ω_B)
        omega_A = v_A / self.r
        omega_B = v_B / self.r

        # Publish the calculated values to the command topics
        steering_msg = Float64MultiArray(
            data=[self.delta_A, self.delta_A, self.delta_B, self.delta_B])
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
