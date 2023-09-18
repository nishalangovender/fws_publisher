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

        # Initialize variables to track whether Twist messages have been received
        self.twist_received = False

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

        # Set the flag to indicate that Twist messages have been received
        self.twist_received = True

    def publish_idle_transforms(self):
        # Publish default or idle values for steering angle and wheel angular velocities
        steering_msg = Float64MultiArray(
            data=[0.0, 0.0, 0.0, 0.0])  # Set to zero or your desired default value
        wheel_msg = Float64MultiArray(
            data=[0.0, 0.0, 0.0, 0.0])  # Set to zero or your desired default value

        self.steering_pub.publish(steering_msg)
        self.wheel_pub.publish(wheel_msg)

    def timer_callback(self):
        # Check if Twist messages have not been received and publish idle transforms if necessary
        if not self.twist_received:
            self.publish_idle_transforms()

        # Reset the flag to indicate that Twist messages have not been received
        self.twist_received = False


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsCalculator()

    # Create a timer to periodically check and publish idle transforms
    timer_period = 5.0  # Adjust this period as needed
    timer = node.create_timer(timer_period, node.timer_callback)

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
