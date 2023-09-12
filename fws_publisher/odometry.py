#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math
import tf2_ros
import geometry_msgs.msg  # Add this import for TransformStamped


class OdometryCalculator(Node):

    def __init__(self):
        super().__init__('odometry_calculator')

        # Initialize subscribers for Twist messages
        self.twist_sub = self.create_subscription(
            Twist,
            '/cmd_vel',  # Replace with the actual topic name for velocity commands
            self.twist_callback,
            10)

        # Initialize publisher for Odometry
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',  # Replace with the actual topic name for odometry data
            10)

        # Robot parameters
        # Replace with your robot's wheelbase (distance between wheels)
        self.wheelbase = 0.235

        # Initial pose and time
        self.x = 0.0
        self.y = 0.0
        self.z_offset = -0.65  # Z-offset for the odom frame below base_link
        self.theta = 0.0
        self.last_time = self.get_clock().now().to_msg()

        # Initialize TF2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def twist_callback(self, msg):
        current_time = self.get_clock().now().to_msg()
        dt = (current_time.sec - self.last_time.sec) + \
            1e-9 * (current_time.nanosec - self.last_time.nanosec)

        # Extract linear and angular velocity from Twist message
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Calculate linear and angular displacement
        linear_displacement = linear_velocity * dt
        angular_displacement = angular_velocity * dt

        # Update the robot's pose
        self.x += linear_displacement * math.cos(self.theta)
        self.y += linear_displacement * math.sin(self.theta)
        self.theta += angular_displacement

        # Create and publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = self.z_offset  # Set the z-offset
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2)

        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity

        self.odom_pub.publish(odom_msg)

        # Publish the transformation between odom and base_link frames
        transform_stamped = geometry_msgs.msg.TransformStamped()
        transform_stamped.header.stamp = current_time
        transform_stamped.header.frame_id = 'odom'
        transform_stamped.child_frame_id = 'base_link'
        transform_stamped.transform.translation.x = self.x
        transform_stamped.transform.translation.y = self.y
        transform_stamped.transform.translation.z = self.z_offset  # Set the z-offset
        transform_stamped.transform.rotation.z = math.sin(self.theta / 2)
        transform_stamped.transform.rotation.w = math.cos(self.theta / 2)

        self.tf_broadcaster.sendTransform(transform_stamped)

        # Update the last time
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = OdometryCalculator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
