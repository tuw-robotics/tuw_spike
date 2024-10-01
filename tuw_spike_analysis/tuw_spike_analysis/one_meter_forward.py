#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math
import time


class DriveForward(Node):
    def __init__(self):
        super().__init__('drive_forward_node')
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        self.initial_x = None
        self.initial_y = None
        self.has_reached_goal = False

    def odom_callback(self, msg):
        if self.initial_x is None or self.initial_y is None:
            self.initial_x = msg.pose.pose.position.x
            self.initial_y = msg.pose.pose.position.y
            self.get_logger().info(f'Initial pose: x={self.initial_x}, y={self.initial_y}')
        else:
            current_x = msg.pose.pose.position.x
            current_y = msg.pose.pose.position.y

            # Calculate Euclidean distance from initial position
            distance = math.sqrt((current_x - self.initial_x)**2 + (current_y - self.initial_y)**2)
            self.get_logger().info(f'Current distance: {distance:.2f} meters')

            if distance >= 1.0 and not self.has_reached_goal:
                self.get_logger().info('Reached 1 meter, stopping robot')
                self.get_logger().info(f'End pose: x={current_x}, y={current_y}; diffX={current_x - self.initial_x} diffY={current_y - self.initial_y}')
                self.stop_robot()
                self.has_reached_goal = True

    def drive_forward(self, speed):
        self.get_logger().info('Waiting for 10 seconds...')
        time.sleep(5)

        twist_stamped = TwistStamped()
        twist_stamped.header.frame_id = 'base_link'

        twist_stamped.twist.linear.x = 0.0

        for x in range(10):
            twist_stamped.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(twist_stamped)
            rclpy.spin_once(self)
        
        twist_stamped.twist.linear.x = speed

        self.get_logger().info('Driving forward...')
        while not self.has_reached_goal:
            twist_stamped.header.stamp = self.get_clock().now().to_msg()
            
            self.publisher_.publish(twist_stamped)
            
            rclpy.spin_once(self)

    def stop_robot(self):
        twist_stamped = TwistStamped()
        twist_stamped.twist.linear.x = 0.0
        twist_stamped.header.frame_id = 'base_link'
        twist_stamped.header.stamp = self.get_clock().now().to_msg()

        # Publish the stop command
        self.publisher_.publish(twist_stamped)
        self.get_logger().info('Robot stopped.')

def main(args=None):
    rclpy.init(args=args)
    node = DriveForward()

    node.drive_forward(0.28)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
