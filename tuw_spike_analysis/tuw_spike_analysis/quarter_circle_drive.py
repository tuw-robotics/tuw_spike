#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math
import time

class QuarterCircleDrive(Node):
    def __init__(self):
        super().__init__('quarter_circle_drive_node')
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        self.initial_yaw = None
        self.current_yaw = None
        self.radius = 0.5
        self.linear_velocity = 0.2
        self.angular_velocity = self.linear_velocity / self.radius
        self.has_reached_goal = False

    def euler_from_quaternion(self, x, y, z, w):
        # Convert quaternion into euler angles (roll, pitch, yaw)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return yaw

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        self.current_yaw = self.euler_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

        if self.initial_yaw is None:
            self.initial_yaw = self.current_yaw
            self.get_logger().info(f'Initial yaw: {self.initial_yaw}')

        if self.initial_yaw is not None and self.current_yaw is not None:
            angle_turned = self.current_yaw - self.initial_yaw

            angle_turned = (angle_turned + math.pi) % (2 * math.pi) - math.pi

            self.get_logger().info(f'Angle turned: {math.degrees(angle_turned):.2f} deg')

            if abs(angle_turned) >= math.pi / 2:
                self.get_logger().info('Reached 90 deg')
                self.stop_robot()
                self.has_reached_goal = True

    def drive_quarter_circle(self):
        twist_stamped = TwistStamped()
        twist_stamped.twist.linear.x = self.linear_velocity
        twist_stamped.twist.angular.z = self.angular_velocity
        twist_stamped.header.frame_id = 'base_link'
        
        self.get_logger().info('Driving quarter circle...')

        while not self.has_reached_goal:
            twist_stamped.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(twist_stamped)
            rclpy.spin_once(self)

    def stop_robot(self):
        twist_stamped = TwistStamped()
        twist_stamped.twist.linear.x = 0.0
        twist_stamped.twist.angular.z = 0.0
        twist_stamped.header.frame_id = 'base_link'
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        
        self.publisher_.publish(twist_stamped)
        self.get_logger().info('Robot stopped.')

def main(args=None):
    rclpy.init(args=args)
    node = QuarterCircleDrive()

    node.drive_quarter_circle()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
