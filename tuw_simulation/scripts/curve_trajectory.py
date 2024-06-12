#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from typing import TextIO
from rclpy.time import Time
import math


class Curve_Trajectory(Node):
    def __init__(self, ground_truth: TextIO, odom: TextIO) -> None:
        super().__init__('Curve_Trajectory')
        
        self.f_ground_truth = ground_truth
        self.f_odom = odom
        
        for f in self.f_ground_truth, self.f_odom:
            print("timestamp tx ty tz qx qy qz qw vx", file=f)
        
        self.pub_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.sub_ground_truth = self.create_subscription(Odometry, "odom_ground_truth", self.ground_truth_callback, 10)

        self.velocity = self.declare_parameter("velocity", 0.5).get_parameter_value().double_value
        self.delay = self.declare_parameter("delay", 1.0).get_parameter_value().double_value
        self.radius = self.declare_parameter("radius", 0.5).get_parameter_value().double_value
        
        self.toggle_time = -self.delay
        self.time = -self.delay
        
        self.delay_timer = self.create_timer(math.pi / 100, self.delay_timer_callback)
        self.delay_timer_callback()
        
          
    def delay_timer_callback(self):
        twist = Twist()
        
        if self.time >= 0.0:
            if self.toggle_time >= math.pi:
                self.radius *= -1
                self.toggle_time = 0.0
            
            twist.linear.x = self.velocity
            twist.angular.z = self.velocity / self.radius
            
        
        self.get_logger().info(f"velocity: {twist.linear.x}, angular: {twist.angular.z}, time: {self.time:.2f}, toggle_time: {self.toggle_time:.2f}")
        self.pub_vel.publish(twist)
        self.time += math.pi / 100
        self.toggle_time += math.pi / 100
        
    def ground_truth_callback(self, msg: Odometry):
        self.write_pos(msg, self.f_ground_truth)
    
    def odom_callback(self, msg: Odometry):
        self.write_pos(msg, self.f_odom)
    
    def write_pos(self, msg: Odometry, file: TextIO):
        stamp = Time.from_msg(msg.header.stamp)
        t = msg.pose.pose.position
        q = msg.pose.pose.orientation
        print(f"{stamp.nanoseconds * 1e-9:.6f} "
              f"{t.x:.3f} {t.y:.3f} {t.z:.3f} "
              f"{q.x:.3f} {q.y:.3f} {q.z:.3f} {q.w:.3f} "
              f"{msg.twist.twist.linear.x:.3f}",
              file=file)

def main(args=None):
    
    with open("ground_truth.csv", "w") as ground_truth, open("odom.csv", "w") as odom:
        rclpy.init(args=args)
        rclpy.spin(Curve_Trajectory(ground_truth, odom))
        rclpy.shutdown()

if __name__ == '__main__':
    main()