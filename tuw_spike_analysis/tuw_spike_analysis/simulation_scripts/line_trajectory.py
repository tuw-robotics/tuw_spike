#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from typing import TextIO
from rclpy.time import Time


class Line_Trajectory(Node):
    def __init__(self, ground_truth: TextIO, odom: TextIO) -> None:
        super().__init__('Line_Trajectory')
        
        self.f_ground_truth = ground_truth
        self.f_odom = odom
        
        for f in self.f_ground_truth, self.f_odom:
            print("timestamp tx ty tz qx qy qz qw vx", file=f)
        
        self.pub_vel = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.sub_ground_truth = self.create_subscription(Odometry, "odom_ground_truth", self.ground_truth_callback, 10)

        self.velocity = self.declare_parameter("velocity", 0.5).get_parameter_value().double_value
        self.delay = self.declare_parameter("delay", 1.0).get_parameter_value().double_value
        
        self.time = -self.delay
        
        self.delay_timer = self.create_timer(0.01, self.delay_timer_callback)
        self.delay_timer_callback()
        
          
    def delay_timer_callback(self):
        twist = TwistStamped()
        
        if self.time >= 0.0 and self.time < 1.0:
            twist.twist.linear.x = self.velocity/4
            self.get_logger().info(f"below 1s")
        elif self.time >= 1.0 and self.time < 2.0:
            self.get_logger().info(f"below 2s")
            twist.twist.linear.x = self.velocity
        elif self.time >= 2.0:
            self.get_logger().info(f"greater 2s")
            twist.twist.linear.x = 0.0
        
        self.get_logger().info(f"velocity: {twist.twist.linear.x}, time: {self.time}")
        self.pub_vel.publish(twist)
        self.time += 0.01
        
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
        rclpy.spin(Line_Trajectory(ground_truth, odom))
        rclpy.shutdown()

if __name__ == '__main__':
    main()