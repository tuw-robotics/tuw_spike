import math

from typing import TextIO

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

TRAJECTORY_UPDATE_PERIOD = 0.01

class TestTrajectoryDriverNode(Node):
    def __init__(self, ground_truth: TextIO, estimated: TextIO) -> None:
        super().__init__("trajectory_driver")

        self.f_ground_truth = ground_truth
        self.f_estimated = estimated

        for f in self.f_ground_truth, self.f_estimated:
            print("#timestamp tx ty tz qx qy qz qw", file=f)

        self.velocity = self.declare_parameter("velocity", 0.3).get_parameter_value().double_value
        self.center_dist = self.declare_parameter("center_dist", 0.3).get_parameter_value().double_value
        self.cross_alpha = self.declare_parameter("cross_alpha", math.radians(5.0)).get_parameter_value().double_value
        self.startup_delay = self.declare_parameter("startup_delay", 1.0).get_parameter_value().double_value

        self.radius = self.center_dist / 2.0 * math.cos(self.cross_alpha)
        self.straight = self.center_dist * math.sin(self.cross_alpha)
        self.angular_velocity = self.velocity / self.radius

        self.time_straight = self.straight / self.velocity
        self.time_curve = 2.0 * (math.pi - self.cross_alpha) / self.angular_velocity
        self.total_time = 2.0 * self.time_straight + 2.0 * self.time_curve

        self.time = -self.startup_delay

        self.pub_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.sub_ground_truth = self.create_subscription(Odometry, "odom_ground_truth", self.ground_truth_callback, 10)
        self.sub_estimated = self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self.estimated_callback, 10)

        self.trajectory_timer = self.create_timer(TRAJECTORY_UPDATE_PERIOD, self.update_trajectory)
        self.update_trajectory()

    def update_trajectory(self):
        twist = Twist()

        if self.time >= 0.0:
            twist.linear.x = self.velocity

        if 0.5*self.time_straight + 0.0*self.time_curve <= self.time \
            and self.time < 0.5*self.time_straight + 1.0*self.time_curve:
            twist.angular.z = self.angular_velocity
        elif 1.5*self.time_straight + 1.0*self.time_curve <= self.time \
            and self.time < 1.5*self.time_straight + 2.0*self.time_curve:
            twist.angular.z = -self.angular_velocity

        self.pub_vel.publish(twist)

        self.time += TRAJECTORY_UPDATE_PERIOD
        if self.time > self.total_time:
            self.time -= self.total_time

    def ground_truth_callback(self, msg: Odometry):
        self.write_pose(msg, self.f_ground_truth)
        
    def estimated_callback(self, msg: PoseWithCovarianceStamped):
        self.write_pose(msg, self.f_estimated)

    def write_pose(self, msg: Odometry | PoseWithCovarianceStamped, file: TextIO):
        stamp = Time.from_msg(msg.header.stamp)
        t = msg.pose.pose.position
        q = msg.pose.pose.orientation
        print(f"{stamp.nanoseconds * 1e-9:.6f} "
              f"{t.x:.3f} {t.y:.3f} {t.z:.3f} "
              f"{q.x:.3f} {q.y:.3f} {q.z:.3f} {q.w:.3f}",
              file=file)

def main(args=None):
    with open("ground_truth.csv", "w") as ground_truth, open("estimated.csv", "w") as estimated:
        rclpy.init(args=args)
        rclpy.spin(TestTrajectoryDriverNode(ground_truth, estimated))
        rclpy.shutdown()

if __name__ == '__main__':
    main()
