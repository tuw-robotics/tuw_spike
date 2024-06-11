from typing import TextIO

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from rclpy.time import Time
from rclpy.node import Node

class PoseLogger(Node):
    def __init__(self, file: TextIO, topic: str, msg_type):
        super().__init__(f"pose_logger_{topic}")
        self.file = file
        print("timestamp tx ty tz qx qy qz qw", file=self.file)
        self.sub = self.create_subscription(msg_type, topic, self.pose_callback, 10)

    def pose_callback(self, msg):
        self.write_pose(msg, self.file)

    def write_pose(self, msg: Odometry | PoseWithCovarianceStamped, file: TextIO):
        stamp = Time.from_msg(msg.header.stamp)
        t = msg.pose.pose.position
        q = msg.pose.pose.orientation
        print(f"{stamp.nanoseconds * 1e-9:.6f} "
                f"{t.x:.3f} {t.y:.3f} {t.z:.3f} "
                f"{q.x:.3f} {q.y:.3f} {q.z:.3f} {q.w:.3f}",
                file=file)