from typing import TextIO

import rclpy
from rclpy.node import Node

from mocap4r2_msgs.msg import RigidBody, RigidBodies

from nav_msgs.msg import Odometry

import rclpy.qos
import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class TFToOdom(Node):
    def __init__(self):
        super().__init__(f"tf_to_odom")

        self.sub = self.create_subscription(TFMessage, "tf", self.tf_callback, rclpy.qos.QoSProfile(
            depth=100,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        ))
        self.pub = self.create_publisher(Odometry, "odom", 10)

    def tf_callback(self, msg: TFMessage):
        for t in msg.transforms:
            t: TransformStamped
            if t.child_frame_id == "base_odom" and t.header.frame_id == "odom":
                odom = Odometry()
                odom.header.frame_id = "map"
                odom.header.stamp = t.header.stamp
                odom.pose.pose.position.x = t.transform.translation.x
                odom.pose.pose.position.y = t.transform.translation.y
                odom.pose.pose.position.z = t.transform.translation.z
                odom.pose.pose.orientation = t.transform.rotation
                self.pub.publish(odom)
        
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TFToOdom())
    rclpy.shutdown()

if __name__ == '__main__':
    main()