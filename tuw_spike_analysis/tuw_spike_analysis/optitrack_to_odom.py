from typing import TextIO

import rclpy
from rclpy.node import Node

from mocap4r2_msgs.msg import RigidBody, RigidBodies

from nav_msgs.msg import Odometry

class OptitrackToOdom(Node):
    def __init__(self):
        super().__init__(f"optitrack_to_odom")

        self.sub = self.create_subscription(RigidBodies, "rigid_bodies", self.optitrack_callback, 10)
        self.pub = self.create_publisher(Odometry, "odom_ground_truth", 10)

    def optitrack_callback(self, msg: RigidBodies):
        if msg.rigidbodies:
            body: RigidBody = msg.rigidbodies[0]
            odom = Odometry()
            odom.header.frame_id = "map"
            odom.header.stamp = msg.header.stamp
            odom.pose.pose = body.pose
            odom.pose.pose.position.z = 0
            self.pub.publish(odom)
        
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(OptitrackToOdom())
    rclpy.shutdown()

if __name__ == '__main__':
    main()