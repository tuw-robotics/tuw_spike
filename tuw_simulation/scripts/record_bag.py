#!/usr/bin/python3
import rclpy
from rclpy.node import Node

import rosbag2_py
from rclpy.serialization import serialize_message
from std_msgs.msg import String
from example_interfaces.msg import Int32
import rosbag2_py._storage
from nav_msgs.msg import Odometry

import weakref


class RecordPosition(Node):
    def __init__(self):
        super().__init__('record_position')
        
        self.subscription = self.create_subscription(Odometry, 'robot0/odom', self.topic_callback, 10)
        
        self.file = open("data.csv", "w")
        weakref.finalize(self, self.file.close)
        print("x;y;z", file=self.file)
    
    def topic_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        print(pos.x, pos.y, pos.z, sep=";", file=self.file)
               

def main(args=None):
    rclpy.init(args=args)
    
    node = RecordPosition()

    rclpy.spin(node) 
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()