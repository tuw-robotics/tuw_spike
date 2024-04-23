#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from pydoc import ModuleScanner
from re import S
import sys
import rclpy
import math
import tf_transformations
from rclpy.task import Future
import subprocess
import re




def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("minimal_client")
    node.declare_parameter('X', 0.)
    node.declare_parameter('Y', 0.)
    node.declare_parameter('model_name', "robot0")
    x = node.get_parameter('X').get_parameter_value().double_value
    y = node.get_parameter('Y').get_parameter_value().double_value
    z = 0.4
    
    with open("/tmp/robot.urdf", "w") as f:
        f.write(sys.argv[1])
    
    model_name = node.get_parameter('model_name').get_parameter_value().string_value
    node.get_logger().info("model_name: " + str(model_name))
    
    subprocess.run([
        "ign", "service",
        "-s", "/world/plain_world/create",
        "--reqtype", "ignition.msgs.EntityFactory",
        "--reptype", "ignition.msgs.Boolean",
        "--timeout", "1000",
        "--req", f'sdf_filename: "/tmp/robot.urdf", name: "{model_name}",  pose: {{ position: {{x: {x}, y: {y}, z: {z}}} }}'
        ], stdout=subprocess.DEVNULL, check=True)
    
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()