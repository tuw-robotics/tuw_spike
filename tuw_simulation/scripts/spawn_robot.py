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

def get_model_list(string):
    befor, sep, after = string.partition(":")
    temp = after.split("\n")
    temp = ' '.join(temp).split()
    temp = [x for x in temp if x != '-']
    return temp


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("minimal_client")
    node.declare_parameter('X', 0.)
    node.declare_parameter('Y', 0.)
    node.declare_parameter('model_name', "robot0")
    x = node.get_parameter('X').get_parameter_value().double_value
    y = node.get_parameter('Y').get_parameter_value().double_value
    z = 0.4
    error_code = 0
    
    with open("/tmp/robot.urdf", "w") as f:
        f.write(sys.argv[1])
    
    model_name = node.get_parameter('model_name').get_parameter_value().string_value
    
    result = subprocess.run([
        "ign", "model", "--list"
    ], stdout = subprocess.PIPE, text=True)
    
    temp = get_model_list(result.stdout)
    if temp.__contains__(model_name):
        node.get_logger().info("model already exists")
        error_code = 1
    elif (len(temp) == 0):
        node.get_logger().info("world not existing")
        error_code = 2
    else:
        node.get_logger().info("create model with name: " + str(model_name))
        
    
    if error_code == 0:
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
    print(error_code)

if __name__ == '__main__':
    main()