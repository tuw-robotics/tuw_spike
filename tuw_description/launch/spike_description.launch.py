from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, LogInfo
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import Command, LaunchConfiguration, FindExecutable

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)


def generate_launch_description():

    use_sim_time     = LaunchConfiguration('use_sim_time',  default='true')
    model_name_arg   = DeclareLaunchArgument('model_name',  default_value=TextSubstitution(text='robot0'))
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("tuw_description"),
                    "model",
                    "spike",
                    "main.xacro",
                ]
            ),
            " namespace:=",
            LaunchConfiguration('model_name')
        ]
    )
    
    params = {'robot_description': robot_description_content}
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='both',
                                  parameters=[params],
                                  namespace=[LaunchConfiguration('model_name')],)  
    
    return LaunchDescription(
        [model_name_arg, robot_state_publisher])