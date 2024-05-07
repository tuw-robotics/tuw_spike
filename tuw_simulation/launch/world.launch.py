from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    spike_sim = FindPackageShare("tuw_simulation")
    ros_gz_sim = FindPackageShare("ros_gz_sim") 
    
    # Start simulation
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([ros_gz_sim, "launch", "gz_sim.launch.py"])),
        launch_arguments={
            # Launch simulation automatically started (-r)
            "gz_args": [PathJoinSubstitution([spike_sim, "world", "empty.sdf"]), TextSubstitution(text=" ")],
            "on_exit_shutdown": "True"
        }.items()
    )
    
    return LaunchDescription([
        gz_sim]
    )