import os

from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, IncludeLaunchDescription, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_context import LaunchContext


def generate_launch_description():
    libcamera_comp = ComposableNode(
        package='tuw_libcamera',
        plugin='tuw_libcamera::LibcameraNode',
        name='camera_capture',
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    container = ComposableNodeContainer(
        name='camera_processing_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[libcamera_comp]
    )

    return LaunchDescription([
        container
    ])
