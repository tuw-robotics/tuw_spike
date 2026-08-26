import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription


def generate_launch_description():
    camera_params_file = os.path.join(
        get_package_share_directory('tuw_libcamera'),
        'config',
        'camera_module3_wide_bgr.yaml'
    )

    capture_comp = ComposableNode(
        package='tuw_libcamera',
        plugin='tuw_libcamera::CaptureNode',
        name='libcamera_node',
        extra_arguments=[{'use_intra_process_comms': True}],
        namespace="camera",
        parameters=[camera_params_file]
    )

    transport_comp = ComposableNode(
        package='tuw_libcamera',
        plugin='tuw_libcamera::TransportNode',
        extra_arguments=[{'use_intra_process_comms': True}],
        namespace="camera"
    )

    container = ComposableNodeContainer(
        name='camera_processing_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        #ros_arguments=["--log-level", "debug"],
        #prefix='gdbserver :2222',
        composable_node_descriptions=[capture_comp, transport_comp]
    )

    return LaunchDescription([
        container
    ])
