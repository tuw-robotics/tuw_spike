from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription


def generate_launch_description():
    capture_comp = ComposableNode(
        package='tuw_libcamera',
        plugin='tuw_libcamera::CaptureNode',
        extra_arguments=[{'use_intra_process_comms': True}],
        namespace="camera",
        parameters=[{
            "camera_info_name": "camera",
            "stream_roles": ["video"],
            "streams.video": {
                "format": "YUYV",
                "target_format": "yuv422_yuy2",
                "width": 1280,
                "height": 720
            }
        }]
    )

    container = ComposableNodeContainer(
        name='camera_processing_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        #ros_arguments=["--log-level", "debug"],
        #prefix='gdbserver :2222',
        composable_node_descriptions=[capture_comp]
    )

    return LaunchDescription([
        container
    ])
