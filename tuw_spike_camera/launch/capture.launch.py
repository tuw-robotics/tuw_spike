from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    capture_comp = ComposableNode(
        package='tuw_libcamera',
        plugin='tuw_libcamera::CaptureNode',
        extra_arguments=[{'use_intra_process_comms': True}],
        namespace="camera",
        parameters=[{
            "camera_info_name": "camera",
            "camera_info_url": "package://tuw_spike_camera/calibration/${NAME}.yaml",
            "frame_id": "camera_optical",
            "stream_roles": ["video"],
            "streams.video": {
                "format": "RGB888",
                "target_format": "bgr8",
                "width": 1280,
                "height": 720
            }
        }]
    )

    transport_comp = ComposableNode(
        package='tuw_libcamera',
        plugin='tuw_libcamera::TransportNode',
        extra_arguments=[{'use_intra_process_comms': True}],
        namespace="camera"
    )

    undistort_comp = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
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
        composable_node_descriptions=[
            capture_comp,
            undistort_comp,
            transport_comp
        ]
    )

    rosbag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '/camera/image', '/camera/camera_info'],
        name='rosbag',
        output='both',
        condition=IfCondition(LaunchConfiguration('record'))
    )

    return LaunchDescription([
        DeclareLaunchArgument("record", default_value="False"),
        container,
        rosbag
    ])
