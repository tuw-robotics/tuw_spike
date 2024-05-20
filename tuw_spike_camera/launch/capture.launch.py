from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    tuw_spike_camera = FindPackageShare("tuw_spike_camera")

    container = [
        LaunchConfiguration("ros_namespace"), "/camera_processing_container"
    ]
    container_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([tuw_spike_camera, "launch", "container.launch.py"]))
    )

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

    rectify_comp = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        extra_arguments=[{'use_intra_process_comms': True}],
        namespace="camera"
    )

    record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '/camera/image', '/camera/camera_info'],
        name='rosbag',
        output='both',
        condition=IfCondition(LaunchConfiguration('record'))
    )

    replay = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', '--clock', '--loop',
            'bags/camera',
            '--remap',
            ['/camera/image:=', LaunchConfiguration("ros_namespace"), '/camera/image'],
            ['/camera/camera_info:=', LaunchConfiguration("ros_namespace"), '/camera/camera_info']
        ],
        name='rosbag',
        output='both',
        condition=IfCondition(LaunchConfiguration('replay'))
    )

    return LaunchDescription([
        DeclareLaunchArgument("record", default_value="False"),
        DeclareLaunchArgument("replay", default_value="False"),
        DeclareLaunchArgument("transport", default_value="False"),
        DeclareLaunchArgument("rectify", default_value="True"),
        record,
        replay,
        container_launch,
        LoadComposableNodes(
            target_container=container,
            composable_node_descriptions=[capture_comp],
            condition=UnlessCondition(LaunchConfiguration('replay'))
        ),
        LoadComposableNodes(
            target_container=container,
            composable_node_descriptions=[transport_comp],
            condition=IfCondition(LaunchConfiguration('transport'))
        ),
        LoadComposableNodes(
            target_container=container,
            composable_node_descriptions=[rectify_comp],
            condition=IfCondition(LaunchConfiguration('rectify'))
        )
    ])
