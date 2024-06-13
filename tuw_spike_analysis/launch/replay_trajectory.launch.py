from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    tuw_camera_laserscan = FindPackageShare("tuw_camera_laserscan")

    container = [
        LaunchConfiguration("ros_namespace"), "/camera_processing_container"
    ]

    container_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([tuw_camera_laserscan, "launch", "container.launch.py"]))
    )

    rectify_comp = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        extra_arguments=[{'use_intra_process_comms': True}],
        namespace="camera",
        name="rectify"
    )

    replay = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            'bags/trajectory_combined',
            '--remap',
            *(
                [topic, ":=", LaunchConfiguration("ros_namespace"), topic]
                for topic in (
                    "/camera/image",
                    "/camera/camera_info",
                    "/tf",
                    "/tf_static"
                )
            ),
            '-r', LaunchConfiguration("rate"),
            '--clock'
        ],
        name='rosbag',
        output='both'
    )

    return LaunchDescription([
        DeclareLaunchArgument("ros_namespace", default_value=""),
        DeclareLaunchArgument("rectify", default_value="True"),
        DeclareLaunchArgument("rate", default_value="1.0"),
        replay,
        container_launch,
        LoadComposableNodes(
            target_container=container,
            composable_node_descriptions=[rectify_comp],
            condition=IfCondition(LaunchConfiguration('rectify'))
        )
    ])
