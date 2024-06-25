from launch_ros.actions import LoadComposableNodes, PushRosNamespace, SetParameter, Node, LifecycleNode
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, AndSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch_ros.events.lifecycle import ChangeState
from launch.events.matchers import matches_action

from lifecycle_msgs.msg import Transition


def robot_ns_from_hostname():
    import socket
    return socket.gethostname().replace("-", "_")

def generate_launch_description():
    tuw_spike_analysis = FindPackageShare("tuw_spike_analysis")
    tuw_camera_laserscan = FindPackageShare("tuw_camera_laserscan")

    params_yaml = PathJoinSubstitution([tuw_camera_laserscan, "config", "amcl.yaml"])

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_yaml,
            {'yaml_filename': PathJoinSubstitution([tuw_camera_laserscan, "maps", "map1.yaml"])}
        ]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    # Replay capture
    capture_replay_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_spike_analysis, "launch", "replay_trajectory.launch.py"]
        ))
    )

    optitrack_to_tf = Node(
        package="tuw_spike_analysis",
        executable="optitrack_to_tf"
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument("robot_ns", default_value=robot_ns_from_hostname()),
        SetParameter("use_sim_time", True),
        GroupAction([
            PushRosNamespace(LaunchConfiguration("robot_ns")),
            capture_replay_launch,
            optitrack_to_tf,
            map_server,
            lifecycle_manager
        ])
    ])
