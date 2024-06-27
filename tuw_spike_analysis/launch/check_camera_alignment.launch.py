from launch_ros.actions import PushRosNamespace, SetParameter, Node, LoadComposableNodes
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import math


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
        executable="optitrack_to_tf",
        parameters=[{
            "frame": "optitrack",
            "child_frame": "base_odom"
        }]
    )
    
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id", "map", "--child-frame-id", "optitrack",
            "--yaw", str(math.radians(0.0))
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static")
        ]
    )

    camera_proc_container = [
        LaunchConfiguration("ros_namespace"), "/camera_processing_container"
    ]
    localizer_comp = ComposableNode(
        package='tuw_camera_laserscan',
        plugin='tuw_camera_laserscan::RayLocalizerNode',
        name='ray_localizer',
        extra_arguments=[{'use_intra_process_comms': True}],
        parameters=[ParameterFile(PathJoinSubstitution([tuw_camera_laserscan, "config", "localizer.yaml"]))],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument("robot_ns", default_value=robot_ns_from_hostname()),
        SetParameter("use_sim_time", True),
        GroupAction([
            PushRosNamespace(LaunchConfiguration("robot_ns")),
            capture_replay_launch,
            static_tf,
            optitrack_to_tf,
            map_server,
            lifecycle_manager,
            LoadComposableNodes(
                target_container=camera_proc_container,
                composable_node_descriptions=[localizer_comp]
            )
        ])
    ])
