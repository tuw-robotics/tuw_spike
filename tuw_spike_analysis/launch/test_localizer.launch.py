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
    tuw_camera_laserscan = FindPackageShare("tuw_camera_laserscan")
    tuw_spike_simulation = FindPackageShare("tuw_spike_simulation")
    tuw_spike_control = FindPackageShare("tuw_spike_control")
    tuw_spike_analysis = FindPackageShare("tuw_spike_analysis")

    source_hw = LaunchConfigurationEquals("source", "hardware")
    source_sim = LaunchConfigurationEquals("source", "simulation")
    source_bag = LaunchConfigurationEquals("source", "bag")
    source_not_hw = LaunchConfigurationNotEquals("source", "hardware")
    source_not_bag = LaunchConfigurationNotEquals("source", "bag")

    # Simulation captrue
    simulation_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_spike_simulation, "launch", "world.launch.py"]
        )),
        condition=source_sim
    )

    capture_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_camera_laserscan, "launch", "capture_simulation.launch.py"]
        )),
        condition=source_sim
    )

    # Real capture
    capture_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_camera_laserscan, "launch", "capture.launch.py"]
        )),
        condition=source_hw
    )

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_spike_control, "launch", "hardware.launch.py"]
        )),
        condition=source_hw
    )

    # Replay capture
    capture_replay_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_spike_analysis, "launch", "replay_trajectory.launch.py"]
        )),
        condition=source_bag
    )

    # Localizer
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

    # AMCL
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([tuw_camera_laserscan, "launch", "amcl.launch.py"]))
    )

    # Trajectory driver and recording
    trajectory_driver = Node(
        package="tuw_spike_analysis",
        executable="trajectory_driver",
        parameters=[{
            "velocity": 0.1,
            "startup_delay": 5.0
        }],
        condition=source_not_bag
    )

    trajectory_est_recoder = Node(
        package="tuw_spike_analysis",
        executable="trajectory_est_recorder"
    )

    trajectory_true_recoder = Node(
        package="tuw_spike_analysis",
        executable="trajectory_true_recorder",
        condition=source_not_hw
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument("debug", default_value="False"),
        DeclareLaunchArgument("source", default_value="hardware"),
        DeclareLaunchArgument("robot_ns", default_value=robot_ns_from_hostname()),
        DeclareLaunchArgument("trajectory", default_value="False"),
        SetParameter("use_sim_time", True, condition=source_sim),
        SetParameter("use_sim_time", True, condition=source_bag),
        # Launch simulation world
        simulation_world_launch,
        GroupAction([
            PushRosNamespace(LaunchConfiguration("robot_ns")),
            # Hardware
            capture_launch,
            hardware_launch,
            # Simulation
            capture_sim_launch,
            # Bag replay
            capture_replay_launch,
            # Camera to Laserscan
            LoadComposableNodes(
                target_container=camera_proc_container,
                composable_node_descriptions=[localizer_comp]
            ),
            # AMCL
            amcl_launch,
            # Test drivers
            GroupAction([
                trajectory_driver,
                trajectory_est_recoder,
                trajectory_true_recoder,
            ], condition=IfCondition(LaunchConfiguration("trajectory"))),
        ])
    ])
