from launch_ros.actions import LoadComposableNodes, PushRosNamespace, SetParameter, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, OrSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition

def robot_ns_from_hostname():
    import socket
    return socket.gethostname().replace("-", "_")

def generate_launch_description():
    tuw_simulation = FindPackageShare("tuw_spike_simulation")
    
    # Simulation
    simulation_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_simulation, "launch", "world.launch.py"]
        ))
    )
    simulation_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_simulation, "launch", "spawn_robot.launch.py"]
        ))
    )

    # Trajectory Driver
    trajectory_driver = Node(
        package="tuw_spike_analysis",
        executable="combined_trajectory"
    )

    return LaunchDescription([
        # Arguments
        SetParameter(name="use_sim_time", value="true"),
        DeclareLaunchArgument("robot_ns", default_value=robot_ns_from_hostname()),
        PushRosNamespace(LaunchConfiguration("robot_ns")),
        # Global Namespace
        simulation_world_launch,
        TimerAction(period=5.0, actions=[simulation_spawn_launch]),
        # Robot Namespace
        trajectory_driver
    ])
