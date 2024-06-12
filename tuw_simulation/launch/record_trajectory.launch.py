from launch_ros.actions import LoadComposableNodes, PushRosNamespace, SetParameter, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, OrSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    tuw_simulation = FindPackageShare("tuw_simulation")
    
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
        package="tuw_simulation",
        executable="combined_trajectory.py",
        namespace=[LaunchConfiguration("model_name")]
    )

    return LaunchDescription([
        # Arguments
        SetParameter(name="use_sim_time", value="true"),
        DeclareLaunchArgument("model_name", default_value="robot0"),
        # Global Namespace
        simulation_world_launch,
        TimerAction(period=5.0, actions=[simulation_spawn_launch]),
        # Robot Namespace
        trajectory_driver
    ])
