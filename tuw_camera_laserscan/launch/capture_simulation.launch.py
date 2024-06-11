from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    tuw_camera_laserscan = FindPackageShare("tuw_camera_laserscan")
    tuw_simulation = FindPackageShare("tuw_simulation")

    container_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([tuw_camera_laserscan, "launch", "container.launch.py"]))
    )

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

    container_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([tuw_camera_laserscan, "launch", "container.launch.py"]))
    )

    return LaunchDescription([
        simulation_world_launch,
        TimerAction(period=5.0, actions=[simulation_spawn_launch]),
        GroupAction([
            PushRosNamespace(LaunchConfiguration("model_name")),
            container_launch
        ])
    ])
