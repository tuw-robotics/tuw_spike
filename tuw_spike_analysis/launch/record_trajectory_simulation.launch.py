from launch_ros.actions import PushRosNamespace, SetParameter, Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition


def robot_ns_from_hostname():
    import socket
    return socket.gethostname().replace("-", "_")

def generate_launch_description():
    tuw_simulation = FindPackageShare("tuw_spike_simulation")
    tuw_spike_control = FindPackageShare("tuw_spike_control")
    DeclareLaunchArgument("simulation", default_value='true'),

    
    # Simulation
    simulation_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_simulation, "launch", "world.launch.py"]
        )),
        condition=IfCondition(LaunchConfiguration("simulation"))
    )
    simulation_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_simulation, "launch", "spawn_robot.launch.py"]
        )),
        condition=IfCondition(LaunchConfiguration("simulation"))
    )
    
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_spike_control, "launch", "hardware.launch.py"]
        )),
        condition=UnlessCondition(LaunchConfiguration("simulation"))
    )

    # Trajectory Driver
    trajectory_driver = Node(
        package="tuw_spike_analysis",
        executable="curve_trajectory"
    )
    
    record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            *(
                [LaunchConfiguration("robot_ns"), topic]
                for topic in (
                    "/tf",
                    "/tf_static",
                    "/odom_ground_truth",
                    '/odom'
                )
            ),
        ],
        name='rosbag',
        output='both'
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
        trajectory_driver,
        hardware_launch,
        record
    ])
