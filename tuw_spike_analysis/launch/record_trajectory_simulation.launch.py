from launch_ros.actions import PushRosNamespace, SetParameter, Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

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
        record
    ])
