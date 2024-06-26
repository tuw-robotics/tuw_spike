from launch_ros.actions import PushRosNamespace, SetParameter, Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess, GroupAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition


def robot_ns_from_hostname():
    import socket
    return socket.gethostname().replace("-", "_")

def generate_launch_description():
    tuw_simulation = FindPackageShare("tuw_spike_simulation")
    tuw_spike_control = FindPackageShare("tuw_spike_control")

    sim = IfCondition(LaunchConfiguration("simulation"))
    no_sim = UnlessCondition(LaunchConfiguration("simulation"))
    
    # Simulation
    simulation_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_simulation, "launch", "world.launch.py"]
        )),
        condition=sim
    )

    simulation_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_simulation, "launch", "spawn_robot.launch.py"]
        )),
        condition=sim
    )
    
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_spike_control, "launch", "hardware.launch.py"]
        )),
        condition=no_sim
    )

    # Trajectory Driver
    trajectory_driver = Node(
        package="tuw_spike_analysis",
        executable="curve_trajectory",
        parameters=[{
                "velocity": 0.3,
                "radius": 0.15,
                "delay" : 1.0,
            }]
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
                    "/odom"
                )
            ),
        ],
        name='rosbag',
        output='both'
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id", "map", "--child-frame-id", "odom"
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static")
        ]
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument("robot_ns", default_value=robot_ns_from_hostname()),
        DeclareLaunchArgument("simulation", default_value="true"),
        SetParameter("use_sim_time", True, condition=sim),
        # Global Namespace
        simulation_world_launch,
        record,
        # Robot Namespace
        GroupAction([
            PushRosNamespace(LaunchConfiguration("robot_ns")),
            TimerAction(period=5.0, actions=[simulation_spawn_launch]),
            trajectory_driver,
            hardware_launch,
            static_tf
        ])
    ])
