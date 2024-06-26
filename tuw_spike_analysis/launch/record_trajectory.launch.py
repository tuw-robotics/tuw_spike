from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    tuw_camera_laserscan = FindPackageShare("tuw_camera_laserscan")
    tuw_spike_control = FindPackageShare("tuw_spike_control")

    capture_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_camera_laserscan, "launch", "capture.launch.py"]
        )),
        launch_arguments=[
            ("rectify", "False")
        ]
    )

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_spike_control, "launch", "hardware.launch.py"]
        ))
    )

    # Trajectory driver and recording
    trajectory = LaunchConfiguration("trajectory")
    trajectory_driver = Node(
        package="tuw_spike_analysis",
        executable="trajectory_driver",
        parameters=[{
            "velocity": 0.1,
            "startup_delay": 5.0
        }],
        condition=IfCondition(trajectory)
    )

    record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/camera/image',
            '/camera/camera_info',
            '/tf',
            '/tf_static',
            '/joint_states',
            '/dynamic_joint_states',
            '/odom',
            '/odom_ground_truth'
        ],
        name='rosbag',
        output='both'
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument("trajectory", default_value="True"),
        capture_launch,
        hardware_launch,
        record,
        trajectory_driver
    ])
