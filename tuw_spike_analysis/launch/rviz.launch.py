from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def robot_ns_from_hostname():
    import socket
    return socket.gethostname().replace("-", "_")

def generate_launch_description():
    tuw_spike_analysis = FindPackageShare("tuw_spike_analysis")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace=LaunchConfiguration("robot_ns"),
        parameters=[{
            "use_sim_time": LaunchConfiguration("sim")
        }],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static")
        ],
        arguments=[
            "-d", PathJoinSubstitution([tuw_spike_analysis, "config", "config.rviz"])
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="False"),
        DeclareLaunchArgument("robot_ns", default_value=robot_ns_from_hostname()),
        rviz
    ])
