from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def robot_ns_from_hostname():
    import socket
    return socket.gethostname().replace("-", "_")

def generate_launch_description():
    rqt = Node(
        package="rqt_gui",
        executable="rqt_gui",
        namespace=LaunchConfiguration("robot_ns"),
        parameters=[{
            "use_sim_time": LaunchConfiguration("sim")
        }],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static")
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="False"),
        DeclareLaunchArgument("robot_ns", default_value=robot_ns_from_hostname()),
        rqt
    ])
