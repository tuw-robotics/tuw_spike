from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PythonExpression, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    prefix = PythonExpression([
        "'gdbserver :2222' if bool(", LaunchConfiguration("debug"), ") else ''"
    ])
    container = Node(
        name="camera_processing_container",
        package="rclcpp_components",
        executable="component_container",
        output="both",
        prefix=prefix
    )
    return LaunchDescription([
        DeclareLaunchArgument("debug", default_value="False"),
        container
    ])
