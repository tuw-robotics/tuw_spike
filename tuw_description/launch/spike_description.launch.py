from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import Command, LaunchConfiguration, FindExecutable

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():    
    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("tuw_description"),
            "model",
            "spike",
            "main.xacro",
        ]),
        " namespace:=",
        LaunchConfiguration("ros_namespace")
    ])
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{
            "robot_description": robot_description_content
        }],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static")
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument("ros_namespace",  default_value=""),
        robot_state_publisher
    ])