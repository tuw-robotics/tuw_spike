from launch_ros.actions import LoadComposableNodes, PushRosNamespace, SetParameter, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, OrSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition



def generate_launch_description():
    tuw_spike_description = FindPackageShare("tuw_spike_description")
    tuw_spike_control = FindPackageShare("tuw_spike_control")
    # Load controller paramter file
    robot_controllers = PathJoinSubstitution([tuw_spike_control, "config", "controllers.yaml"])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        # ros_arguments=["--log-level", "debug"],
        output="both",
        remappings=[
            ('controller_manager/robot_description', 'robot_description'),
            ('controller_diff_drive/cmd_vel', 'cmd_vel'),
            ('controller_diff_drive/odom', 'odom'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    robot_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_spike_description, "launch", "spike_description.launch.py"]
        )),
    )

    spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "controller_diff_drive",
            "--controller-manager", "controller_manager"
        ]
    )

    return LaunchDescription([
        control_node,
        robot_state_pub,
        spawner_node
    ])
