from launch_ros.actions import LoadComposableNodes, PushRosNamespace, SetParameter, Node, LifecycleNode
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, EmitEvent
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, AndSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_ros.events.lifecycle import ChangeState
from launch.events.matchers import matches_action

from lifecycle_msgs.msg import Transition


def robot_ns_from_hostname():
    import socket
    return socket.gethostname().replace("-", "_")

def generate_launch_description():
    tuw_spike_analysis = FindPackageShare("tuw_spike_analysis")

    optitrack_node = LifecycleNode(
        name='mocap4r2_optitrack_driver_node',
        namespace='',
        package='mocap4r2_optitrack_driver',
        executable='mocap4r2_optitrack_driver_main',
        output='screen',
        parameters=[
            PathJoinSubstitution([tuw_spike_analysis, "config", "optitrack.yaml"])
        ],
    )

    # Make the driver node take the 'configure' transition
    optitrack_configure_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(optitrack_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    optitrack_activate_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(optitrack_node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    optitrack_to_odom = Node(
        package="tuw_spike_analysis",
        executable="optitrack_to_odom"
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_ns", default_value=robot_ns_from_hostname()),
        GroupAction([
            PushRosNamespace(LaunchConfiguration("robot_ns")),
            optitrack_node,
            optitrack_configure_trans_event,
            optitrack_activate_trans_event,
            optitrack_to_odom
        ])
    ])
