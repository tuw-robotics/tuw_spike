from launch_ros.actions import LoadComposableNodes, PushRosNamespace, SetParameter, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, OrSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import SetLaunchConfiguration
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution 



def generate_launch_description():
    tuw_spike_description = FindPackageShare("tuw_spike_description")
    tuw_spike_control = FindPackageShare("tuw_spike_control")
    # Load controller paramter file
    def controllers_config_fnc(context):
        file = PathJoinSubstitution([tuw_spike_control, "config", context.launch_configurations['controllers_config']])
        return [SetLaunchConfiguration('controllers_config', file)]

    controllers_config_ofnc = OpaqueFunction(function=controllers_config_fnc)
    
    controllers_config_arg = DeclareLaunchArgument('controllers_config', 
                default_value=TextSubstitution(text='controllers.yaml'), 
                description='controller paramter file')
    

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[LaunchConfiguration('controllers_config')],
        # ros_arguments=["--log-level", "debug"],
        output="both",
        remappings=[
            ('controller_manager/robot_description', 'robot_description'),
            ('controller_diff_drive/cmd_vel', 'cmd_vel'),   #   ('controller_diff_drive/cmd_vel_unstamped', 'cmd_vel_unstamped'),
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

    # Trajectory driver and recording
    trajectory_driver = Node(
        package="tuw_spike_analysis",
        executable="trajectory_driver",
        parameters=[{
            "velocity": 0.2,
            "startup_delay": 5.0
        }],
    )

    # Trajectory driver and recording
    one_meter_forward = Node(
        package="tuw_spike_analysis",
        executable="one_meter_forward",
    )

    spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "controller_diff_drive",
            "--controller-manager", "controller_manager",
            "--controller-manager-timeout", "20",
            "--param-file", LaunchConfiguration('controllers_config'),
        ]
    )

    return LaunchDescription([
        controllers_config_arg,
        controllers_config_ofnc,
        control_node,
        robot_state_pub,
        spawner_node,
        # trajectory_driver
        # one_meter_forward
    ])
