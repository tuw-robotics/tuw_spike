from launch_ros.actions import LoadComposableNodes, PushRosNamespace, SetParameter, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, OrSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition


def robot_ns_from_hostname():
    import socket
    return socket.gethostname().replace("-", "_")

def generate_launch_description():
    tuw_spike_camera = FindPackageShare("tuw_spike_camera")
    tuw_spike_description = FindPackageShare("tuw_description")
    simulation = LaunchConfiguration("simulation")

    # Simulation captrue
    capture_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_spike_camera, "launch", "capture_simulation.launch.py"]
        )),
        condition=IfCondition(simulation)
    )

    # Real capture
    capture_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_spike_camera, "launch", "capture.launch.py"]
        )),
        condition=UnlessCondition(simulation)
    )

    robot_state_pub_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_spike_description, "launch", "spike_description.launch.py"]
        )),
        condition=UnlessCondition(simulation)
    )

    # Localizer
    localizer_comp = ComposableNode(
        package='tuw_spike_camera',
        plugin='tuw_spike_camera::RayLocalizerNode',
        name='ray_localizer',
        extra_arguments=[{'use_intra_process_comms': True}],
        parameters=[ParameterFile(PathJoinSubstitution([tuw_spike_camera, "config", "localizer.yaml"]))]
    )

    # AMCL
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([tuw_spike_camera, "launch", "amcl.launch.py"]))
    )

    # Trajectory Driver
    trajectory_driver = Node(
        package="tuw_spike_camera",
        executable="test_trajectory_driver",
        parameters=[{"velocity": 0.1}]
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument("debug", default_value="False"),
        DeclareLaunchArgument("replay", default_value="False"),
        DeclareLaunchArgument("simulation", default_value="False"),
        DeclareLaunchArgument("model_name", default_value=robot_ns_from_hostname()),
        SetParameter(name="use_sim_time", value=OrSubstitution(
            simulation,
            LaunchConfiguration("replay")
        )),
        robot_state_pub_launch,
        capture_sim_launch,
        GroupAction([
            PushRosNamespace(LaunchConfiguration("model_name")),
            capture_launch,
            LoadComposableNodes(
                target_container=[
                    LaunchConfiguration("ros_namespace"), "/camera_processing_container"
                ],
                composable_node_descriptions=[localizer_comp]
            ),
            amcl_launch,
            trajectory_driver
        ])
    ])
