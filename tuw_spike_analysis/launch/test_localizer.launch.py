from launch_ros.actions import LoadComposableNodes, PushRosNamespace, SetParameter, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, OrSubstitution, AndSubstitution, NotSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition


def robot_ns_from_hostname():
    import socket
    return socket.gethostname().replace("-", "_")

def generate_launch_description():
    tuw_camera_laserscan = FindPackageShare("tuw_camera_laserscan")
    tuw_spike_description = FindPackageShare("tuw_description")
    tuw_simulation = FindPackageShare("tuw_simulation")
    tuw_spike_control = FindPackageShare("tuw_spike_control")
    simulation = LaunchConfiguration("simulation")

    # Simulation captrue
    simulation_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_simulation, "launch", "world.launch.py"]
        )),
        condition=IfCondition(simulation)
    )

    capture_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_camera_laserscan, "launch", "capture_simulation.launch.py"]
        )),
        condition=IfCondition(simulation)
    )

    # Real capture
    capture_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_camera_laserscan, "launch", "capture.launch.py"]
        )),
        condition=UnlessCondition(simulation)
    )

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_spike_control, "launch", "hardware.launch.py"]
        )),
        condition=UnlessCondition(simulation)
    )

    # Localizer
    camera_proc_container = [
        LaunchConfiguration("ros_namespace"), "/camera_processing_container"
    ]
    localizer_comp = ComposableNode(
        package='tuw_camera_laserscan',
        plugin='tuw_camera_laserscan::RayLocalizerNode',
        name='ray_localizer',
        extra_arguments=[{'use_intra_process_comms': True}],
        parameters=[ParameterFile(PathJoinSubstitution([tuw_camera_laserscan, "config", "localizer.yaml"]))],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # AMCL
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([tuw_camera_laserscan, "launch", "amcl.launch.py"]))
    )

    # Trajectory driver and recording
    trajectory = LaunchConfiguration("trajectory")
    trajectory_driver = Node(
        package="tuw_spike_analysis",
        executable="trajectory_driver",
        parameters=[{
            "velocity": 0.05,
            "startup_delay": 5.0
        }],
        condition=IfCondition(trajectory)
    )

    trajectory_est_recoder = Node(
        package="tuw_spike_analysis",
        executable="trajectory_est_recorder",
        condition=IfCondition(trajectory)
    )

    trajectory_sim_recoder = Node(
        package="tuw_spike_analysis",
        executable="trajectory_sim_recorder",
        condition=IfCondition(AndSubstitution(trajectory, simulation))
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument("debug", default_value="False"),
        DeclareLaunchArgument("replay", default_value="False"),
        DeclareLaunchArgument("simulation", default_value="False"),
        DeclareLaunchArgument("robot_ns", default_value=robot_ns_from_hostname()),
        DeclareLaunchArgument("trajectory", default_value="False"),
        SetParameter(name="use_sim_time", value=OrSubstitution(
            simulation,
            LaunchConfiguration("replay")
        )),
        # Launch simulation world
        simulation_world_launch,
        GroupAction([
            PushRosNamespace(LaunchConfiguration("robot_ns")),
            # Hardware
            capture_launch,
            hardware_launch,
            # Simulation
            capture_sim_launch,
            # Camera to Laserscan
            LoadComposableNodes(
                target_container=camera_proc_container,
                composable_node_descriptions=[localizer_comp]
            ),
            # AMCL
            amcl_launch,
            # Test drivers
            trajectory_driver,
            trajectory_est_recoder,
            trajectory_sim_recoder
        ])
    ])
