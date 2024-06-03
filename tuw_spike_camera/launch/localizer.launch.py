from launch_ros.actions import LoadComposableNodes, PushRosNamespace, SetParameter, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, OrSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    tuw_spike_camera = FindPackageShare("tuw_spike_camera")
    tuw_spike_description = FindPackageShare("tuw_description")
    tuw_simulation = FindPackageShare("tuw_simulation")
    simulation = LaunchConfiguration("simulation")

    container = "camera_processing_container"

    # Simulation
    simulation_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_simulation, "launch", "world.launch.py"]
        )),
        condition=IfCondition(simulation)
    )
    simulation_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_simulation, "launch", "spawn_robot.launch.py"]
        )),
        condition=IfCondition(simulation)
    )
    container_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([tuw_spike_camera, "launch", "container.launch.py"])),
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
        extra_arguments=[{'use_intra_process_comms': True}],
        parameters=[{
            'ray_frame': 'ray_origin',
            'num_rays': 200,
            'debug_img_size': 1280,
            'debug_real_size': 0.6
        }]
    )

    # AMCL
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([tuw_spike_camera, "launch", "amcl.launch.py"]))
    )

    # Trajectory Driver
    trajectory_driver = Node(
        package="tuw_spike_camera",
        executable="test_trajectory_driver"
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument("debug", default_value="False"),
        DeclareLaunchArgument("replay", default_value="False"),
        DeclareLaunchArgument("simulation", default_value="False"),
        DeclareLaunchArgument("model_name", default_value="robot0"),
        SetParameter(name="use_sim_time", value=OrSubstitution(
            simulation,
            LaunchConfiguration("replay")
        )),
        # Global Namespace
        simulation_world_launch,
        TimerAction(period=5.0, actions=[simulation_spawn_launch]),
        robot_state_pub_launch,
        # Robot Namespace
        GroupAction([
            PushRosNamespace(LaunchConfiguration("model_name")),
            container_launch,
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
