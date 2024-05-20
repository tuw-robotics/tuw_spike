from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    tuw_spike_camera = FindPackageShare("tuw_spike_camera")
    tuw_spike_description = FindPackageShare("tuw_description")

    capture_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_spike_camera, "launch", "capture.launch.py"]
        ))
    )

    robot_state_pub_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_spike_description, "launch", "spike_description.launch.py"]
        ))
    )

    localizer_comp = ComposableNode(
        package='tuw_spike_camera',
        plugin='tuw_spike_camera::RayLocalizerNode',
        extra_arguments=[{'use_intra_process_comms': True}],
        namespace="camera",
        parameters=[{
            'ray_frame': 'ray_origin',
            'num_rays': 50,
            'debug_img_size': 1280,
            'debug_real_size': 0.4
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument("replay", default_value="False"),
        DeclareLaunchArgument("debug", default_value="False"),
        DeclareLaunchArgument('model_name',  default_value="robot0"),
        capture_launch,
        robot_state_pub_launch,
        LoadComposableNodes(
            target_container='camera_processing_container',
            composable_node_descriptions=[localizer_comp]
        )
    ])
