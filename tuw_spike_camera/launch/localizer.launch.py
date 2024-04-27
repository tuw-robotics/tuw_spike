from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    replay = IfCondition(LaunchConfiguration("replay"))
    not_replay = UnlessCondition(LaunchConfiguration("replay"))

    capture_comp = ComposableNode(
        condition=not_replay,
        package='tuw_libcamera',
        plugin='tuw_libcamera::CaptureNode',
        extra_arguments=[{'use_intra_process_comms': True}],
        namespace="camera",
        parameters=[{
            "camera_info_name": "camera",
            "camera_info_url": "package://tuw_spike_camera/calibration/${NAME}.yaml",
            "frame_id": "camera_optical",
            "stream_roles": ["video"],
            "streams.video": {
                "format": "RGB888",
                "target_format": "bgr8",
                "width": 1280,
                "height": 720
            }
        }]
    )

    transport_comp = ComposableNode(
        package='tuw_libcamera',
        plugin='tuw_libcamera::TransportNode',
        extra_arguments=[{'use_intra_process_comms': True}],
        namespace="camera"
    )

    undistort_comp = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        extra_arguments=[{'use_intra_process_comms': True}],
        namespace="camera"
    )

    localizer_comp = ComposableNode(
        package='tuw_spike_camera',
        plugin='tuw_spike_camera::RayLocalizerNode',
        extra_arguments=[{'use_intra_process_comms': True}],
        namespace="camera",
        parameters=[{
            'ray_frame': 'ray_origin'
        }]
    )

    rosbag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '--loop', 'bags/camera'],
        name='rosbag',
        output='both',
        condition=replay
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("tuw_simulation"),
                    "model",
                    "spike",
                    "main.xacro",
                ]
            ),
            " namespace:=",
            LaunchConfiguration('model_name')
        ]
    )

    params = {'robot_description': robot_description_content}
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='both',
                                  parameters=[params],
                                  namespace=[LaunchConfiguration('model_name')],)  

    container = ComposableNodeContainer(
        name='camera_processing_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        #ros_arguments=["--log-level", "debug"],
        #prefix='gdbserver :2222',
        composable_node_descriptions=[
            capture_comp,
            undistort_comp,
            #transport_comp,
            localizer_comp
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument("replay", default_value="False"),
        DeclareLaunchArgument('model_name',  default_value="robot0"),
        container,
        robot_state_publisher,
        rosbag
    ])
