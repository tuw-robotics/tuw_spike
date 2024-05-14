from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, AndSubstitution, NotSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    tuw_spike_camera = FindPackageShare("tuw_spike_camera")

    capture = LaunchConfiguration("capture")
    calibrate = LaunchConfiguration("calibrate")
    republish = AndSubstitution(
        NotSubstitution(LaunchConfiguration("capture")),
        LaunchConfiguration("calibrate")
    )

    transport = AndSubstitution(
        NotSubstitution(LaunchConfiguration("calibrate")),
        LaunchConfiguration("capture")
    )

    capture_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([tuw_spike_camera, "launch", "capture.launch.py"])),
        launch_arguments=[("transport", transport), ("transport_lores", "False"), ("rectify", "False")],
        condition=IfCondition(capture)
    )

    republish_node = Node(
        condition=IfCondition(republish),
        package="image_transport",
        executable="republish",
        arguments=[
            "compressed", "raw"
        ],
        namespace=LaunchConfiguration("camera"),
        remappings=[
            ("in/compressed", "image/compressed"),
            ("out", "image/uncompressed")
        ],
        output="both"
    )

    image_topic = PythonExpression([
        "'image/uncompressed' if '", republish , "' == 'true' else 'image'"
    ])

    calibration_node = Node(
        condition=IfCondition(calibrate),
        package="camera_calibration",
        executable="cameracalibrator",
        arguments=[
            "-s", LaunchConfiguration("dim"), "-q", LaunchConfiguration("size"),
        ],
        namespace=LaunchConfiguration("camera"),
        remappings=[
            ("image", image_topic),
            ("camera/set_camera_info", "set_camera_info")
        ],
        output="both"
    )

    # Copy calibration file via docker (in deploy directory):
    # docker -c lego0 cp calibrate-camera-1:/opt/ros/ros2_lego/ws02/install/tuw_spike_camera/share/tuw_spike_camera/calibration ../../src/ws02/tuw_spike_camera/

    return LaunchDescription([
        DeclareLaunchArgument("capture", default_value="True"),
        DeclareLaunchArgument("calibrate", default_value="True"),
        DeclareLaunchArgument("camera", default_value="camera"),
        DeclareLaunchArgument("dim", default_value="9x6"),
        DeclareLaunchArgument("size", default_value="0.024"),
        capture_launch,
        republish_node,
        calibration_node
    ])
