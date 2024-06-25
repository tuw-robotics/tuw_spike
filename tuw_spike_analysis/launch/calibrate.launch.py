from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, AndSubstitution, NotSubstitution, OrSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    tuw_camera_laserscan = FindPackageShare("tuw_camera_laserscan")
    tuw_spike_description = FindPackageShare("tuw_spike_description")

    capture = LaunchConfiguration("capture")
    intrinsic = LaunchConfiguration("cal_intrinsic")
    extrinsic = LaunchConfiguration("cal_extrinsic")
    calibrate = OrSubstitution(intrinsic, extrinsic)
    republish = AndSubstitution(
        NotSubstitution(capture),
        calibrate
    )
    transport = AndSubstitution(
        NotSubstitution(calibrate),
        capture
    )

    capture_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([tuw_camera_laserscan, "launch", "capture.launch.py"])),
        launch_arguments=[
            ("transport", transport),
            ("transport_lores", "False"),
            ("rectify", "False")
        ],
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

    intrinsic_calibration_node = Node(
        condition=IfCondition(intrinsic),
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

    extrinsic_calibration_node = Node(
        condition=IfCondition(extrinsic),
        package="tuw_spike_analysis",
        executable="extrinsic_calibration",
        output="both",
        namespace=LaunchConfiguration("camera"),
        remappings=[
            ("image", image_topic),
        ]
    )

    robot_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_spike_description, "launch", "spike_description.launch.py"]
        )),
        condition=IfCondition(extrinsic)
    )

    # Copy calibration file via docker (in deploy directory):
    # docker -c lego0 cp calibrate-camera-1:/opt/ros/ros2_lego/ws02/install/tuw_camera_laserscan/share/tuw_camera_laserscan/calibration ../../src/ws02/tuw_camera_laserscan/

    return LaunchDescription([
        DeclareLaunchArgument("capture", default_value="True"),
        DeclareLaunchArgument("cal_intrinsic", default_value="False"),
        DeclareLaunchArgument("cal_extrinsic", default_value="False"),
        DeclareLaunchArgument("camera", default_value="camera"),
        DeclareLaunchArgument("dim", default_value="9x6"),
        DeclareLaunchArgument("size", default_value="0.036"),
        capture_launch,
        republish_node,
        intrinsic_calibration_node,
        extrinsic_calibration_node,
        robot_state_pub
    ])
