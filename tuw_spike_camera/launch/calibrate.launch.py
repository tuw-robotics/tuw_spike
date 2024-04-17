from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    tuw_spike_camera = FindPackageShare("tuw_spike_camera")

    calibration_node = Node(
        package='camera_calibration',
        executable='cameracalibrator',
        arguments=['-s', '9x6', '-q', '0.036'],
        namespace="camera",
        remappings=[("camera/set_camera_info", "set_camera_info")],
        output="both"
    )
    
    capture = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([tuw_spike_camera, "launch", "capture.launch.py"]))
    )

    return LaunchDescription([
        #capture,
        calibration_node
    ])
