from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, Shutdown
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch.events.process import ProcessExited

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    tuw_description = FindPackageShare("tuw_description")
    
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_description, "launch", "spike_description.launch.py"]
        ))
    )
    
    def spawner_exit(event: ProcessExited, ctx):
        code = event.returncode
        if code != 0:
            return Shutdown(reason="Could not successfully spawn robot")
    
    spawner = Node(
        package="tuw_simulation",
        executable="spawn",
        parameters=[{
            "X": LaunchConfiguration("X"),
            "Y": LaunchConfiguration("Y")
        }],
        on_exit=spawner_exit
    )
     
    tuw_simulation = FindPackageShare("tuw_simulation")
    bridge_config = PathJoinSubstitution([tuw_simulation, "world", "tuw_simulation_bridge.yaml"])
    
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": bridge_config,
            "expand_gz_topic_names": True
        }]
    )
            
    return LaunchDescription([
        DeclareLaunchArgument("ros_namespace", default_value=""),
        DeclareLaunchArgument("X", default_value="0.0"),
        DeclareLaunchArgument("Y", default_value="0.0"),
        SetParameter("use_sim_time", True),
        robot_state_publisher,
        bridge,
        spawner,
    ])