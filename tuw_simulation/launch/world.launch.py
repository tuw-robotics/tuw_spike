from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tuw_simulation = FindPackageShare("tuw_simulation")
    ros_gz_sim = FindPackageShare("ros_gz_sim") 
    
    # Start simulation
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([ros_gz_sim, "launch", "gz_sim.launch.py"])),
        launch_arguments={
            # Launch simulation automatically started (-r)
            "gz_args": [PathJoinSubstitution([tuw_simulation, "world", "empty.sdf"]), TextSubstitution(text=" ")],
            "on_exit_shutdown": "True"
        }.items()
    )
    
    bridge_config = PathJoinSubstitution([tuw_simulation, "world", "tuw_simulation_bridge_clock.yaml"])
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": bridge_config,
            "expand_gz_topic_names": True
        }]
    )
    
    return LaunchDescription([
        SetParameter("use_sim_time", True),
        gz_sim,
        bridge
    ])