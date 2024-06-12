from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, LogInfo, Shutdown
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
from launch.events.process import ProcessExited

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    use_sim_time     = LaunchConfiguration('use_sim_time',  default='true')
    X_launch_arg     = DeclareLaunchArgument('X',           default_value=TextSubstitution(text='0.0'))
    Y_launch_arg     = DeclareLaunchArgument('Y',           default_value=TextSubstitution(text='0.0'))
    tuw_description = FindPackageShare("tuw_description")
    
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [tuw_description, "launch", "spike_description.launch.py"]
        ))
    )
    
    def spawner_exit(event: ProcessExited, ctx):
        code = event.returncode
        if code != 0:
            return Shutdown(reason="could not successfully spawn entity")
        else:
            return bridge
    
    spawner = Node(
        package="tuw_simulation",
        executable="spawn",
        parameters=[{
                "X": LaunchConfiguration('X'),
                "Y": LaunchConfiguration('Y')},],
        on_exit=spawner_exit
        
    )
     
    
    tuw_simulation = FindPackageShare("tuw_simulation")
    bridge_config = PathJoinSubstitution([tuw_simulation, "world", "tuw_simulation_bridge.yaml"])
    
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}, {'use_sim_time': True}, {'expand_gz_topic_names': True}]
    )
            
    return LaunchDescription(
        [
        DeclareLaunchArgument("ros_namespace", default_value="robot0"),
        X_launch_arg,
        Y_launch_arg,
        robot_state_publisher,
        spawner,
    ])