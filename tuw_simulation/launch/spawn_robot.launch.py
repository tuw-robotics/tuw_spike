from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, LogInfo
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import Command, LaunchConfiguration, FindExecutable

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)


def generate_launch_description():

    use_sim_time     = LaunchConfiguration('use_sim_time',  default='true')
    model_name_arg   = DeclareLaunchArgument('model_name',  default_value=TextSubstitution(text='robot0'))
    X_launch_arg     = DeclareLaunchArgument('X',           default_value=TextSubstitution(text='0.0'))
    Y_launch_arg     = DeclareLaunchArgument('Y',           default_value=TextSubstitution(text='0.0'))
    
        
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
    
    spawner = Node(
        package="tuw_simulation",
        executable="spawn_robot.py",
        #namespace=[LaunchConfiguration('model_name')],
        parameters=[{
                "X": LaunchConfiguration('X'),
                "Y": LaunchConfiguration('Y'),
                "model_name": LaunchConfiguration('model_name')}],
        arguments=[robot_description_content]
    )
    
    params = {'robot_description': robot_description_content}
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='both',
                                  parameters=[params],
                                  arguments="urdf",
                                  namespace=[LaunchConfiguration('model_name')],)  
    
    tuw_simulation = FindPackageShare("tuw_simulation")
    bridge_config = PathJoinSubstitution([tuw_simulation, "world", "tuw_simulation_bridge.yaml"])
    
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}, {'use_sim_time': True}],
        namespace=[LaunchConfiguration("model_name")]
    )
            
    def error_handling(event):
        code = event.text.decode().strip()
        if code == '0':
            return [bridge, robot_state_publisher]
        else:
            return LogInfo(msg=f"aborting launch")

    return LaunchDescription(
        [
        X_launch_arg,
        Y_launch_arg,
        model_name_arg,
        spawner,
        RegisterEventHandler(
            OnProcessIO(
                target_action=spawner,
                on_stdout=error_handling
            )
        )
    ])