from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    use_sim_time     = LaunchConfiguration('use_sim_time',  default='true')
    namespace_arg    = DeclareLaunchArgument('namespace',   default_value=TextSubstitution(text=''))
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
    
    params = {'robot_description': robot_description_content}
    robot_state_publisher = launch_ros.actions.Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='both',
                                  parameters=[params],
                                  arguments="urdf",
                                  namespace=[LaunchConfiguration('model_name')],)    
   
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
    
    tuw_simulation = FindPackageShare("tuw_simulation")
    bridge_config = PathJoinSubstitution([tuw_simulation, "world", "tuw_simulation_bridge.yaml"])
    
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}, {'use_sim_time': True}],
        namespace=[LaunchConfiguration("model_name")]
    )

    return LaunchDescription(
        [
        X_launch_arg,
        Y_launch_arg,
        model_name_arg,
        namespace_arg,
        bridge,
        robot_state_publisher,
        spawner,
    ])