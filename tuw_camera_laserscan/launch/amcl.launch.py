from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    tuw_camera_laserscan = FindPackageShare("tuw_camera_laserscan")

    params_yaml = PathJoinSubstitution([tuw_camera_laserscan, "config", "amcl.yaml"])

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_yaml,
            {'yaml_filename': PathJoinSubstitution([tuw_camera_laserscan, "maps", "map1.yaml"])}
        ]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_yaml],
        ros_arguments=["--log-level", "robot0.amcl:=debug"],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['amcl', 'map_server']
        }]
    )

    return LaunchDescription([
        map_server,
        amcl,
        lifecycle_manager
    ])
