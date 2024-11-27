import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction

def handle_configuration(context, *args, **kwargs):
    config_path = os.path.join(os.path.dirname(__file__), '../config')
    config_file = os.path.join(config_path, 'vision.yaml') 
    config_local_file = os.path.join(config_path, 'vision_local.yaml') 

    return [
        Node(
            package='vision',
            executable='vision_node',
            name='vision_node',
            output='screen',
            arguments=[config_file, config_local_file]
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=handle_configuration)
    ])