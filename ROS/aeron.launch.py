from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aeron',
            executable='speech_processor',
            name='speech_processor'
        ),
        Node(
            package='aeron',
            executable='nlp_processor',
            name='nlp_processor'
        ),
        Node(
            package='aeron',
            executable='robot_controller',
            name='robot_controller'
        ),
    ])
