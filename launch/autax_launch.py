from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autax',
            executable='wheel',
            name='wheel_node',
            output='screen',
        ),
        Node(
            package='autax',
            executable='ai_agent',
            name='ai_agent_node',
            output='screen',
        ),
        Node(
            package='autax',
            executable='vision',
            name='vision_node',
            output='screen',
        ),
        Node(
            package='autax',
            executable='gnss',
            name='gnss_node',
            output='screen',
        ),
    ])