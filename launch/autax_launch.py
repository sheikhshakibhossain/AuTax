from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autax',
            executable='base_control',
            name='base_control',
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
            executable='local_planner',
            name='local_planner',
            output='screen',
        ),
        Node(
            package='autax',
            executable='global_planner',
            name='global_planner',
            output='screen',
        ),
    ])