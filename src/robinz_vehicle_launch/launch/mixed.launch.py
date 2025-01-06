from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robinz_vehicle_launch',
            executable='cpp_node',
            name='cpp_node',
            output='screen'
        ),
        Node(
            package='robinz_vehicle_launch',
            executable='python_node.py',
            name='python_node',
            output='screen'
        )
    ])
