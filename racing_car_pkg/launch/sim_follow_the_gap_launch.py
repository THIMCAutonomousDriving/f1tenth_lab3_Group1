from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='racing_car_pkg',
            executable='reactive_node',
            name='reactive_node',
            parameters=[
                {'sim_or_real': 'sim'},
            ]
        ),
        Node(
            package='racing_car_pkg',
            executable='safety_node',
            name='safety_node',
            parameters=[
                {'sim_or_real': 'sim'},
            ]
        ),
    ])