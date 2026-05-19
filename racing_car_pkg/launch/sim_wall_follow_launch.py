from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='racing_car_pkg',
            executable='wall_follow_node',
            name='wall_follow_node',
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