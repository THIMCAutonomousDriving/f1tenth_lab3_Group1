from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='racing_car_pkg',
            executable='reactive_node',
            name='reactive_node'
        ),
        Node(
            package='racing_car_pkg',
            executable='safety_node',
            name='safety_node',
            parameters=[
                # Setting 'target_frame' to 'base_link'
                {'sim_or_real': 'real'},
            ]
        ),
        #Node(
            #package='teleop_twist_keyboard',
            #executable='teleop_twist_keyboard',
            #name='teleop_twist_keyboard',
            #remappings=[
                #('/teleop', '/teleop_key'),
            #]
        #)
    ])