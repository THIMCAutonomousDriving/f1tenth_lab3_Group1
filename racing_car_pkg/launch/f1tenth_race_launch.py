from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_follow',
            executable='wall_follow_node.py',
            name='wall_follow_node'
        ),
        Node(
            package='wall_follow',
            executable='aeb.py',
            name='aeb'
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