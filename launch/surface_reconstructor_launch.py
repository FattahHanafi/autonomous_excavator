from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonomous_excavator',
            executable='excavator_tf2_broadcaster',
        ),
        Node(
            package='autonomous_excavator',
            executable='camera_reader',
        ),
        Node(
            package='autonomous_excavator',
            executable='roi_node',
        ),
        Node(
            package='autonomous_excavator',
            executable='surface_reconstructor',
        )
    ])
