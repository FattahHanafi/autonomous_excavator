from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os.path

urdf_file_name = 'electrical_excavator.urdf'
urdf = os.path.join(get_package_share_directory('autonomous_excavator'), 'urdf', urdf_file_name)
print(urdf)
with open(urdf, 'r') as infp:
    robot_desc = infp.read()

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonomous_excavator',
            executable='joint_publisher',
            ),
        #  Node(
            #  package='autonomous_excavator',
            #  executable='excavator_tf2_broadcaster',
        #  ),
        Node(
            package='autonomous_excavator',
            executable='blade_node',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf]),
        #  Node(
            #  package='rviz2',
            #  executable='rviz2',
            #  arguments=['-d', [os.path.join(get_package_share_directory('autonomous_excavator'), 'config', 'machine_visualizer.rviz')]]
            #  ),
    ])
