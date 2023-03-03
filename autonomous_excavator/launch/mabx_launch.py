from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
import launch
from launch_ros.actions import Node
import launch.events
from ament_index_python.packages import get_package_share_directory
import os.path
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from launch_utils import to_urdf

urdf_file_name = 'electrical_excavator.urdf.xacro'
rviz_file_name = 'machine_visualizer.rviz'

rviz = os.path.join(get_package_share_directory('autonomous_excavator'), 'config', rviz_file_name)

xacro_path = os.path.join(get_package_share_directory('autonomous_excavator'), 'urdf', urdf_file_name)
urdf = to_urdf(xacro_path, {'use_nominal_extrinsics': 'true', 'add_plug': 'true'})

with open(urdf, 'r') as infp:
    robot_desc = infp.read()

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonomous_excavator',
            executable='udp_rx_node.py'),
        Node(
            package='autonomous_excavator',
            executable='udp_tx_node.py'),
        Node(
            package='autonomous_excavator',
            executable='joint_publisher'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz]),
    ])
