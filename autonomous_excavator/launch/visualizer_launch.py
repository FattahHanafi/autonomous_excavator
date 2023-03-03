from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os.path

import os
import xacro
import tempfile

def to_urdf(xacro_path, parameters=None):
    """Convert the given xacro file to URDF file.
    * xacro_path -- the path to the xacro file
    * parameters -- to be used when xacro file is parsed.
    """
    urdf_path = tempfile.mktemp(prefix="%s_" % os.path.basename(xacro_path))

    # open and process file
    doc = xacro.process_file(xacro_path, mappings=parameters)
    # open the output file
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    return urdf_path

urdf_file_name = 'electrical_excavator.urdf.xacro'
rviz_file_name = 'machine_visualizer.rviz'

xacro_path = os.path.join(get_package_share_directory('autonomous_excavator'), 'urdf', 'electrical_excavator.urdf.xacro')
urdf = to_urdf(xacro_path, {'use_nominal_extrinsics': 'true', 'add_plug': 'true'})
rviz = os.path.join(get_package_share_directory('autonomous_excavator'), 'config', rviz_file_name)

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
            package='autonomous_excavator',
            executable='excavator_tf2_broadcaster'),
        # Node(
            # package='autonomous_excavator',
            # executable='blade_node'),
        # Node(
            # package='autonomous_excavator',
            # executable='merged'),
        # Node(
            # package='autonomous_excavator',
            # executable='surface_reconstructor'),
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
