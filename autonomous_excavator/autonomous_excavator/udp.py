#!/usr/bin/env python3

import rclpy
import socket
import struct
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

CLIENT_ADDR = ('192.168.1.12', 5000)
SERVER_ADDR = ('192.168.1.9', 5000)


class udp_rx(Node):
    def __init__(self):
        super().__init__("udp_rx_node")
        self.joint_publishers = self.create_publisher(JointState, 'Machine/ActuatorStroke/Feedback', 10)
        joint_message = JointState()
        joint_message.header.frame_id = "world"
        for i in range(4):
            joint_message.name.append("S" + str(i))
            joint_message.position.append(0)
            joint_message.velocity.append(0)
            joint_message.effort.append(0)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        sock.bind(('192.168.1.12', 5000))
        while (rclpy.ok()):
            data, _ = sock.recvfrom(100)
            data = struct.unpack('d' * (len(data) // 8), data)
            joint_message.header.stamp = self.get_clock().now().to_msg()
            joint_message.position[0] = -data[0]
            joint_message.position[1] = data[1]
            joint_message.position[2] = data[2]
            joint_message.position[3] = data[3]
            self.joint_publishers.publish(joint_message)


class udp_tx(Node):
    def __init__(self):
        super().__init__("udp_tx_node")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('192.168.1.12', 5001))
        self.joint_subscriber = self.create_subscription(JointState, 'Machine/ActuatorStroke/Command', self.joint_message_callback, 10)

    def joint_message_callback(self, msg):
        data = struct.pack('dddd', msg.position[0], msg.position[1], msg.position[2], msg.position[3])
        self.sock.sendto(data, ('192.168.1.9', 5001))
