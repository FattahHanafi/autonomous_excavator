#!/usr/bin/env python3

import time
import math
import rclpy
import socket
import struct
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, JointState
from autonomous_excavator_interfaces.msg import Float64Array

CLIENT_ADDR = ('192.168.1.12', 5000)
SERVER_ADDR = ('192.168.1.9', 5000)

NUMBER_OF_PRESSURE_SENSORS = 6
NUMBER_OF_DI = 24
NUMBER_OF_DO = 24
NUMBER_OF_AI = 16
NUMBER_OF_AO = 8

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
        # sock.bind(('192.168.1.12', 5000))
        while (rclpy.ok()):
            # data, _ = sock.recvfrom(100)
            # data = struct.unpack('d' * (len(data) // 8), data)
            joint_message.header.stamp = self.get_clock().now().to_msg()
            # joint_message.position[0] = data[0]
            # joint_message.position[1] = data[1]
            # joint_message.position[2] = data[2]
            # joint_message.position[3] = data[3]

            joint_message.position[0] = 0.0
            joint_message.position[1] = 0.0
            joint_message.position[2] = 0.0
            joint_message.position[3] = 0.0
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

class udp_state(Node):
    def __init__(self):
        super().__init__("udp_state")

        # publishers
        self.battery_publisher = self.create_publisher(BatteryState, 'Machine/Status/Battery', 10)
        self.pressure_publisher = self.create_publisher(Float64Array, 'Machine/Status/Pressure', 10)
        self.di_publisher = self.create_publisher(Float64Array, 'Machine/IO/DI', 10)
        self.do_publisher = self.create_publisher(Float64Array, 'Machine/IO/DO', 10)
        self.ai_publisher = self.create_publisher(Float64Array, 'Machine/IO/AI', 10)
        self.ao_publisher = self.create_publisher(Float64Array, 'Machine/IO/AO', 10)
        
        # messages
        battery_message = BatteryState()
        battery_message.voltage = 0.0
        
        pressure_message = Float64Array()
        pressure_message.name = [""] * NUMBER_OF_PRESSURE_SENSORS
        pressure_message.value = [0.0] * NUMBER_OF_PRESSURE_SENSORS

        di_message = Float64Array()
        di_message.name = [""] * NUMBER_OF_DI
        di_message.value = [0.0] * NUMBER_OF_DI

        do_message = Float64Array()
        do_message.name = [""] * NUMBER_OF_DO
        do_message.value = [0.0] * NUMBER_OF_DO

        ai_message = Float64Array()
        ai_message.name = [""] * NUMBER_OF_AI
        ai_message.value = [0.0] * NUMBER_OF_AI
        
        ao_message = Float64Array()
        ao_message.name = [""] * NUMBER_OF_AO
        ao_message.value = [0.0] * NUMBER_OF_AO

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('192.168.1.12', 5002))
        while (rclpy.ok()):
            data, _ = sock.recvfrom(1000)
            data = struct.unpack('d' * (len(data) // (1 + NUMBER_OF_PRESSURE_SENSORS + NUMBER_OF_DI + NUMBER_OF_DO + NUMBER_OF_AI + NUMBER_OF_AO)), data)
            now = self.get_clock().now().to_msg()

            battery_message.header.stamp = now
            pressure_message.header.stamp = now
            di_message.header.stamp = now
            do_message.header.stamp = now
            ai_message.header.stamp = now
            ao_message.header.stamp = now

            battery_message.voltage = data[0]
            Counter = 1
            pressure_message.value = list(data[Counter : Counter + NUMBER_OF_PRESSURE_SENSORS])
            Counter += NUMBER_OF_PRESSURE_SENSORS
            di_message.value = list(data[Counter : Counter + NUMBER_OF_DI])
            Counter += NUMBER_OF_DI
            do_message.value = list(data[Counter : Counter + NUMBER_OF_DO])
            Counter += NUMBER_OF_DO
            di_message.value = list(data[Counter : Counter + NUMBER_OF_AI])
            Counter += NUMBER_OF_AI
            di_message.value = list(data[Counter : Counter + NUMBER_OF_AO])

            self.battery_publisher.publish(battery_message)
            self.pressure_publisher.publish(pressure_message)
            self.di_publisher.publish(di_message)
            self.do_publisher.publish(do_message)
            self.ai_publisher.publish(ai_message)
            self.ao_publisher.publish(ao_message)
