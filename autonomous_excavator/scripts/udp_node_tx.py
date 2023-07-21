#!/usr/bin/env python3

import rclpy
from autonomous_excavator.udp import udp_tx


def main(args=None):
    rclpy.init(args=args)
    node = udp_tx()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
