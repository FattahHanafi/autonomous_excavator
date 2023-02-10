#!/usr/bin/env python3

import rclpy
from autonomous_excavator.udp import udp_rx


def main(args=None):
    rclpy.init(args=args)
    node = udp_rx()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
