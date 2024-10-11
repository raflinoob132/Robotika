#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')  # Nama node adalah 'simple_node'
        self.get_logger().info('Hello ROS 2 from Python!')

def main(args=None):
    rclpy.init(args=args)  # Inisialisasi komunikasi ROS 2
    node = SimpleNode()    # Buat instance dari node
    rclpy.spin(node)       # Jalankan node
    node.destroy_node()    # Hancurkan node saat selesai
    rclpy.shutdown()       # Matikan komunikasi ROS 2

if __name__ == '__main__':
    main()
