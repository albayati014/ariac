#!/usr/bin/env python3

import rclpy

from ariac_package.example import ExampleNode

def main(args=None):
    rclpy.init(args=args)

    example_node = ExampleNode()

    rclpy.spin(example_node)

    example_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()