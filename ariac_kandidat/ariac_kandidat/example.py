import rclpy
from rclpy.node import Node

class ExampleNode(Node):

    def __init__(self):
        super().__init__('example_node')
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Example node running")