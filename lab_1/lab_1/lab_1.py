#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String

class Lab01Node(Node):
    def __init__(self):
        super().__init__('lab01')
        
        # TODO: publish commands to make robot move to desired pose
        
        self.get_logger().info('Lab01 node has been initialized.')


def main():
    rclpy.init()
    node = Lab01Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

if __name__ == '__main__':
    main()
