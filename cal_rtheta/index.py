import os
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Int8


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('index')

    publisher = node.create_publisher(Int8, 'index', 10)

    msg = Int8()

    while rclpy.ok():
        index=int(input('Enter Index: '))
        msg.index = index
        node.get_logger().info('Publishing: "%s"' % msg)
        publisher.publish(msg)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
