import os
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from std_msgs.msg import String


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('index')

    state_publisher = node.create_publisher(String,'cmd_state',10)

    state_msg = String()

    while rclpy.ok():

        state_cmd = input('Enter Cmd_State: ')
        state_msg.data = state_cmd
        node.get_logger().info('Publishing: "%s"' % state_cmd)
        state_publisher.publish(state_msg)


if __name__ == '__main__':
    main() 