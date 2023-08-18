import os
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class Input(Node):
    def __init__(self):
        super().__init__('input')
        self.publisher_ = self.create_publisher(Point, 'input', 1)
        self.timer_period = 5.0
        self.timer = self.create_timer(self.timer_period, self.publish_coordinates)

    def publish_coordinates(self):
        coordinates = [
            (0.0, 0.0),
        ]

        for coord in coordinates:
            msg = Point()
            msg.x, msg.y= coord
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published coordinate: {msg.x}, {msg.y}")

def main(args=None):
    rclpy.init(args=args)
    node = Input()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
