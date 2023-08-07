import os
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class Input(Node):
    def __init__(self):
        super().__init__('input')
        self.publisher_ = self.create_publisher(Point, 'input', 1)
        self.timer_period = 5.0  # メッセージを送信する間隔（秒）
        self.timer = self.create_timer(self.timer_period, self.publish_coordinates)

    def publish_coordinates(self):
        # ここで座標を読み込むか、直接座標を生成するか決定します
        # この例では、座標を直接生成します
        coordinates = [
            (0.4, 0.0),
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
