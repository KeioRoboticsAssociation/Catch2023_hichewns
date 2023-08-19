import os
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

# class Input(Node):
#     def __init__(self):
#         super().__init__('input')
#         self.publisher_ = self.create_publisher(Point, 'input', 1)
#         self.timer_period = 5.0
#         self.timer = self.create_timer(self.timer_period, self.publish_coordinates)

#     def publish_coordinates(self):
#         coordinates = [
#             (0.0, 0.5),
#         ]

#         for coord in coordinates:
#             msg = Point()
#             msg.x, msg.y= coord
#             self.publisher_.publish(msg)
#             self.get_logger().info(f"Published coordinate: {msg.x}, {msg.y}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = Input()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('input')

    publisher = node.create_publisher(Point, 'input', 10)

    msg = Point()

    while rclpy.ok():
        x=float(input('Enter X: '))
        y=float(input('Enter Y: '))
        msg.x = x
        msg.y = y
        node.get_logger().info('Publishing: "%s"' % msg)
        publisher.publish(msg)
        # sleep.tim(0.5)  # seconds

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
