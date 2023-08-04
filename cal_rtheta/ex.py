import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math

class Cal(Node):
    def __init__(self):
        super().__init__('cal')
        self.pos_publisher = self.create_publisher(Float32MultiArray, 'pos_data', 10)
        self.tmr = self.create_timer(0.1, self.callback)
        self.currentPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

      

    def callback(self):
        pos_data = Float32MultiArray()
        pos_data.data = self.currentPos
        self.pos_publisher.publish(pos_data)


def main():
    rclpy.init()
    cal = Cal()
    rclpy.spin(cal)
    rclpy.shutdown()


if __name__ == '__main__':
    main()