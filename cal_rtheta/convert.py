import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import array
import numpy as np


class Convert(Node):
    def __init__(self):
        super().__init__('convert')
        self.current_pos_publisher_ = self.create_publisher(Float32MultiArray, 'current_pos', 10)
        self.subscription = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

    def joint_state_callback(self, msg):
        current_pos = Float32MultiArray()
        current_pos.data = array.array('f',msg.position) #list to array
        self.current_pos_publisher_.publish(current_pos)

    
def main():
    rclpy.init()
    convert = Convert()
    rclpy.spin(convert)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

