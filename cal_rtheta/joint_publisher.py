import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import math


class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.subscription = self.create_subscription(Float32MultiArray, 'pos_data', self.pos_callback, 10)
        self.js0 = JointState()
        self.pos = [0.0, 0.0, 0.0, 0.0,0.0,0.0,0.0]
        self.js0.name = ['body_to_revolute', 'revolute_to_prismatic', 'prismatic_to_arm2', 'arm2_to_arm3','arm3_to_hand1','arm3_to_hand2','arm3_to_hand3']
        self.tmr = self.create_timer(0.05, self.callback)

    def pos_callback(self, pos_msg):
        self.pos[0] = pos_msg.data[0]
        self.pos[1] = pos_msg.data[1]
        self.pos[2] = pos_msg.data[2]
        self.pos[3] = pos_msg.data[3]
        self.pos[4] = pos_msg.data[4]
        self.pos[5] = pos_msg.data[5]
        self.pos[6] = pos_msg.data[6]

    def callback(self):
        self.js0.header.stamp = self.get_clock().now().to_msg()
        self.js0.position = self.pos
        self.publisher_.publish(self.js0)


def main():
    rclpy.init()
    joint_publisher = JointPublisher()
    rclpy.spin(joint_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

