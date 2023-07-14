import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray


class JoySubscriber(Node):
    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'joy_data', 10)
        self.tmr = self.create_timer(0.1, self.callback)
        self.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def joy_callback(self, joy_msg):
        axes = joy_msg.axes
        buttons = joy_msg.buttons

        if len(axes) >= 3:
            self.data[0] = axes[0]  # revolute (top_linkの回転)左軸左右回転
            self.data[1] = axes[1]  # prismatic (center_linkの直動)右軸上下

        if len(buttons) >= 15:
            self.data[2] = buttons[0]  # up (arm2上昇)十字キー上
            self.data[3] = buttons[1]  # down (arm2下降)十字キー下
            self.data[4] = buttons[2]  # hand1 (hand1回転)△
            self.data[5] = buttons[3]  # hand2 (hand2回転)○
            self.data[6] = buttons[4]  # hand3 (hand3回転)×

    def callback(self):
        joy_data = Float32MultiArray()
        joy_data.data = self.data
        self.publisher_.publish(joy_data)


def main():
    rclpy.init()
    joy_subscriber = JoySubscriber()
    rclpy.spin(joy_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


