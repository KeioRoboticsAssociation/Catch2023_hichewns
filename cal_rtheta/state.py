import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from std_msgs.msg import Int8
import time

class State(Node):
    def __init__(self):
        super().__init__('state')
        # self.joy_subscription = self.create_subscription(Float32MultiArray, 'joy_data', self.joy_callback, 10)
        self.end_subscription = self.create_subscription(String, 'is_ended', self.is_ended_callback, 10)
        self.state_publisher = self.create_publisher(Int8, 'state', 10)
        self.target_publisher = self.create_publisher(Float32MultiArray, 'target_xy', 10)
        self.tmr = self.create_timer(0.1, self.callback)
        self.red_own_target = [[0.395,0.100],[0.395,0.300],[0.395,0.500],[0.395,-0.100],[0.395,-0.300],[0.395,-0.500]]
        self.state = 0
        self.cnt = 0
        self.is_ended = False
    
    def callback(self,msg):
        if self.state == 0:
            self.target_publisher.publish(self.red_own_target[int(self.cnt)])
        

        elif self.state == 1:
            self.state = 2
            
        elif self.state == 2:
            self.state=3
            
        elif self.state == 3:
            self.state = 0
        
        self.cnt +=1
            
    def is_ended_callback(self,msg):
        self.is_ended = True
        if self.is_ended == True:
            self.state += 1
            self.state_publisher.publish(int(self.state))
            self.is_ended = False 
        
def main():
    rclpy.init()
    state = State()
    rclpy.spin(state)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    