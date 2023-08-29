import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String
from std_msgs.msg import Int8
from catch2023_interfaces.msg import CreateMessage
import time

class State(Node):
    def __init__(self):
        super().__init__('state')
        self.degpos_subscription = self.create_subscription(CreateMessage, 'degpos_data', self.degpos_callback, 10)
        self.joy_subscription = self.create_subscription(Float32MultiArray, 'joy_data', self.joy_callback, 10)
        self.flag_subscription = self.create_subscription(String, 'is_ended', self.is_ended_callback, 10)
        self.state_publisher = self.create_publisher(Int32MultiArray, 'state_data', 10)
        self.stepper_publisher = self.create_publisher(Int8, 'stepper_cmd', 10)
        self.target_publisher = self.create_publisher(Float32MultiArray, 'target_xy', 10)
        self.tmr = self.create_timer(0.1, self.callback)
        self.red_own_target = [[0.395,0.100],[0.395,0.300],[0.395,0.500],[0.395,-0.100],[0.395,-0.300],[0.395,-0.500]]
        #self.red_com_target = [[],[],[],[],]
        self.state = 1
        self.cnt = 0
        self.stepper_cmd = 1
        self.is_ended = False
        self.state_data = [0,0]
        self.catched = False
     
    def is_ended_callback(self,msg):
        if msg.data == 'is_ended':
            self.is_ended = True
    
    def degpos_callback(self,degpos_msg):
        if degpos_msg.judge == True:
            self.catched = True
        
        elif degpos_msg.judge == False:
            self.catched = False
    
    def joy_callback(self,joy_msg):
        self.next = joy_msg.data[9]
        self.back = joy_msg.data[10]

        if self.next == 1:
            self.state += 1
            time.sleep(0.5)
            if self.state > 4:
                self.state = 1
        
        if self.back == 1:
            self.state -= 1
            time.sleep(0.5)
            if self.state < 1:
                self.state = 4
    
    def callback(self):
        if self.state == 1:
            target_xy = Float32MultiArray()
            target_xy.data = self.red_own_target[self.cnt]
            self.target_publisher.publish(target_xy)
            # if self.is_ended == True:

                # self.state = 2
        
        elif self.state == 2:
            # self.is_ended = False
            self.stepper_cmd = 2
            if self.catched == True:
                self.stepper_cmd = 1
            # self.state = 3
            
        elif self.state == 3:
            time.sleep(0.1)
            #  self.state=4
            
        elif self.state == 4:
            time.sleep(2.0)
            # self.stepper_cmd = 2
            # if self.catched == False:
            #     self.stepper_cmd = 1
            self.state = 1
            self.cnt +=1

        
        #state_publish
        self.state_data = [self.state, self.cnt]
        state_data = Int32MultiArray()
        state_data.data = self.state_data
        self.state_publisher.publish(state_data)

        #stepper_publish
        stepper_data = Int8()
        stepper_data.data = self.stepper_cmd
        self.stepper_publisher.publish(stepper_data)
        
        
def main():
    rclpy.init()
    state = State()
    rclpy.spin(state)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    