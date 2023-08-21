import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

class State(Node):
    def __init__(self):
        super().__init__('state')
        # self.joy_subscription = self.create_subscription(Float32MultiArray, 'joy_data', self.joy_callback, 10)
        self.state_publisher = self.create_publisher(Bool, 'state', 10)
        self.target_publisher = self.create_publisher(Float32MultiArray, 'target_xy', 10)
        self.to_xy_publisher = self.create_publisher(String, 'flag', 10)
        self.tmr = self.create_timer(0.1, self.callback)
        self.r_own_target_list = [[0.395,0.100],[0.395,0.300],[0.395,0.500],[0.395,-0.100],[0.395,-0.300],[0.395,-0.500]]
        # self.r_common_target_list = [[]]

        self.state = 0
        self.cnt = 0
    
    
    def callback(self,msg):
        if self.state == 0:
            self.target_pubself.publish(self.r_own_target_list[int(self.cnt)])
            msg=String()
            msg.data= "Start"
            self.to_xy_publisher(msg)

        elif self.state == 1:
            self.state = 3
            
        elif self.state == 2:
            self.state=4
            
        elif self.state == 3:
            self.state = 0
        
        self.cnt +=1
            
        
        
def main():
    rclpy.init()
    state = State()
    rclpy.spin(state)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    # def select_state(self):
    