import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

class State(Node):
    def __init__(self):
        super().__init__('state')
        self.state_publisher = self.create_publisher(Bool, 'state', 10)
        self.r_own_target_list = [[0.395,0.100],[0.395,0.300],[0.395,0.500],[0.395,-0.100],[0.395,-0.300],[0.395,-0.500]]
        # self.r_common_target_list = [[]]
        self.state = 0
        self.cnt = 0
    
    # def select_state(self):
        
