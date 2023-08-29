import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from catch2023_interfaces.msg import CreateMessage
import math
import numpy as np

MAX_R = 0.48
ERROR = 0.00001

class XY_to_Rtheta(Node):
    def __init__(self):
        super().__init__('xy_to_rtheta')
        self.degpos_publisher = self.create_publisher(CreateMessage, 'degpos_data', 10)
        self.pos_publisher = self.create_publisher(Float32MultiArray, 'pos_data', 10)
        self.flag_publisher = self.create_publisher(String, 'is_ended', 10)
        self.joy_subscription = self.create_subscription(Float32MultiArray, 'joy_data', self.joy_callback, 10)
        # self.subscription = self.create_subscription(Point, 'input', self.input_callback, 10)
        self.stepper_subscription = self.create_subscription(Int8, 'stepper_cmd', self.stepper_callback,10)
        self.target_subscription = self.create_subscription(Float32MultiArray, 'target_xy',self.target_callback,  10)
        self.cur_subscription = self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
        
        self.tmr = self.create_timer(0.1, self.callback)
        self.status = 0
        self.theta = 0.0
        self.r = 0.0
        # arm2
        self.up = 0.0
        self.mid = 0.0
        self.down = 0.0
        self.revarm3 = 0.0
        self.rev = 0.0 #hand1-3
        self.catch = 0.0
        self.release = 0.0
        self.init = 0.0
        self.currentPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.degPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.x=0.0
        self.y=0.0
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.stepper_pos = 0
    
    def joy_callback(self,joy_msg):
        self.catch = joy_msg.data[7]
        self.release = joy_msg.data[8]
    
    def target_callback(self, target_msg):
        self.target_x = target_msg.data[0]
        self.target_y = target_msg.data[1]

        self.currentPos[0]=math.atan2(self.target_y,self.target_x)
        self.currentPos[1]=math.sqrt(self.target_x**2+self.target_y**2)
        if self.currentPos[1] >= MAX_R:
            self.currentPos[1] = MAX_R
        elif self.currentPos[1] <= 0.0:
            self.currentPos[1] = 0.0

        if self.stepper_pos == 0:
            self.currentPos[2] = 0.08
        
        if self.stepper_pos == 1:
            self.currentPos[2] = 0.04
        
        if self.stepper_pos == 2:
            self.currentPos[2] = -0.08

        self.currentPos[3]=0.0
        self.currentPos[4]=0.0
        self.currentPos[5]=0.0
        self.currentPos[6]=0.0

        self.degPos[0]=math.degrees(math.atan2(self.target_y,self.target_x))
        self.degPos[1]=math.sqrt(self.target_x**2+self.target_y**2)
        if self.degPos[1] >= MAX_R:
            self.degPos[1] = MAX_R
        elif self.degPos[1] <= 0.0:
            self.degPos[1] = 0.0
        self.degPos[2]= self.stepper_pos
        self.degPos[3]=0.0
        self.degPos[4]=0.0

        if self.catch == 1.0:
            self.degPos[5] = True
        
        if self.release == 1.0:
            self.degPos[5] = False

        self.dist = math.sqrt((self.target_x - self.cur_x)**2 + (self.target_y - self.cur_y)**2)

        if self.dist <= ERROR:
            msg = String()
            msg.data = 'is_ended'
            self.flag_publisher.publish(msg)
    
    def stepper_callback(self,msg):
        self.stepper_pos = msg.data
    
        
    def joint_states_callback(self,joint_msg):
        self.theta = joint_msg.position[0]
        self.r = joint_msg.position[1]
        self.cur_x = self.r * math.cos(self.theta)
        self.cur_y = self.r * math.sin(self.theta)
    
    def callback(self):
        pos_data = Float32MultiArray()
        pos_data.data = self.currentPos
        self.pos_publisher.publish(pos_data)

        degpos_data = CreateMessage()
        degpos_data.theta = self.degPos[0]
        degpos_data.r = self.degPos[1]
        degpos_data.r *= 1000
        degpos_data.stepper = int(self.degPos[2])
        degpos_data.armtheta = self.degPos[3]
        degpos_data.hand= self.degPos[4]
        degpos_data.judge = bool(self.degPos[5])
        self.degpos_publisher.publish(degpos_data)

def main():
    rclpy.init()
    xy_to_rtheta = XY_to_Rtheta()
    rclpy.spin(xy_to_rtheta)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


## inputで(x,y)入力できるやつ

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# import math
# import numpy as np
# from geometry_msgs.msg import Point
# from std_msgs.msg import String
# from std_msgs.msg import Int8
# from catch2023_interfaces.msg import CreateMessage

# MAX_R = 0.48

# class XY_to_Rtheta(Node):
#     def __init__(self):
#         super().__init__('xy_to_rtheta')
#         self.degpos_publisher = self.create_publisher(CreateMessage, 'degpos_data', 10)
#         self.pos_publisher = self.create_publisher(Float32MultiArray, 'pos_data', 10)
#         self.subscription = self.create_subscription(Point, 'input', self.input_callback, 10)
#         self.flag_subscription = self.create_subscription(String, )
#         self.stepper_subscription = self.create_subscription(Int8, 'stepper_cmd', self.stepper_callback,10)
#         self.target_subscription = self.create_subscription(Float32MultiArray, 'target_xy',self.target_callback,  10)
#         # self.cur_subscription = self.create_subscription(Float32MultiArray, 'current_pos', self.current_pos_callback, 10)
        
#         self.tmr = self.create_timer(0.1, self.callback)
#         self.status = 0
#         self.theta = 0.0
#         self.r = 0.0
#         # arm2
#         self.up = 0.0
#         self.mid = 0.0
#         self.down = 0.0
#         self.revarm3 = 0.0
#         self.rev = 0.0 #hand1-3
#         self.catch = 0.0
#         self.release = 0.0
#         self.init = 0.0
#         self.currentPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#         self.degPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#         self.x=0.0
#         self.y=0.0

#     def input_callback(self,input_msg):
#         self.x=input_msg.x
#         self.y=input_msg.y

#         self.currentPos[0]=math.atan2(self.y,self.x)
#         self.currentPos[1]=math.sqrt(self.x**2+self.y**2)
#         if self.currentPos[1] >= MAX_R:
#             self.currentPos[1] = MAX_R
#         elif self.currentPos[1] <= 0.0:
#             self.currentPos[1] = 0.0
#         self.currentPos[2]=0.0
#         self.currentPos[3]=0.0
#         self.currentPos[4]=0.0
#         self.currentPos[5]=0.0
#         self.currentPos[6]=0.0


#         self.degPos[0]=math.degrees(math.atan2(self.y,self.x))
#         self.degPos[1]=math.sqrt(self.x**2+self.y**2)
#         if self.degPos[1] >= MAX_R:
#             self.degPos[1] = MAX_R
#         elif self.degPos[1] <= 0.0:
#             self.degPos[1] = 0.0
#         self.degPos[2]=0.0
#         self.degPos[3]=0.0
#         self.degPos[4]=0.0
#         self.degPos[5]=0.0
    
#     def stepper_callback(self,msg):
#         self.stepper_pos = msg

#     def callback(self):
#         pos_data = Float32MultiArray()
#         pos_data.data = self.currentPos
#         self.pos_publisher.publish(pos_data)

#         degpos_data = CreateMessage()
#         degpos_data.theta = self.degPos[0]
#         degpos_data.r = self.degPos[1]
#         degpos_data.r *= 1000
#         degpos_data.stepper = int(self.degPos[2])
#         degpos_data.armtheta = self.degPos[3]
#         degpos_data.hand= self.degPos[4]
#         degpos_data.judge = bool(self.degPos[5])
#         self.degpos_publisher.publish(degpos_data)

# def main():
#     rclpy.init()
#     xy_to_rtheta = XY_to_Rtheta()
#     rclpy.spin(xy_to_rtheta)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()