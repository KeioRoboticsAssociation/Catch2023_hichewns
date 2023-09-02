import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from catch2023_interfaces.msg import CreateMessage
import math

MAX_R = 0.48

class XY_to_Rtheta(Node):
    def __init__(self):
        #publisher
        super().__init__('xy_to_rtheta')
        self.degpos_publisher = self.create_publisher(CreateMessage, 'degpos_data', 10)
        self.pos_publisher = self.create_publisher(Float32MultiArray, 'pos_data', 10)
        self.flag_publisher = self.create_publisher(String, 'is_ended', 10)
        #subscriber
        self.target_error_publisher = self.create_publisher(Float32MultiArray, 'target_error', 10)
        self.joy_subscription = self.create_subscription(Float32MultiArray, 'joy_data', self.joy_callback, 10)
        self.stepper_subscription = self.create_subscription(Int8, 'stepper_cmd', self.stepper_callback,10)
        self.servo_subscription = self.create_subscription(Int8, 'servo_cmd', self.servo_callback,10)
        self.target_subscription = self.create_subscription(Float32MultiArray, 'target_xy',self.target_callback,  10)
        self.shootingbox_subscripition = self.create_subscription(Float32MultiArray, 'shootingbox_xy', self.shooting_callback,10)
        self.cur_subscription = self.create_subscription(CreateMessage, 'real_pos', self.real_pos_callback, 10)
        self.move_subscription = self.create_subscription(Bool, 'move_cmd', self.move_callback, 10)
        self.init_subscription = self.create_subscription(Bool, 'init', self.init_callback, 10)
        self.joint_subscription = self.create_subscription(JointState, 'joint_states', self.joint_states_callback,10)
        
        self.tmr = self.create_timer(0.1, self.callback)
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
        self.shooting_x = 0.0
        self.shooting_y = 0.0
        self.stepper_pos = 0
        self.servo_cmd = 1
        self.real_r = 0.0
        self.real_theta = 0.0
        self.shooting_error = 0.0
        self.target_error = [0.0]
        self.ERROR_theta = 45.0
    
    def joint_states_callback(self,joint_states):
        self.real_theta = joint_states.position[0]
    
    def init_callback(self,init_msg):
        self.init = init_msg.data
        if self.init == True:
            self.currentPos = [math.pi/2, 0.0, 0.08, 0.0, 0.0, 0.0, 0.0]
            self.degPos = [90.0, 0.0, 0.0, 0.0, 0.0, True]
    
    def real_pos_callback(self,real_pos_msg):
        self.real_theta = real_pos_msg.theta
        self.real_r = real_pos_msg.r/1000
    
    def joy_callback(self,joy_msg):
        self.catch = joy_msg.data[7]
        self.release = joy_msg.data[8]
        if self.catch == 1.0:
            self.degPos[5] = True
        
        if self.release == 1.0:
            self.degPos[5] = False
    
    def servo_callback(self,servo_msg):
        self.servo_cmd = servo_msg.data
        if self.servo_cmd == 1:
            self.currentPos[4]=-math.pi/4
            self.currentPos[5]=-math.pi/4
            self.currentPos[6]=-math.pi/4
            self.degPos[4] = -45.0

            self.currentPos[3]=((math.pi / 2) - self.currentPos[0]) + math.pi/4
            self.degPos[3] = (90 - self.degPos[0]) + 45
        
        elif self.servo_cmd == 0:
            self.currentPos[4]=0.0
            self.currentPos[5]=0.0
            self.currentPos[6]=0.0
            self.degPos[4] = 0.0

            self.currentPos[3]= -self.currentPos[0]
            self.degPos[3] = -self.degPos[0]

            
    def move_callback(self,move_msg):
        self.move_cmd = move_msg.data
        if self.move_cmd == True:
            if self.degPos[1] > 0.10:
                self.currentPos[1] = 0.10
                if self.currentPos[1] >= MAX_R:
                    self.currentPos[1] = MAX_R
                elif self.currentPos[1] <= 0.0:
                    self.currentPos[1] = 0.0

                self.degPos[1] = 0.10
                if self.degPos[1] >= MAX_R:
                    self.degPos[1] = MAX_R
                elif self.degPos[1] <= 0.0:
                    self.degPos[1] = 0.0
            
    def target_callback(self, target_msg):
        self.target_x = target_msg.data[0]
        self.target_y = target_msg.data[1]

        self.currentPos[0]=math.atan2(self.target_y,self.target_x)
        self.degPos[0]=math.degrees(math.atan2(self.target_y,self.target_x))

        self.target_error[0] = float(self.degPos[0] - self.real_theta)   

        if self.target_error[0] <= self.ERROR_theta:
            self.currentPos[1]=math.sqrt(self.target_x**2+self.target_y**2) - 0.407
            if self.currentPos[1] >= MAX_R:
                self.currentPos[1] = MAX_R
            elif self.currentPos[1] <= 0.0:
                self.currentPos[1] = 0.0

            self.degPos[1]=math.sqrt(self.target_x**2+self.target_y**2) - 0.407
            if self.degPos[1] >= MAX_R:
                self.degPos[1] = MAX_R
            elif self.degPos[1] <= 0.0:
                self.degPos[1] = 0.0
        
        target_error = Float32MultiArray()
        target_error.data = self.target_error
        self.target_error_publisher.publish(target_error)

    
    def shooting_callback(self, shooting_msg):
        self.shooting_x = shooting_msg.data[0]
        self.shooting_y = shooting_msg.data[1]

        self.currentPos[0] = math.atan2(self.shooting_y,self.shooting_x)

        self.currentPos[3]=-(self.currentPos[0] - math.pi/2)
        self.currentPos[4]=0.0
        self.currentPos[5]=0.0
        self.currentPos[6]=0.0

        self.degPos[0] = math.degrees(math.atan2(self.shooting_y,self.shooting_x))
        self.degPos[3]=-(self.degPos[0] - 90.0)
        self.degPos[4]=0.0

        self.shooting_error = float(self.degPos[0] - self.real_theta)
        if self.shooting_error <= self.ERROR_theta:
            self.currentPos[1] = math.sqrt(self.shooting_x**2 +self.shooting_y**2) - 0.407
            if self.currentPos[1] >= MAX_R:
                self.currentPos[1] = MAX_R
            elif self.currentPos[1] <= 0.0:
                self.currentPos[1] = 0.0

            self.degPos[1] = math.sqrt(self.shooting_x**2+self.shooting_y**2) - 0.407
            if self.degPos[1] >= MAX_R:
                self.degPos[1] = MAX_R
            elif self.degPos[1] <= 0.0:
                self.degPos[1] = 0.0
    
    def stepper_callback(self,msg):
        self.stepper_pos = msg.data
        self.degPos[2]= self.stepper_pos

        if self.stepper_pos == 0:
            self.currentPos[2] = 0.08
        
        if self.stepper_pos == 1:
            self.currentPos[2] = 0.04
        
        if self.stepper_pos == 2:
            self.currentPos[2] = -0.08
    
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