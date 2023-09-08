import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String
from std_msgs.msg import Int8
from catch2023_interfaces.msg import CreateMessage
import time
import csv
import math

class State(Node):
    def __init__(self):
        super().__init__('state')
        self.real_pos_subscription = self.create_subscription(CreateMessage, 'real_pos', self.real_pos_callback, 10)
        self.cmd_state_subscription = self.create_subscription(String,'cmd_state',self.cmd_state_callback,10)
        self.pose_subscription = self.create_subscription(Int8, 'index', self.index_callback, 10)
        self.shooting_index_subscription = self.create_subscription(Int8, 'shooting_index', self.shooting_index_callback, 10)
        self.target_pose_subscription = self.create_subscription(Float32MultiArray, 'target_pose', self.target_pose_callback,10)
        self.shooting_pose_subscription = self.create_subscription(Float32MultiArray, 'shooting_pose', self.shooting_pose_callback,10)
        self.target_comp_subscription = self.create_subscription(Bool, 'target_comp', self.target_comp_callback, 10)
        self.targetcomp_publisher = self.create_publisher(Bool, 'target_comp', 10)
        self.shooting_comp_subscription = self.create_subscription(Bool, 'shooting_comp', self.shooting_comp_callback, 10)
        self.shooting_comp_publisher = self.create_publisher(Bool, 'shooting_comp', 10)
        self.state_publisher = self.create_publisher(Int32MultiArray, 'state_data', 10)
        self.stepper_publisher = self.create_publisher(Int8, 'stepper_cmd', 10)
        self.servo_publisher = self.create_publisher(Int8, 'servo_cmd', 10)
        self.target_publisher = self.create_publisher(Float32MultiArray, 'target_xy', 10)
        self.shootingbox_publisher = self.create_publisher(Float32MultiArray, 'shootingbox_xy', 10)
        self.init_publisher = self.create_publisher(Bool, 'init', 10)
        self.move_publisher = self.create_publisher(Bool, 'move_cmd', 10)
        self.release_publisher = self.create_publisher(Bool, 'release_cmd', 10)
        # self.is_manual_publisher = self.create_publisher(Bool, 'is_manual', 10)
        self.is_manual_subscription = self.create_subscription(Bool, 'is_manual', self.is_manual_callback,10)
        # self.joy_subscription = self.create_subscription(Float32MultiArray, 'joy_data', self.joy_callback, 10)
        
        self.tmr = self.create_timer(0.1, self.callback)

        self.red_own_target = []
        self.red_shooting_box = []
        self.theta = 0.0
        self.state = 0
        self.cnt = 0
        self.box = 0
        self.init = False
        self.stepper_cmd = 0
        self.servo_cmd = 1
        self.move_cmd = False
        self.is_ended = False
        self.target_cmd = False
        self.shooting_cmd = False
        self.state_data = [0,0,0]
        self.target_comp = False
        self.shooting_comp = False
        self.catched = False
        self.index = 0
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.state_cmd = False
        self.next = False
        self.back = False
        self.release_cmd = False
        self.catch_cmd = False
        self.is_manual = False
        self.manual_flag = True

        with open('/home/moyuboo/ros2_ws/src/catch2023/cal_rtheta/csv/pose_red.csv', 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                self.red_own_target.append([float(row[0]),float(row[1])])

        with open('/home/moyuboo/ros2_ws/src/catch2023/cal_rtheta/csv/shooting_red.csv', 'r') as f:
            reader1 = csv.reader(f)
            for row in reader1:
                self.red_shooting_box.append([float(row[0]),float(row[1])])


    def is_manual_callback(self,manualmsg):
        self.is_manual = manualmsg.data

    def index_callback(self,index_msg):
        self.index = index_msg.data
        self.move_cmd = False
        self.state = 1
    
    def target_comp_callback(self, target_comp_msg):
        self.target_comp = target_comp_msg.data
        if self.target_comp == True:
            self.target_cmd = False
            # self.state = 2
    
    def real_pos_callback(self, real_pos_msg):
        self.stepper = real_pos_msg.stepper
    
    def shooting_comp_callback(self, shooting_comp_msg):
        self.shooting_comp = shooting_comp_msg.data
        if self.shooting_comp == True:
            self.shooting_cmd = False

            self.shooting_comp = False
            shootingcomp = Bool()
            shootingcomp.data = self.shooting_comp
            self.shooting_comp_publisher.publish(shootingcomp)

            self.state = 6

    def shooting_index_callback(self,shooting_index_msg):
        self.box = shooting_index_msg.data
        self.move_cmd = False
        self.state = 5
    
    def target_pose_callback(self,targetpos):
        self.red_own_target[self.index] = targetpos.data
        with open('/home/moyuboo/ros2_ws/src/catch2023/cal_rtheta/csv/re_pose.csv','w') as f:
            writer = csv.writer(f)
            writer.writerows(self.red_own_target)
    
    def shooting_pose_callback(self,shootingpos):
        self.red_shooting_box[self.box] = shootingpos.data
        with open('/home/moyuboo/ros2_ws/src/catch2023/cal_rtheta/csv/re_shooting.csv','w') as f:
            writer = csv.writer(f)
            writer.writerows(self.red_shooting_box)
    
    def cmd_state_callback(self, cmd_state):
        if cmd_state.data == 'n':
            self.next = True

        if self.next == True:
            self.state += 1
            time.sleep(0.5)
            if self.state > 7:
                self.state = 1
            self.next = False
    
        elif cmd_state.data == 'b':
            self.back = True
        
        if self.back == True:
            self.state -= 1
            time.sleep(0.5)
            if self.state < 1:
                self.state = 7
            self.back = False
        
        elif cmd_state.data == 'i':
            self.init = True

        if self.init == True:
            self.shooting_cmd = False
            self.target_cmd = False
            self.move_cmd = False
            self.state = 0
            self.init = False

    def callback(self):
        if self.state == 0:
            self.stepper_cmd = 0
            init_msg = Bool()
            init_msg.data = True
            self.init_publisher.publish(init_msg)

        elif self.state == 1:
            self.target_cmd = True
            self.release_cmd = True
            release_cmd = Bool()
            release_cmd.data = self.release_cmd
            self.release_publisher.publish(release_cmd)
            self.move_cmd = False
            self.target_cmd = True
            servo_cmd = Int8()
            if self.index < 6:
                self.servo_cmd = 1

            elif self.index >= 6:
                self.servo_cmd = 0
            servo_cmd.data = self.servo_cmd
            self.servo_publisher.publish(servo_cmd) 

            if self.target_comp == True:
                self.stepper_cmd = 1
                if self.stepper == 1:
                    if not self.is_manual:
                        self.target_cmd = False
                        self.target_comp = False
                        targetcomp = Bool()
                        targetcomp.data = self.target_comp
                        self.targetcomp_publisher.publish(targetcomp)
                        self.state = 2
        
        elif self.state == 2:
            self.move_cmd = False
        #ここコメントにした
            if self.index > 5:
                self.stepper_cmd = 2
                if self.stepper == 2:
                    self.release_cmd = False
                    release_cmd = Bool()
                    release_cmd.data = self.release_cmd
                    self.release_publisher.publish(release_cmd)
                    time.sleep(0.5)
                    self.state = 3

            elif self.index <= 5:
                self.stepper_cmd = 3
                if self.stepper == 3:
                    self.release_cmd = False
                    release_cmd = Bool()
                    release_cmd.data = self.release_cmd
                    self.release_publisher.publish(release_cmd)
                    time.sleep(0.2)
                    self.state = 3

        elif self.state == 3:
            # self.target_cmd = False
            self.move_cmd = False
            if self.index == 0:
                self.stepper_cmd = 1
                if self.stepper == 1:
                    self.box = 6
                    self.state = 4
            self.stepper_cmd = 0
            if self.stepper == 0:
                self.state = 4

        elif self.state == 4:
            self.move_cmd = True
            
        elif self.state == 5:
            self.move_cmd = False
            self.shooting_cmd = True

        elif self.state == 6:
            # self.shooting_cmd = False
            self.release_cmd = True
            release_cmd = Bool()
            release_cmd.data = self.release_cmd
            self.release_publisher.publish(release_cmd)
            time.sleep(0.5)
            self.state = 7

        elif self.state == 7:
            self.shooting_cmd = False
            self.move_cmd = True
           
        #state_publish
        self.state_data = [self.state, self.cnt, self.box]
        state_data = Int32MultiArray()
        state_data.data = self.state_data
        self.state_publisher.publish(state_data)

        #stepper_publish
        stepper_data = Int8()
        stepper_data.data = self.stepper_cmd
        self.stepper_publisher.publish(stepper_data)

        if self.move_cmd == True:
            move_cmd = Bool()
            move_cmd.data = self.move_cmd
            self.move_publisher.publish(move_cmd)

        if self.target_cmd == True:
            target_xy = Float32MultiArray()
            target_xy.data = self.red_own_target[self.index]
            self.target_publisher.publish(target_xy)

        if self.shooting_cmd == True:
            shooting_xy = Float32MultiArray()
            shooting_xy.data = self.red_shooting_box[self.box]
            self.shootingbox_publisher.publish(shooting_xy)
   
def main():
    rclpy.init()
    state = State()
    rclpy.spin(state)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    