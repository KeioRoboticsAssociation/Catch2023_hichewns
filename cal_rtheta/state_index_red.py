import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Int32
from catch2023_interfaces.msg import CreateMessage
import time
import csv
import math

class State(Node):
    def __init__(self):
        super().__init__('state')
        # self.move_flg_subscription = self.create_subscription(Bool, 'move_flg', self.move_flg_callback, 10)

        self.ping_publisher = self.create_publisher(Bool, 'ping', 10)
        self.real_pos_subscription = self.create_subscription(CreateMessage, 'real_pos', self.real_pos_callback, 10)
        self.cmd_state_subscription = self.create_subscription(String,'cmd_state',self.cmd_state_callback,10)
        # self.pose_subscription = self.create_subscription(Int8, 'index', self.index_callback, 10)
        # self.shooting_index_subscription = self.create_subscription(Int8, 'shooting_index', self.shooting_index_callback, 10)
        self.target_pose_subscription = self.create_subscription(Float32MultiArray, 'target_pose', self.target_pose_callback,10)
        self.shooting_pose_subscription = self.create_subscription(Float32MultiArray, 'shooting_pose', self.shooting_pose_callback,10)
        self.target_comp_subscription = self.create_subscription(Bool, 'target_comp', self.target_comp_callback, 10)
        # self.target_comp_r_subscription = self.create_subscription(Bool, 'target_comp_r', self.target_comp_r_callback, 10)
        
        self.auto_start_subscription = self.create_subscription(Bool,'auto_start', self.auto_start_callback, 10)

        self.target_error_subscription = self.create_subscription(Float32, 'target_error', self.target_error_callback, 10)
        self.targetcomp_publisher = self.create_publisher(Bool, 'target_comp', 10)
        # self.target_comp_r_publisher = self.create_publisher(Bool, 'target_comp_r', 10)
        self.shooting_comp_subscription = self.create_subscription(Bool, 'shooting_comp', self.shooting_comp_callback, 10)
        self.shooting_comp_publisher = self.create_publisher(Bool, 'shooting_comp', 10)
        self.state_publisher = self.create_publisher(Int32MultiArray, 'state_data', 10)
        self.stepper_publisher = self.create_publisher(Int32, 'stepper_cmd', 10)
        self.servo_publisher = self.create_publisher(Int8, 'servo_cmd', 10)
        self.target_publisher = self.create_publisher(Float32MultiArray, 'target_xy', 10)
        self.shootingbox_publisher = self.create_publisher(Float32MultiArray, 'shootingbox_xy', 10)
        self.init_publisher = self.create_publisher(Bool, 'init', 10)
        self.move_publisher = self.create_publisher(Bool, 'move_cmd', 10)
        self.release_publisher = self.create_publisher(Bool, 'release_cmd', 10)
        

        self.seiton_callback = self.create_subscription(Int8, 'seiton', self.seiton_callback, 10)
        self.index_publisher = self.create_publisher(Int8, 'index', 10)
        self.box_publisher = self.create_publisher(Int8, 'shooting_index', 10)
        # self.is_manual_publisher = self.create_publisher(Bool, 'is_manual', 10)
        # self.is_manual_subscription = self.create_subscription(Bool, 'is_manual', self.is_manual_callback,10)
        self.seiton_publisher = self.create_publisher(Int8, 'seiton', 10)
        self.start_publisher = self.create_publisher(Bool, 'start', 10)
        # self.joy_subscription = self.create_subscription(Float32MultiArray, 'joy_data', self.joy_callback, 10)
        
        self.tmr = self.create_timer(0.1, self.callback)

        self.red_own_target = []
        self.red_shooting_box = []
        self.theta = 0.0
        self.state = 0
        self.cnt = 0
        self.box = 0
        self.init = False

        self.STP_INIT_POS = 0 #0
        self.STP_SHOOT_POS = 40 #1
        self.STP_POI_POS = 110 #1.5
        self.STP_SERCH_POS = 200 #2
        self.STP_SERCH_COMMON_POS = 187
        self.STP_COMMON_POS = 220 #3
        self.STP_PUT_POS = 250 #4
        self.STP_OWN_POS = 270 #5
        self.stepper_cmd = self.STP_INIT_POS

        self.servo_cmd = 1
        self.move_cmd = False
        self.is_ended = False
        self.target_cmd = False
        self.shooting_cmd = False
        self.state_data = [0,0,0]
        self.target_comp = False
        self.target_comp_r = False
        self.ping = True
        self.seiton = 3
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

        self.start = False

        with open('/home/moyuboo/ros2_ws/src/catch2023/cal_rtheta/csv/pose_red.csv', 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                self.red_own_target.append([float(row[0]),float(row[1])])

        with open('/home/moyuboo/ros2_ws/src/catch2023/cal_rtheta/csv/shooting_red.csv', 'r') as f:
            reader1 = csv.reader(f)
            for row in reader1:
                self.red_shooting_box.append([float(row[0]),float(row[1])])
    
    def auto_start_callback(self, auto_start_msg):
        self.auto_start = auto_start_msg.data
        if self.auto_start == True:
            self.state = 2
            self.auto_start = False
    
    def target_error_callback(self, target_error_msg):
        self.target_error = target_error_msg.data
        if self.target_error < 2.0:
            self.target_comp_r = True
        else:
            self.target_comp_r = False
    
    def target_comp_callback(self, target_comp_msg):
        self.target_comp = target_comp_msg.data
        # if self.target_comp == True:
        #     self.target_cmd = False
            # self.state = 2
    
    def target_comp_r_callback(self, target_comp_r_msg):
        self.target_comp_r = target_comp_r_msg.data
        # if self.target_comp_r == True:
        #     self.target_cmd = False
            # self.state = 2
    
    def real_pos_callback(self, real_pos_msg):
        self.stepper = real_pos_msg.stepper
        self.real_r = real_pos_msg.r / 1000.0 
    
    def seiton_callback(self, seiton_msg):
        self.seiton = seiton_msg.data
    
    def shooting_comp_callback(self, shooting_comp_msg):
        self.shooting_comp = shooting_comp_msg.data
        if self.shooting_comp == True:
            self.shooting_cmd = False

            self.shooting_comp = False
            shootingcomp = Bool()
            shootingcomp.data = self.shooting_comp
            self.shooting_comp_publisher.publish(shootingcomp)
            self.state = 7
    
    def target_pose_callback(self,targetpos):
        self.red_own_target[self.index] = targetpos.data
        with open('/home/moyuboo/ros2_ws/src/catch2023/cal_rtheta/csv/re_pose_red.csv','w') as f:
            writer = csv.writer(f)
            writer.writerows(self.red_own_target)
    
    def shooting_pose_callback(self,shootingpos):
        self.red_shooting_box[self.box] = shootingpos.data
        with open('/home/moyuboo/ros2_ws/src/catch2023/cal_rtheta/csv/re_shooting_red.csv','w') as f:
            writer = csv.writer(f)
            writer.writerows(self.red_shooting_box)
    
    def cmd_state_callback(self, cmd_state):
        if cmd_state.data == 'i':
            self.init = True
            init_msg = Bool()
            init_msg.data = True
            self.init_publisher.publish(init_msg)
            self.state = 1
            self.init = False

    def callback(self):
        if self.state == 0:
            self.shooting_cmd = False
            self.target_cmd = False
            self.move_cmd = False
            self.start = True
            self.stepper_cmd = self.STP_INIT_POS
            start = Bool()
            start.data = self.start
            self.start_publisher.publish(start)
            if self.init == True:
                self.shooting_cmd = False
                self.target_cmd = False
                self.move_cmd = False

        elif self.state == 1:
            self.shooting_cmd = False
            self.target_cmd = False
            self.move_cmd = False


        elif self.state == 2:
            index = Int8()
            index.data = self.index
            self.index_publisher.publish(index)

            box = Int8()
            box.data = self.box
            self.box_publisher.publish(box)

            self.move_cmd = False
            self.release_cmd = True
            release_cmd = Bool()
            release_cmd.data = self.release_cmd
            self.release_publisher.publish(release_cmd)
            self.target_cmd = True

            if self.index == 4 and self.box == 8:
                self.state = 9

            if self.index == 1 or self.index == 2:
                self.seiton = 0
            
            elif self.index == 3 or self.index == 4:
                self.seiton = 1

            elif self.index == 0 or self.index == 5 or self.index == 6 or self.index == 7 or self.index == 8:
                self.seiton = 2

            # servo_cmd = Int8()
            # if self.index < 6:
            #     self.servo_cmd = 1

            # elif self.index >= 6:
            #     self.servo_cmd = 0
            # servo_cmd.data = self.servo_cmd
            # self.servo_publisher.publish(servo_cmd) 

            if self.target_comp == True:
                if not(self.index == 1 or self.index == 2 or self.index == 3):
                    self.stepper_cmd = self.STP_SERCH_POS
    
                elif self.index == 1 or self.index == 2 or self.index == 3:
                    self.stepper_cmd = self.STP_SERCH_COMMON_POS

            if self.target_comp_r == True:
                seiton = Int8()
                seiton.data = self.seiton
                self.seiton_publisher.publish(seiton)
                if not(self.index == 1 or self.index == 2 or self.index == 3):
                    if abs(self.stepper - self.STP_SERCH_POS) <= 1:
                        self.target_comp = False
                        targetcomp = Bool()
                        targetcomp.data = self.target_comp
                        self.targetcomp_publisher.publish(targetcomp)
                        self.target_comp_r = False
                        self.target_cmd = False
                        self.state = 3

                elif self.index == 1 or self.index == 2 or self.index == 3:
                    if abs(self.stepper - self.STP_SERCH_COMMON_POS) <= 1:
                        self.target_comp = False
                        targetcomp = Bool()
                        targetcomp.data = self.target_comp
                        self.targetcomp_publisher.publish(targetcomp)
                        self.target_comp_r = False
                        self.target_cmd = False
                        self.state = 3
                        
        elif self.state == 3:
            self.move_cmd = False

        #ここコメントにした
            if not(self.index == 1 or self.index == 2 or self.index == 3):
                self.stepper_cmd = self.STP_OWN_POS
                if abs(self.stepper - self.STP_OWN_POS) <= 1:
                    self.release_cmd = False
                    release_cmd = Bool()
                    release_cmd.data = self.release_cmd
                    self.release_publisher.publish(release_cmd)
                    time.sleep(0.2)
                    self.state = 4

            elif self.index == 1 or self.index == 2 or self.index == 3:
                self.stepper_cmd = self.STP_COMMON_POS
                if abs(self.stepper - self.STP_COMMON_POS) <= 1:
                    time.sleep(1.0)
                    self.release_cmd = False
                    release_cmd = Bool()
                    release_cmd.data = self.release_cmd
                    self.release_publisher.publish(release_cmd)
                    time.sleep(0.5)
                    self.state = 4

        elif self.state == 4:
            self.move_cmd = False

            # if not(self.index == 1 or self.index == 2 or self.index == 3):
            #     self.state = 5

            # elif self.index == 1 or self.index == 2 or self.index == 3:
            if not self.index == 8:
                self.stepper_cmd = self.STP_INIT_POS
                if abs(self.stepper - self.STP_INIT_POS) <= 1:
                    self.state = 5

            elif self.index == 8:
                self.stepper_cmd = self.STP_SERCH_POS
                if abs(self.stepper - self.STP_SERCH_POS) <= 1:
                    self.state = 5

        elif self.state == 5:
            self.move_cmd = True
            self.stepper_cmd = self.STP_INIT_POS
            if abs(self.stepper - self.STP_INIT_POS) <= 1:
                if self.real_r <= 0.159:
                    self.state = 6

        elif self.state == 6:
            self.move_cmd = False
            self.shooting_cmd = True

        elif self.state == 7:
            if self.box == 0 or self.box == 7 or self.box == 8:
                self.stepper_cmd = self.STP_SERCH_POS
                if abs(self.stepper - self.STP_SERCH_POS) <= 1:
                    self.release_cmd = True
                    release_cmd = Bool()
                    release_cmd.data = self.release_cmd
                    self.release_publisher.publish(release_cmd)
                    time.sleep(0.5)
                    self.state = 8

            else:
                self.stepper_cmd = self.STP_SHOOT_POS
                if abs(self.stepper - self.STP_SHOOT_POS) <= 1:
                    self.release_cmd = True
                    release_cmd = Bool()
                    release_cmd.data = self.release_cmd
                    self.release_publisher.publish(release_cmd)
                    time.sleep(0.5)
                    self.state = 8
                    
        elif self.state == 8:
            self.stepper_cmd = self.STP_INIT_POS
            if abs(self.stepper - self.STP_INIT_POS) <= 1:
                self.shooting_cmd = False
                self.move_cmd = True
                if self.real_r <= 0.159:
                    self.index += 1
                    if self.index > 8:
                        self.index = 4
                    self.box += 1
                    if self.box > 8:
                        self.box = 8
                    self.state = 2
        
        elif self.state == 9:
            self.seiton = 3
            seiton = Int8()
            seiton.data = self.seiton
            self.seiton_publisher.publish(seiton)
        
        ping_data = Bool()
        ping_data.data = self.ping
        self.ping_publisher.publish(ping_data)

        #state_publish
        self.state_data = [self.state, self.index, self.box]
        state_data = Int32MultiArray()
        state_data.data = self.state_data
        self.state_publisher.publish(state_data)

        #stepper_publish
        stepper_data = Int32()
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
    