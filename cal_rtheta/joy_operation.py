import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
import math
import time

class Joy_operation(Node):
    def __init__(self):
        super().__init__('joy_operation')
        self.pos_publisher = self.create_publisher(Float32MultiArray, 'pos_data', 10)
        self.catch_judge_publisher = self.create_publisher(Bool, 'catch_data', 10)
        self.degpos_publisher = self.create_publisher(Float32MultiArray, 'degpos_data', 10)
        self.subscription = self.create_subscription(Float32MultiArray, 'joy_data', self.joy_callback, 10)
        self.tmr = self.create_timer(0.1, self.callback)
        self.status = 0
        self.r = 0.0
        self.y = 0.0
        # arm2
        self.up = 0.0
        self.mid = 0.0
        self.down = 0.0
        self.revarm3 = 0.0
        self.flag = 1
        self.rev = 0.0 #hand1-3
        self.catch = 0.0
        self.release = 0.0
        self.init = 0.0
        self.grasp = Bool()
        self.currentPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.degPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def joy_callback(self, joy_msg):
        self.r = joy_msg.data[0]
        self.y = joy_msg.data[1]
        self.mid = joy_msg.data[2]
        self.up = joy_msg.data[3]
        self.down = joy_msg.data[4]
        self.revarm3 = joy_msg.data[5]
        self.rev = joy_msg.data[6]
        self.init = joy_msg.data[7]
        self.catch = joy_msg.data[8]
        self.release = joy_msg.data[9]
  
        if self.init==1:
            self.currentPos=[0.0, 0.1, 0.08, 0.0, 0.0, 0.0]
            self.degPos=[0.0, 0.1, 0.08, 0.0, 0.0, 0.0]

        if self.r >= 0:
            self.currentPos[0] += self.r / 100
            self.degPos[0] += math.degrees(self.r / 100)

        else:
            self.currentPos[0] -= abs(self.r) / 100
            self.degPos[0] -= math.degrees(abs(self.r) / 100)

        if self.y > 0:
            self.currentPos[1] += self.y / 500
            self.degPos[1] += self.y / 500
            if self.currentPos[1]>=0.542:
                self.currentPos[1]=0.542
            if self.degPos[1]>=0.542:
                self.degPos[1]=0.542

        else:
            self.currentPos[1] -= abs(self.y) / 500
            self.degPos[1] -= abs(self.y) / 500
            if self.currentPos[1]<=0.0:
                self.currentPos[1]=0.0
            if self.degPos[1]<=0.0:
                self.degPos[1]=0.0

        if self.up == 1.0:
            self.currentPos[2] = 0.08
            self.degPos[2] = 0.08
        
        if self.mid == 1.0:
            self.currentPos[2] = 0.04
            self.degPos[2] = 0.04
        
        if self.down == 1.0:
            self.currentPos[2] = -0.08
            self.degPos[2] = -0.08

                
        if self.revarm3 >= 0.0:
            self.currentPos[3] += self.revarm3 / 50
            self.degPos[3] += math.degrees(self.revarm3 / 50)

        else:
            self.currentPos[3] -= abs(self.revarm3) / 50
            self.degPos[3] -= math.degrees(abs(self.revarm3) / 50)

        if self.catch==1.0:
            self.grasp.data = True
        
        if self.release==1.0:
            self.grasp.data = False

        self.catch_judge_publisher.publish(self.grasp)


        
        if self.rev == 1.0:
            if self.flag == 1:
                if self.status==1:
                    self.currentPos[4] = math.pi/4
                    self.currentPos[5] = math.pi/4
                    self.currentPos[6] = math.pi/4
                    self.degPos[4] = math.degrees(math.pi/4)
                    self.degPos[5] = math.degrees(math.pi/4)
                    self.degPos[6] = math.degrees(math.pi/4)
                    self.status = 0
                    self.flag = 0

                elif self.status==0:
                    self.currentPos[4] = 0.0
                    self.currentPos[5] = 0.0
                    self.currentPos[6] = 0.0
                    self.degPos[4] = math.degrees(0.0)
                    self.degPos[5] = math.degrees(0.0)
                    self.degPos[6] = math.degrees(0.0)
                    self.status = 1
                    self.flag = 0 

            elif self.flag == 0:
                time.sleep(0.2)
                self.flag = 1

    def callback(self):
        pos_data = Float32MultiArray()
        pos_data.data = self.currentPos
        self.pos_publisher.publish(pos_data)
        degpos_data = Float32MultiArray()
        degpos_data.data = self.degPos
        degpos_data.data[1]*=1000
        self.degpos_publisher.publish(degpos_data)


def main():
    rclpy.init()
    cal = Joy_operation()
    rclpy.spin(cal)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
