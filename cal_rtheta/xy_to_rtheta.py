import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
import numpy as np
from geometry_msgs.msg import Point

class XY_to_Rtheta(Node):
    def __init__(self):
        super().__init__('xy_to_rtheta')
        self.pos_publisher = self.create_publisher(Float32MultiArray, 'pos_data', 10)
        #self.degpos_publisher = self.create_publisher(Float32MultiArray, 'degpos_data', 10)
        self.subscription = self.create_subscription(Point, 'input', self.input_callback, 1)
        self.tmr = self.create_timer(0.1, self.callback)
        self.status = 0
        self.r = 0.0
        self.y = 0.0
        
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
        self.degPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.x=0.0
        self.y=0.0

    def input_callback(self,input_msg):
        self.x=input_msg.x
        self.y=input_msg.y

        self.currentPos[0]=math.atan2(self.y,self.x)
        self.currentPos[1]=math.sqrt(self.x**2+self.y**2)
        self.currentPos[2]=0.0
        self.currentPos[3]=0.0
        self.currentPos[4]=0.0
        self.currentPos[5]=0.0
        self.currentPos[6]=0.0

    def callback(self):
        pos_data = Float32MultiArray()
        pos_data.data = self.currentPos
        self.pos_publisher.publish(pos_data)
        # degpos_data = Float32MultiArray()
        # degpos_data.data = self.degPos
        # self.degpos_publisher.publish(degpos_data)


def main():
    rclpy.init()
    xy_to_rtheta = XY_to_Rtheta()
    rclpy.spin(xy_to_rtheta)
    rclpy.shutdown()


if __name__ == '__main__':
    main()