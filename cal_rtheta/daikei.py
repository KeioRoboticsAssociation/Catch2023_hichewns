import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math

class Daikei(Node):
    def __init__(self):
        super().__init__('daikei')
        self.subscription = self.create_subscription()
        self.publisher_ = self.create_publisher(Float32MultiArray, 'daikei', 10)
        self.tmr = self.create_timer(0.05, self.callback)

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


    

def main():
    rclpy.init()
    daikei = Daikei()
    rclpy.spin(daikei)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
