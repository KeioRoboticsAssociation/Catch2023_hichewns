import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class Input(Node):
    def __init__(self):
        super().__init__('input')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'input', 10)
        self.tmr = self.create_timer(0.1, self.callback)
        self.xy=[0.0,0.0]
        
    
    def callback(self,x,y):
        self.xy[0]=x
        self.xy[1]=y
        input_data = Float32MultiArray()
        input_data.data = self.xy
        self.pos_publisher.publish(input_data)

def main(args=None):
    rclpy.init(args=args)
    input_publisher = Input()

    while rclpy.ok():
        try:
            x = float(input('Enter x coordinate: '))
            y = float(input('Enter y coordinate: '))
        except ValueError:
            print('Invalid input! Please enter numerical values.')
            continue
        
        input_publisher.callback(x, y)

    input_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


