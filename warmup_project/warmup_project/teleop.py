import tty
import select
import sys
import termios
import rclpy
from rclpy.node import Node
from neato2_interfaces.msg  import Bump
from geometry_msgs.msg import Twist
import time
from geometry_msgs.msg import Vector3

settings = termios.tcgetattr(sys.stdin) 
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class EStop(Node):
    def __init__(self):
        super().__init__('teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        
    def forward(self):
        twist = Twist()
        twist.linear.x = 0.2
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        
    def right(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.2
        self.publisher.publish(twist)
        
    def backward(self):
        twist = Twist()
        twist.linear.x = -0.2
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        
    def left(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -0.2
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = EStop()

    key = None
    while key != '\x03':
        key = getKey()
        print(key)
        print(type(key))
        if key == 'w':
            print("forw")
            node.forward()
        elif key =='s':
            node.backward()
        elif key == 'd':
            node.left()
        elif key == 'a':
            node.right()
        elif key == 'f':
            node.stop()
        time.sleep(0.05)

    rclpy.shutdown()

if __name__ == '__main__':
    main()