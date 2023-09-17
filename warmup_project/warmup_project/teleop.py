import tty
import select
import sys
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

#Global Settings variable for keyboard
settings = termios.tcgetattr(sys.stdin) 
def getKey():
    '''
    Gets key value that user presses
    Returns:
        a string that represents the key the user pressed.
    '''
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')
        #Creates publisher to cmd_vel topic
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
    def stop(self):
        '''
        Stops the robot
        '''
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        
    def forward(self):
        '''
        Sets the linear velocity to move the robot forward
        '''
        twist = Twist()
        twist.linear.x = 0.2
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        
    def right(self):
        '''
        Sets the angular velocity to have the robot rotate right
        '''
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -0.2
        self.publisher.publish(twist)
        
    def backward(self):
        '''
        Sets the linear velocity to negative to move the robot backwards
        '''
        twist = Twist()
        twist.linear.x = -0.2
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        
    def left(self):
        '''
        Sets the angular velocity to have the robot rotate left
        '''
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.2
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = Teleop()
    key = None
    #While control c isn't clicked by the user
    while key != '\x03':
        #Gets what key is clicked every 0.05 seconds.
        key = getKey()
        #Use wasd to move and rotate the robot forward and f to stop
        if key == 'w':
            node.forward()
        elif key =='s':
            node.backward()
        elif key == 'd':
            node.right()
        elif key == 'a':
            node.left()
        elif key == 'f':
            node.stop()
        time.sleep(0.05)
    node.stop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()