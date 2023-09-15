import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class DriveSquare(Node):
    def __init__(self):
        super().__init__('drive_square')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        time.sleep(1)
        self.state = 0
        #Initial linear velocity is 0.2m/s
        self.linear_vel = 0.2
        #This is pi/4 rads per second with a scaling to adjust for any inconsistencies when making the square
        self.angular_vel = 3.14/4 * 1.035
    def square(self):
        '''
        Publishes to the cmd_vel topic to drive the neato in a square
        '''
        count = 0
        #Publish to cmd_vel first to make sure that the topic is ready
        self.publisher.publish(Twist())
        time.sleep(2)
        # While statement to switch between two states until the square is drawn
        while count < 4:
            if self.state == 0:
                '''
                Moves the robot forward approximately 1 meter forward using time
                '''
                twist = Twist()
                twist.linear.x = self.linear_vel
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = 0.0
                self.publisher.publish(twist)
                time.sleep(5)
                self.state = 1
                count += 1
            elif self.state == 1:
                '''
                Rotates the robot left approximately 90 degrees using time
                '''
                twist = Twist()
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = self.angular_vel
                self.publisher.publish(twist)
                time.sleep(2)
                self.state = 0
        self.stop()
    def stop(self):
        '''
        Stops the Robot when called
        '''
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
def main(args=None):
    rclpy.init(args=args)
    node = DriveSquare()
    #Rather than spinning up the node we call the square function in the class. 
    node.square()
    rclpy.shutdown()

if __name__ == '__main__':
    main()