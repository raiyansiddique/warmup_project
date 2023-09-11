import rclpy
from rclpy.node import Node
from neato2_interfaces.msg  import Bump
from geometry_msgs.msg import Twist
import time
from geometry_msgs.msg import Vector3


class EStop(Node):
    def __init__(self):
        super().__init__('teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        time.sleep(1)
        self.state = 0
        self.linear_vel = 0.2
        self.angular_vel = 3.14/4 * 1.035
    def square(self):
        count = 0
        while count < 4:
            if self.state == 0:
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
    node = EStop()
    node.square()
    rclpy.shutdown()

if __name__ == '__main__':
    main()