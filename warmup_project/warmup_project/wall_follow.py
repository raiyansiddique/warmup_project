import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data


class WallFollowing(Node):
    """ Class for WallFollowing for the Neato"""
    def __init__(self):
        super().__init__('wall_follow')
        # the run_loop adjusts the robot's velocity based on latest laser data
        self.create_timer(0.1, self.run_loop)
        #Subscribe to the lidar scan topic
        self.create_subscription(LaserScan, 'scan', self.process_scan, qos_profile=qos_profile_sensor_data)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state = 1
    def run_loop(self):
        '''
        Using the state, drive the robot one of 3 directions
        1: Forward
        2: Rotate Left
        3: Rotate Right
        '''
        if self.state == 1:
            self.forward()
        elif self.state == 2:
            self.left()
        elif self.state == 3:
            self.right()

    def forward(self):
        '''
        Sets the robot's forward velocity to 0.2 m/s
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
        Sets robot to rotate right and move forward
        '''
        twist = Twist()
        twist.linear.x = 0.2
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.2
        self.publisher.publish(twist)

    def left(self):
        '''
        Sets robot to rotate left and move forward
        '''
        twist = Twist()
        twist.linear.x = 0.2
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -0.2
        self.publisher.publish(twist)
        

    def process_scan(self, msg):
        '''
        Controller for state machine defined in run_loop. Only follows wall to
            left of the robot

        Args:
            msg: Topic message for lidar scan
        '''
        left_low = 45
        left_high = 135
        #Uses to angles to construct a wall between those two points if there is a line to draw between the two
        ranges_var = msg.ranges
        if ranges_var[left_low] != 0 and ranges_var[left_high] != 0:
            if ranges_var[left_low] - ranges_var[left_high] > 0.05:
                self.state = 3
            elif ranges_var[left_low] - ranges_var[left_high] < -0.05:
                self.state = 2
            else:
                self.state = 1

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowing()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()