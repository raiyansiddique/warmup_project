import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data

class PersonFollowing(Node):
    """ Class for Person Following """
    def __init__(self):
        super().__init__('person_following')
        # the run_loop adjusts the robot's velocity based on latest laser data
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(LaserScan, 'scan', self.process_scan, qos_profile=qos_profile_sensor_data)
        #Sets up publisher to cmd_vel 
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        #Sets the initial state to 1
        self.state = 1
    def run_loop(self):
        '''
        Using the state, drive the robot one of 3 directions
        1: Forward
        2: Rotate Left and Forward
        3: Rotate Right and Forward
        '''
        if self.state == 1:
            self.forward()
        elif self.state == 2:
            self.left()
        elif self.state == 3:
            self.right()
        elif self.state == 4:
            self.stop()

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
        twist.linear.x = 0.05
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
        twist.linear.x = 0.05
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -0.2
        self.publisher.publish(twist)

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

    def process_scan(self, msg):
        '''
        Controller for state machine defined in run_loop

        Args:
            msg: Topic message for lidar scan
        '''


        #Extracts the ranges from the message
        ranges_var = msg.ranges
        #Finds the value of the object closest to the neato excluding 0s
        ranges_min_value = min([num for num in ranges_var if num != 0])
        #Finds the angle at which the minimum value is at
        min_index = ranges_var.index(ranges_min_value)
        #If the person is to the right of the neato rotate and move forward right
        # if min_index < 15 or min_index > 345:
        #     if ranges_min_value < 0.05:
        #         self.state = 4
        # else: 
        #     if min_index >= 15 and min_index <= 180:
        #         self.state = 3
        #     #If the person is to the left of the neato rotate and move forward left
        #     elif min_index >= 181 and min_index <= 345:
        #         self.state = 2
        #     else:
        #         self.state = 1


        if (min_index < 15 and ranges_min_value < 0.1) or (min_index > 345 and ranges_min_value < 0.1):
            self.state = 4
        elif min_index >= 15 and min_index <= 180:
            self.state = 3
        #If the person is to the left of the neato rotate and move forward left
        elif min_index >= 181 and min_index <= 345:
            self.state = 2
        else:
            self.state = 1


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowing()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()