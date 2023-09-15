import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data

class ObstacleAvoid(Node):
    """
    Class for Obstacle Avoidance using ROS
    """
    def __init__(self):
        super().__init__('Obstacle Avoid')
        # The run_loop adjusts the robot's velocity based on latest laser data
        self.create_timer(0.1, self.run_loop)
        #Subscribes to the lidar scan topic
        self.create_subscription(LaserScan, 'scan', self.process_scan, qos_profile=qos_profile_sensor_data)
        #Sets up publisher to cmd_vel 
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        #Sets angular velocity to pi/4 rads per second with a scaling factor
        self.angular_vel = 3.14/4 * 1.035
        #Sets the initial state to 1
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
        Sets robot to rotate right
        '''
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.angular_vel
        self.publisher.publish(twist)
    def left(self):
        '''
        Sets robot to rotate left
        '''
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -self.angular_vel
        self.publisher.publish(twist)
        

    def process_scan(self, msg):
        '''
        Controller for state machine defined in run_loop

        Args:
            msg: Topic message for lidar scan
        '''
        # extracts the distances away the neato is from some object at each angle
        ranges_var = msg.ranges
        #Finds the angle where there is something closest to the neato, excluding 0s
        ranges_min_value = min([num for num in ranges_var if num != 0])
        #The minimum index gives us the angle at which the object closest is.
        min_index = ranges_var.index(ranges_min_value)
        #If something is 0.7 meters or closer then it should see if it needs to rotate
        #Else go forward
        if ranges_min_value < 0.7:
            #If the object is in the front of the neato then rotate left else move forward
            if min_index >= 0 and min_index <= 90:
                self.state = 2
            elif min_index >= 270 and min_index <= 360:
                self.state = 2
            else:
                self.state = 1
        else: 
            self.state = 1

        


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoid()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()