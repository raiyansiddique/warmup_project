import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data
import time

class FiniteStateController(Node):
    """ Finite State Machine Controller """
    def __init__(self):
        super().__init__('wall_approach')
        # the run_loop adjusts the robot's velocity based on latest laser data
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(LaserScan, 'scan', self.process_scan, qos_profile=qos_profile_sensor_data)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # distance_to_obstacle is used to communciate laser data to run_loop
        self.distance_to_obstacle = None
        # Kp is the constant or to apply to the proportional error signal
        self.Kp = 0.4
        # target_distance is the desired distance to the obstacle in front
        self.angular_vel = 3.14/4 * 1.035
        self.state = 1
        self.rotation = 0
    def run_loop(self):
        '''
        State machine controller
        '''

        if self.state == 1:
            self.forward()
        elif self.state == 2:
            self.leftPerson()
        elif self.state == 3:
            self.rightPerson()
        elif self.state == 4:
            self.leftAvoid()
        elif self.state == 5:
            self.rightAvoid()

    def forward(self):
        '''
        Sets robot to move forward
        '''
        twist = Twist()
        twist.linear.x = 0.2
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

    def rightPerson(self):
        '''
        Sets robot to rotate right
        '''
        twist = Twist()
        twist.linear.x = 0.05
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.2
        self.publisher.publish(twist)

    def leftPerson(self):
        '''
        Sets robot to rotate left
        '''
        twist = Twist()
        twist.linear.x = 0.05
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -0.2
        self.publisher.publish(twist)

    def rightAvoid(self):
        '''
        Sets robot to rotate right
        '''
        self.rotation -= self.angular_vel *0.1
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -self.angular_vel
        self.publisher.publish(twist)
        time.sleep(1)

    def leftAvoid(self):
        '''
        Sets robot to rotate left
        '''
        self.rotation += self.angular_vel *0.1
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.angular_vel
        self.publisher.publish(twist)
        time.sleep(1)


        # self.vel_pub.publish(msg)
        

    def process_scan(self, msg):
        '''
        Controller for state machine defined in run_loop

        Args:
            msg: Topic message for lidar scan
        '''
        ranges_var = msg.ranges
        ranges_min_value = min([num for num in ranges_var if num != 0])
        min_index = ranges_var.index(ranges_min_value)
        print(min_index)
        print(ranges_min_value)
        # if the neato detects an object within 0.8 meters it will
        # object avoid.
        if ranges_min_value < 0.8:
            if ranges_min_value < 0.8:
                if min_index >= 0 and min_index <= 40:
                    self.prev_state = self.state
                    self.state = 5
                elif min_index >= 320 and min_index <= 360:
                    self.prev_state = self.state
                    self.state = 4
                else:
                    # moves forward
                    self.state = 1
            else:
                # once the neato is far enough away it will re-orient.
                if self.rotation < 0:
                    self.state = 4
                elif self.rotation >0:
                    self.state = 5
                else:
                    self.state = 1
        # if no object is detected it will follow the closest object
        # it detects.
        else:
            if min_index >= 15 and min_index <= 180:
                self.state = 3
            elif min_index >= 181 and min_index <= 345:
                self.state = 2
            else:
                self.state = 1


def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()