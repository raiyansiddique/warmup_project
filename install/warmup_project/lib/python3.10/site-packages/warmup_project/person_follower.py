import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data

# 1 = Wall Left
# 2 = Wall Right
# 3 = Rotate to Left Wall
# 4 = Rotate to Right Wall
class WallApproachNode(Node):
    """ This class wraps the basic functionality of the node """
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
        self.target_distance = 1.2
        self.state = 1
    def run_loop(self):
        msg = Twist()
        print(self.state)

        if self.state == 1:
            self.forward()
        elif self.state == 2:
            self.left()
        elif self.state == 3:
            self.right()

        # Your logic here!

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
        twist.linear.x = 0.2
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.2
        self.publisher.publish(twist)

    def left(self):
        twist = Twist()
        twist.linear.x = 0.2
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -0.2
        self.publisher.publish(twist)


        # self.vel_pub.publish(msg)
        

    def process_scan(self, msg):

        ranges_var = msg.ranges
        ranges_min_value = min([num for num in ranges_var if num != 0])
        min_index = ranges_var.index(ranges_min_value)
        if min_index >= 5 and min_index <= 180:
            self.state = 2
        elif min_index >= 181 and min_index <= 355:
            self.state = 3
        else:
            self.state = 1

            # if ranges_var[left] < 0.1:
            #     self.state = 3
            # elif ranges_var[left] > 0.2:
            #     self.state = 2

        


def main(args=None):
    rclpy.init(args=args)
    node = WallApproachNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()