import rclpy
from rclpy.node import Node
from neato2_interfaces.msg  import Bump
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class EStop(Node):
    def __init__(self):
        super().__init__('drive_square')
        self.initial_odom = None
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, 'odom', self.init_odom, 10)
        self.sub1 = self.create_subscription(Odometry, 'odom', self.check_dist, 10)
    def init_odom(self, msg):
        if self.initial_odom == None:
            self.initial_odom = msg.pose
            print(self.initial_odom)
            print(type(self.initial_odom))
    def check_dist(self, msg):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = EStop()   # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()
