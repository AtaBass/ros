import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from math import sqrt

class Turtlebot3Controller(Node):
    def __init__(self):
        super().__init__('turtlebot3_controller')
        
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.current_position = Point()
        
        
        self.target_position = Point()
        self.target_position.x = 1.0 
        self.target_position.y = 1.0  
        
        self.linear_speed = 0.2  # Linear speed
        self.distance_threshold = 0.1  # Stopping distance
        
        
        self.create_timer(0.1, self.move_to_target)
    
    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
    
    def get_distance_to_target(self):
        return sqrt(
            (self.target_position.x - self.current_position.x) ** 2 +
            (self.target_position.y - self.current_position.y) ** 2
        )
    
    def move_to_target(self):
        distance = self.get_distance_to_target()
        
        vel_msg = Twist()
        
        if distance > self.distance_threshold:
            vel_msg.linear.x = min(self.linear_speed, distance)
        else:
            vel_msg.linear.x = 0.0
            self.cmd_vel_pub.publish(vel_msg)
            self.get_logger().info("Target reached!")
            return
        
        self.cmd_vel_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = Turtlebot3Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
