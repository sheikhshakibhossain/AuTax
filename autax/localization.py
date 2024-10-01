import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObjectAvoidanceNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        
        # Create a subscriber for the /scan topic
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Create a publisher for the /laser/cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/laser/cmd_vel', 10)
        
        # Create a Twist message for movement commands
        self.twist_msg = Twist()

        self.get_logger().info('Object Avoidance Node initialized')

    def scan_callback(self, msg):
        # Analyze laser scan data to detect objects
        front_distance = min(msg.ranges[60:120] + msg.ranges[-30:])  # Front range (0 to 30 degrees)
        left_distance = min(msg.ranges[121:195])  # Left range (60 to 90 degrees)
        right_distance = min(msg.ranges[0:61])  # Right range (-90 to -60 degrees)

        if front_distance < 0.2:  # Stop if an object is very close
            self.stop()
        elif front_distance < 0.5:
            # Decide whether to go left or right
            if left_distance > right_distance:
                self.turn_right()
            else:
                self.turn_left()

        else:
            self.get_logger().info('clear')

    def stop(self):
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0
        self.publisher_.publish(self.twist_msg)
        self.get_logger().info('Stopping: Object detected too close')

    def turn_left(self):
        self.twist_msg.linear.x = 0.7  # Move forward
        self.twist_msg.angular.z = 0.5  # Turn left
        self.publisher_.publish(self.twist_msg)
        self.get_logger().info('Turning left')

    def turn_right(self):
        self.twist_msg.linear.x = 0.7  # Move forward
        self.twist_msg.angular.z = -0.5  # Turn right
        self.publisher_.publish(self.twist_msg)
        self.get_logger().info('Turning right')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
