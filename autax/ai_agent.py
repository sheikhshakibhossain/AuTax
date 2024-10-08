import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from std_msgs.msg import String  # Fixed import statement
from rclpy.time import Time

class CmdVelSelector(Node):

    def __init__(self):
        super().__init__('ai_agent_node')

        # Create a publisher for the /cmd_vel topic
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to /vision/cmd_vel, /gnss/cmd_vel, and /laser/cmd_vel
        self.vision_sub = self.create_subscription(
            Twist,
            '/vision/cmd_vel',
            self.vision_callback,
            10
        )
        self.gnss_sub = self.create_subscription(
            Twist,
            '/gnss/cmd_vel',
            self.gnss_callback,
            10
        )
        self.localization_sub = self.create_subscription(
            Twist,
            '/laser/cmd_vel',
            self.localization_callback,
            10
        )

        # Subscribe to emergency stop topic
        self.emergency_sub = self.create_subscription(
            String,
            '/emergency',
            self.emergency_callback,
            10
        )

        # Variables to store the latest messages from all topics
        self.latest_cmd_vel = None
        self.use_gnss = True  # Start by using GNSS as the default
        self.use_vision = False
        self.use_laser = False
        self.emergency_stop = False

        self.stop_cmd_vel = Twist()
        self.stop_cmd_vel.linear.x = 0.0
        self.stop_cmd_vel.angular.z = 0.0

        # Time of the last received message
        self.last_vision_msg_time = self.get_system_time()
        self.last_gnss_msg_time = self.get_system_time()
        self.last_laser_msg_time = self.get_system_time()

        # Timeout duration (e.g., 1 second)
        self.timeout_duration = Duration(seconds=1)

        # Timer to publish to /cmd_vel at regular intervals
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)

    def emergency_callback(self, msg):
        msg = msg.data
        msg = msg.strip()
        if msg == 'STOP':
            self.emergency_stop = True
        elif msg == 'RESUME':
            self.emergency_stop = False
        self.get_logger().info(f'*** Emergency Protocol Initiated: {msg}')

    def get_system_time(self):
        """Helper function to return the current time using the system clock."""
        return self.get_clock().now().to_msg()

    def vision_callback(self, msg):
        """Callback for /vision/cmd_vel, priority over GNSS but lower than laser."""
        self.latest_cmd_vel = msg
        self.use_vision = True  # Prioritize vision when vision data is available
        self.use_gnss = False
        self.use_laser = False  # Laser takes priority over vision
        self.last_vision_msg_time = self.get_system_time()  # Update the time of the last message
        self.get_logger().info('Received data from /vision/cmd_vel, prioritizing vision.')

    def gnss_callback(self, msg):
        """Callback for /gnss/cmd_vel, used as the default if no vision or laser data."""
        self.last_gnss_msg_time = self.get_system_time()  # Update the time of the last message
        if self.use_gnss:
            self.latest_cmd_vel = msg
            self.use_laser = False
            self.use_vision = False
            self.get_logger().info('Received data from /gnss/cmd_vel, using it as default.')

    def localization_callback(self, msg):
        """Callback for /laser/cmd_vel, top priority."""
        self.latest_cmd_vel = msg
        self.use_laser = True  # Laser has top priority
        self.use_vision = False
        self.use_gnss = False  # Disable GNSS and vision when laser data is available
        self.last_laser_msg_time = self.get_system_time()  # Update the time of the last message
        self.get_logger().info('Received data from /laser/cmd_vel, prioritizing laser.')

    def time_difference(self, start, end):
        """Helper function to calculate time difference between two Time objects."""
        start_time = Time(seconds=start.sec, nanoseconds=start.nanosec)
        end_time = Time(seconds=end.sec, nanoseconds=end.nanosec)
        return end_time - start_time

    def publish_cmd_vel(self):
        """Publishes the latest cmd_vel to /cmd_vel."""
        now = self.get_system_time()

        # Check if laser data is stale (timeout)
        if self.time_difference(self.last_laser_msg_time, now) > self.timeout_duration:
            self.use_laser = False  # Laser data is outdated, switch to vision or GNSS

        # Check if vision data is stale (timeout)
        if self.time_difference(self.last_vision_msg_time, now) > self.timeout_duration:
            self.use_vision = False  # Vision data is outdated, switch to GNSS

        # Check if GNSS data is stale (timeout)
        if self.time_difference(self.last_gnss_msg_time, now) > self.timeout_duration:
            self.get_logger().warn('No valid cmd_vel msg from GNSS, stopping publication.')
            return  # Do not publish anything if no valid data

        # check if user paused it from dashboard
        if self.emergency_stop:
            self.cmd_vel_pub.publish(self.stop_cmd_vel)
            self.get_logger().info(f'*** Emergency Protocol ***')
            return

        # Publish the highest-priority available command
        if self.latest_cmd_vel is not None:
            self.cmd_vel_pub.publish(self.latest_cmd_vel)
            self.get_logger().info('Publishing data to /cmd_vel')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSelector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
