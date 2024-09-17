import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class WaypointSubscriber(Node):
    def __init__(self):
        super().__init__('waypoint_subscriber')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/waypoint',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('waypoint_subscriber initiated')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received waypoint: Latitude: {msg.latitude}, Longitude: {msg.longitude}, Altitude: {msg.altitude}')
        self.goalTest(msg)


    def goalTest(self, msg):
        if msg.position_covariance_type == 1:
            self.get_logger().info('reached')



def main(args=None):
    rclpy.init(args=args)
    waypoint_subscriber = WaypointSubscriber()

    try:
        rclpy.spin(waypoint_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        waypoint_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

