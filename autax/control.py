import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from autax.GNSS import GNSS

class Control(Node):
    def __init__(self):
        super().__init__('control_node')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/waypoint',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.GPS = GNSS()
        self.get_logger().info('control_node initiated')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received waypoint: Latitude: {msg.latitude}, Longitude: {msg.longitude}, Altitude: {msg.altitude}')
        self.GPS.navigate(publisher=self, target_lat=msg.latitude, target_long=msg.longitude)
        self.goalTest(msg)

    def goalTest(self, msg):
        if msg.position_covariance_type == 1:
            self.get_logger().info('reached destination')
        else:
            self.get_logger().info('reached waypoint')



def main(args=None):
    rclpy.init(args=args)
    control_node = Control()

    try:
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        pass
    finally:
        control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

