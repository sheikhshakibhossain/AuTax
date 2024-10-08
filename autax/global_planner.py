import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from autax.GNSS import GNSS

class Control(Node):
    def __init__(self):
        super().__init__('global_planner')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/waypoint',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.GPS = GNSS()
        self.get_logger().info('global_planner initiated')
        while self.GPS.gps_ser.in_waiting > 0:
            self.GPS.publish_nav_sat_data()

    def listener_callback(self, msg):
        self.get_logger().info(f'Received waypoint: Latitude: {msg.latitude}, Longitude: {msg.longitude}, Altitude: {msg.altitude}')
        try:
            self.GPS.navigate(publisher=self, target_lat=msg.latitude, target_long=msg.longitude)
        except KeyboardInterrupt:
            self.get_logger().info('keyboard interrupt')
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

