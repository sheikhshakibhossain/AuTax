import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial

class SerialToLidar(Node):

    def __init__(self):
        super().__init__('laser_scan')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)

        # Set up the serial connection
        self.serial_port = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1)

        # Create a timer that runs every 0.5 seconds
        self.timer = self.create_timer(0.5, self.read_and_publish)

        # Set up parameters for LaserScan message
        self.angle_min = 0.0  # 0 degrees
        self.angle_max = 3.14159  # 180 degrees (3.14159 radians)
        self.angle_increment = 3.14159 / 360  # 1 degree increments
        self.range_min = 0.02  # Minimum range of the sensor (20 cm)
        self.range_max = 4.0  # Maximum range of the sensor (400 cm)
        self.num_readings = 360  # Total number of readings (360 for 180 degrees)

        self.get_logger().info('Serial to Lidar node initialized')
        

    def read_and_publish(self):
        # Read from the serial port
        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            data = line.split(',')
            # print(data)

            if len(data) == 6:  # Ensure we have 6 distance values
                distances = [float(d) / 100.0 for d in data]
                self.publish_laser_scan(distances)
            else:
                self.get_logger().warn('Invalid data length received from serial port')

        except Exception as e:
            self.get_logger().error(f'Failed to parse data: {e}')

    def publish_laser_scan(self, distances):
        # Create a LaserScan message
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()  # Add the current time
        scan_msg.header.frame_id = 'base_scan'  # Match the frame_id expected by RViz

        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max

        # Initialize ranges array with infinities
        scan_msg.ranges = [float('inf')] * self.num_readings

        # Assign distances to angular ranges in laser_scan_data
        # We are assigning distances[0] to 165° to 195°, distances[1] to 135° to 165°, and so on.
        angle_ranges = [
            (180 - 15, 180 + 15),  # distances[0] -> 165° to 195°
            (180 - 30, 180 - 15),  # distances[1] -> 135° to 165°
            (180 - 45, 180 - 30),  # distances[2] -> 105° to 135°
            (180 - 60, 180 - 45),  # distances[3] -> 75° to 105°
            (180 - 75, 180 - 60),  # distances[4] -> 45° to 75°
            (180 - 90, 180 - 75)   # distances[5] -> 15° to 45°
        ]

        for i, (start_angle, end_angle) in enumerate(angle_ranges):
            # Convert angles to indices in the scan_msg.ranges array
            start_index = int(start_angle)
            end_index = int(end_angle)

            # Assign the corresponding distance to each angle range
            for idx in range(start_index, end_index):
                if 0 <= idx < self.num_readings:
                    scan_msg.ranges[idx] = distances[i] if distances[i] > self.range_min else float('inf')

        # Publish the LaserScan message
        self.publisher_.publish(scan_msg)
        self.get_logger().info('Published LaserScan data')
        # print(scan_msg)
        # self.get_logger().info(f'Publishing ranges: {scan_msg.ranges}')


def main(args=None):
    rclpy.init(args=args)
    node = SerialToLidar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
