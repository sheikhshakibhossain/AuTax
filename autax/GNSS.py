import time
import math
import smbus
import serial
import serial.tools.list_ports
import pynmea2
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist, Vector3


class GNSS(Node):

    def __init__(self):
        super().__init__('gnss_navigation_node')
        # Define your constants and variables here
        self.Device_Address = 0x1e
        self.Register_A = 0
        self.Register_B = 0x01
        self.Register_mode = 0x02
        self.X_axis_H = 0x03
        self.Z_axis_H = 0x05
        self.Y_axis_H = 0x07
        self.declination = -0.00669
        self.pi = 3.14159265359
        self.bus = smbus.SMBus(1)

        self.bearingTolerance = 10
        self.distanceTolerance = 0.3
        self.distanceForSteeringRotation = 5.0
        self.targetHeading = 0
        self.currentHeading = 0
        self.targetLat = 0.0
        self.targetLong = 0.0
        self.currentLat = 0.0
        self.currentLong = 0.0
        self.distanceToTarget = 10
        self.flag = 0

        self.cmd_vel_topic = '/gnss/cmd_vel'
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.nmea_msg = None
        self.nmea_msg_str = None
        self.gps_data = None
        self.nav_sat_msg = NavSatFix()
        self.nav_sat_topic = '/nav_sat'
        self.nav_sat_pub = self.create_publisher(NavSatFix, self.nav_sat_topic, 10)
        
        self.gps_port = self.find_serial_device_by_vid_pid(6790) # vid = 1659 of our ttl cable | 6790 for battery ttl
        self.gps_baudrate = 9600
        self.gps_ser = serial.Serial(self.gps_port, baudrate=self.gps_baudrate, timeout=0.5)
        self.get_logger().info(f'Opened GPS Serial at port={self.gps_port}, baudrate={self.gps_baudrate}')

    
    def find_serial_device_by_vid_pid(self, vid): # finding port using VID 
        ports = serial.tools.list_ports.comports()
        target_port = None
        for port in ports:
            if port.vid == vid:
                target_port = port.device
                break
        return target_port


    def load_target_waypoint(self, lat, long):
        try:
            self.targetLat = float(lat)
            self.targetLong = float(long)
            self.get_logger().info(f'Waypoint {self.targetLat}, {self.targetLong}\nWaiting 10 seconds ...')
            time.sleep(10)

        except Exception as coordinate_exception:
            self.get_logger().info(f'load_target_waypoint: {coordinate_exception}')

    def go_forward(self):
        try:
            twist = Twist(linear=Vector3(x=0.9), angular=Vector3(z=0.0))
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info('forward')
            
        except Exception as e:
            self.get_logger().info(f'go_forward: {e}')

    def rotate_right(self):
        try:
            self.get_logger().info('right')
            if self.distanceToTarget > self.distanceForSteeringRotation:
                twist = Twist(linear=Vector3(x=0.75), angular=Vector3(z=-0.25))
                self.cmd_vel_pub.publish(twist)
            else:
                twist = Twist(linear=Vector3(x=0.0), angular=Vector3(z=-0.25))
                self.cmd_vel_pub.publish(twist)

        except Exception as e:
            self.get_logger().info(f'rotate_right: {e}')

    def rotate_left(self):
        try:
            self.get_logger().info('left')
            if self.distanceToTarget > self.distanceForSteeringRotation:
                twist = Twist(linear=Vector3(x=0.75), angular=Vector3(z=0.25))
                self.cmd_vel_pub.publish(twist)
            else:
                twist = Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.25))
                self.cmd_vel_pub.publish(twist)

        except Exception as e:
            self.get_logger().info(f'rotate_left: {e}')

    def stop_car(self):
        try:
            twist = Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0))
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info('stop')
        except Exception as e:
            self.get_logger().info(f'stop_car: {e}')

    def magnetometer_init(self):
        self.bus.write_byte_data(self.Device_Address, self.Register_A, 0x70)
        self.bus.write_byte_data(self.Device_Address, self.Register_B, 0xa0)
        self.bus.write_byte_data(self.Device_Address, self.Register_mode, 0)

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.Device_Address, addr)
        low = self.bus.read_byte_data(self.Device_Address, addr + 1)
        value = ((high << 8) | low)

        if value > 32768:
            value = value - 65536
        return value

    def read_compass(self):
        try:
            x = self.read_raw_data(self.X_axis_H)
            z = self.read_raw_data(self.Z_axis_H)
            y = self.read_raw_data(self.Y_axis_H)

            heading = math.atan2(y, x) + self.declination

            if heading > 2 * self.pi:
                heading = heading - 2 * self.pi

            if heading < 0:
                heading = heading + 2 * self.pi

            heading_angle = int(heading * 180 / self.pi)

            time.sleep(0.0004)
            return heading_angle

        except Exception as e:
            self.get_logger().info(f'read_compass: {e}')

    def distanceToWaypoint(self):
        try:
            delta = float(math.radians(self.currentLong - self.targetLong))
            sdlong = float(math.sin(delta))
            cdlong = float(math.cos(delta))
            lat1 = float(math.radians(self.currentLat))
            lat2 = float(math.radians(self.targetLat))
            slat1 = float(math.sin(lat1))
            clat1 = float(math.cos(lat1))
            slat2 = float(math.sin(lat2))
            clat2 = float(math.cos(lat2))
            delta = (clat1 * slat2) - (slat1 * clat2 * cdlong)
            delta = (delta * delta)
            delta += (clat2 * sdlong) * (clat2 * sdlong)
            delta = math.sqrt(delta)
            denom = float((slat1 * slat2) + (clat1 * clat2 * cdlong))
            delta = math.atan2(delta, denom)

            # motive of this function
            self.distanceToTarget = delta * 6372795

        except Exception as e:
            self.get_logger().info(f'distance_to_waypoint: {e}')

    def courseToWaypoint(self):
        try:
            dlon = float(math.radians(self.targetLong - self.currentLong))
            cLat = float(math.radians(self.currentLat))
            tLat = float(math.radians(self.targetLat))
            a1 = float(math.sin(dlon) * math.cos(tLat))
            a2 = float(math.sin(cLat) * math.cos(tLat) * math.cos(dlon))
            a2 = math.cos(cLat) * math.sin(tLat) - a2
            a2 = math.atan2(a1, a2)
            if a2 < 0.0:
                a2 += 2 * 3.14159265359

            # motive of this function
            self.targetHeading = math.degrees(a2)

        except Exception as e:
            self.get_logger().info(f'course_to_waypoint: {e}')


    def processGPS(self):
        try:
            self.nmea_msg = self.gps_ser.readline()
            if self.nmea_msg.startswith((b'$GNGGA', b'$GNRMC', b'$GPGGA', b'$GPRMC', b'$GNGLL')):
                self.nmea_msg_str = self.nmea_msg.decode('utf-8')
                self.gps_data = pynmea2.parse(self.nmea_msg_str)
                self.currentLat = float(self.gps_data.latitude)
                self.currentLong = float(self.gps_data.longitude)
                self.nav_sat_msg.latitude = self.currentLat
                self.nav_sat_msg.longitude = self.currentLong
                self.nav_sat_pub.publish(self.nav_sat_msg)

                self.distanceToWaypoint()
                self.courseToWaypoint()
            
        except Exception as e:
            self.get_logger().info(f'error in processGPS(): {e}')



    def navigate(self, publisher, target_lat, target_long):
        
        self.load_target_waypoint(target_lat, target_long)
        self.flag = 0

        try:
            self.magnetometer_init()

            while True:

                while self.gps_ser.in_waiting > 0: # take fresh serial data
                    self.processGPS()

                self.get_logger().info(f'{self.currentLat}, {self.currentLong}')

                self.currentHeading = self.read_compass()
                self.get_logger().info(f'current heading: {self.currentHeading}')

                if self.flag == 1:
                    self.stop_car()
                    self.get_logger().info(f'GPS.navigate() terminates with -> GNSS Reached {target_lat}, {target_long}')
                    break

                elif self.flag == 0:

                    if self.currentLong != 0 and self.targetLong != 0:

                        self.currentHeading = self.read_compass()
                        while abs(self.targetHeading - self.currentHeading) > self.bearingTolerance:

                            if self.currentHeading > self.targetHeading:

                                if self.currentHeading - self.targetHeading > 180:
                                    self.rotate_right()
                                else:
                                    self.rotate_left()
                            else:
                                if self.targetHeading - self.currentHeading > 180:
                                    self.rotate_left()
                                else:
                                    self.rotate_right()

                            if self.distanceToTarget < self.distanceTolerance:
                                self.flag = 1
                                break

                            # if self.distanceToTarget > 1.0:
                            #     ManualControl.sleep_while_avoiding_obstacle_is_on(publisher)
                            
                            # ManualControl.sleep_while_teleop_is_ongoing(publisher)
                            
                            self.currentHeading = self.read_compass()
                            while self.gps_ser.in_waiting > 0: # take fresh serial data
                                self.processGPS()


                        while self.distanceToTarget > self.distanceTolerance: # takes long route || onek shomoy ei loop a thake
                            self.get_logger().info(f'GNSS[{self.current_waypoint}]    |    distance: {self.distanceToTarget}')
                            self.go_forward()
                            
                            while self.gps_ser.in_waiting > 0: # take fresh serial data
                                self.processGPS()

                            if self.distanceToTarget < self.distanceTolerance:
                                self.flag = 1
                                break

                            self.currentHeading = self.read_compass()
                            if abs(self.targetHeading - self.currentHeading) > self.bearingTolerance:
                                self.flag = 0
                                break
                    else:
                        self.stop_car()

        except KeyboardInterrupt:
            self.get_logger().info('Keyboard interrupt. Exiting...')
        except Exception as e:
            self.get_logger().info(f'navigate: {e}')

        # end of navigate()
        return True
