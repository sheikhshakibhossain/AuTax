import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

# GPIO pin configuration for L298N motor driver
# Motor A (Left Motor)
LEFT_MOTOR_FORWARD_PIN = 11
LEFT_MOTOR_BACKWARD_PIN = 12

# Motor B (Right Motor)
RIGHT_MOTOR_FORWARD_PIN = 13
RIGHT_MOTOR_BACKWARD_PIN = 17

# Set GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(LEFT_MOTOR_FORWARD_PIN, GPIO.OUT)
GPIO.setup(LEFT_MOTOR_BACKWARD_PIN, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_FORWARD_PIN, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_BACKWARD_PIN, GPIO.OUT)

class MotorController(Node):

    def __init__(self):
        super().__init__('drive_node')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription
        self.get_logger().info('Drive node initialized.')



    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Calculate motor speeds
        left_motor_speed = linear_x - angular_z
        right_motor_speed = linear_x + angular_z

        # Drive the motors and print the state
        self.drive_motors(left_motor_speed, right_motor_speed)

    def drive_motors(self, left_speed, right_speed):
        # Determine motion state based on motor speeds and print appropriate messages
        if left_speed > 0 and right_speed > 0:
            self.get_logger().info('Moving Forward')
        elif left_speed < 0 and right_speed < 0:
            self.get_logger().info('Moving Backward')
        elif left_speed > right_speed:
            self.get_logger().info('Turning Left')
        elif right_speed > left_speed:
            self.get_logger().info('Turning Right')
        elif left_speed == 0 and right_speed == 0:
            self.get_logger().info('Stopping')

        # Left motor control
        if left_speed > 0:
            GPIO.output(LEFT_MOTOR_FORWARD_PIN, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_BACKWARD_PIN, GPIO.LOW)
        elif left_speed < 0:
            GPIO.output(LEFT_MOTOR_FORWARD_PIN, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_BACKWARD_PIN, GPIO.HIGH)
        else:
            GPIO.output(LEFT_MOTOR_FORWARD_PIN, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_BACKWARD_PIN, GPIO.LOW)

        # Right motor control
        if right_speed > 0:
            GPIO.output(RIGHT_MOTOR_FORWARD_PIN, GPIO.HIGH)
            GPIO.output(RIGHT_MOTOR_BACKWARD_PIN, GPIO.LOW)
        elif right_speed < 0:
            GPIO.output(RIGHT_MOTOR_FORWARD_PIN, GPIO.LOW)
            GPIO.output(RIGHT_MOTOR_BACKWARD_PIN, GPIO.HIGH)
        else:
            GPIO.output(RIGHT_MOTOR_FORWARD_PIN, GPIO.LOW)
            GPIO.output(RIGHT_MOTOR_BACKWARD_PIN, GPIO.LOW)

def main(args=None):
    rclpy.init(args=args)

    motor_controller = MotorController()

    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass

    # Cleanup on shutdown
    motor_controller.get_logger().info('Shutting down motor controller')
    motor_controller.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()
