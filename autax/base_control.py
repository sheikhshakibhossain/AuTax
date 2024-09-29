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

# PWM frequency (in Hz)
PWM_FREQ = 100

# Set GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(LEFT_MOTOR_FORWARD_PIN, GPIO.OUT)
GPIO.setup(LEFT_MOTOR_BACKWARD_PIN, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_FORWARD_PIN, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_BACKWARD_PIN, GPIO.OUT)

# Initialize PWM for each motor pin
left_motor_forward_pwm = GPIO.PWM(LEFT_MOTOR_FORWARD_PIN, PWM_FREQ)
left_motor_backward_pwm = GPIO.PWM(LEFT_MOTOR_BACKWARD_PIN, PWM_FREQ)
right_motor_forward_pwm = GPIO.PWM(RIGHT_MOTOR_FORWARD_PIN, PWM_FREQ)
right_motor_backward_pwm = GPIO.PWM(RIGHT_MOTOR_BACKWARD_PIN, PWM_FREQ)

# Start the PWM with 0 duty cycle (motors off)
left_motor_forward_pwm.start(0)
left_motor_backward_pwm.start(0)
right_motor_forward_pwm.start(0)
right_motor_backward_pwm.start(0)

class MotorController(Node):

    def __init__(self):
        super().__init__('base_control')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription
        self.get_logger().info('base_control node initialized.')

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Calculate motor speeds
        left_motor_speed = linear_x - angular_z
        right_motor_speed = linear_x + angular_z

        # Drive the motors and print the state
        self.drive_motors(left_motor_speed, right_motor_speed)

    def drive_motors(self, left_speed, right_speed):
        # Normalize the speed values (-1 to 1) to a duty cycle (0 to 100)
        left_duty_cycle = min(max(abs(left_speed) * 100, 0), 100)
        right_duty_cycle = min(max(abs(right_speed) * 100, 0), 100)

        # Determine motion state based on motor speeds and print appropriate messages
        if left_speed > 0 and right_speed > 0:
            self.get_logger().info('Moving Forward')
        elif left_speed < 0 and right_speed < 0:
            self.get_logger().info('Moving Backward')
        elif left_speed > right_speed:
            self.get_logger().info('Turning Right')
        elif right_speed > left_speed:
            self.get_logger().info('Turning Left')
        elif left_speed == 0 and right_speed == 0:
            self.get_logger().info('Stopping')

        # Left motor control
        if left_speed > 0:
            left_motor_forward_pwm.ChangeDutyCycle(left_duty_cycle)
            left_motor_backward_pwm.ChangeDutyCycle(0)
        elif left_speed < 0:
            left_motor_forward_pwm.ChangeDutyCycle(0)
            left_motor_backward_pwm.ChangeDutyCycle(left_duty_cycle)
        else:
            left_motor_forward_pwm.ChangeDutyCycle(0)
            left_motor_backward_pwm.ChangeDutyCycle(0)

        # Right motor control
        if right_speed > 0:
            right_motor_forward_pwm.ChangeDutyCycle(right_duty_cycle)
            right_motor_backward_pwm.ChangeDutyCycle(0)
        elif right_speed < 0:
            right_motor_forward_pwm.ChangeDutyCycle(0)
            right_motor_backward_pwm.ChangeDutyCycle(right_duty_cycle)
        else:
            right_motor_forward_pwm.ChangeDutyCycle(0)
            right_motor_backward_pwm.ChangeDutyCycle(0)

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