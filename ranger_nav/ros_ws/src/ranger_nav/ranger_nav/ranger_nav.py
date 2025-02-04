import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import board
import busio
from adafruit_pca9685 import PCA9685

class MotorController(Node):
    def __init__(self):
        super().__init__('ranger_nav')

        # Initialize I2C and PCA9685 motor driver
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 60  # Set the PWM frequency to 60 Hz (standard for motors)

        # Define motor channels (PCA9685 has 16 channels, use 4 for motors)
        self.motor_a_forward = 0
        self.motor_a_backward = 1
        self.motor_b_forward = 2
        self.motor_b_backward = 3

        # Create a subscription to the cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x  # Forward speed
        angular_z = msg.angular.z  # Turning speed

        # Map linear and angular velocities to motor speeds and directions
        self.control_motors(linear_x, angular_z)

    def control_motors(self, linear_x, angular_z):
        # Control forward/backward based on linear velocity
        if linear_x > 0:
            self.set_motor_speed(self.motor_a_forward, 1.0)  # Move forward
            self.set_motor_speed(self.motor_a_backward, 0.0)
            self.set_motor_speed(self.motor_b_forward, 1.0)
            self.set_motor_speed(self.motor_b_backward, 0.0)
        elif linear_x < 0:
            self.set_motor_speed(self.motor_a_forward, 0.0)  # Move backward
            self.set_motor_speed(self.motor_a_backward, 1.0)
            self.set_motor_speed(self.motor_b_forward, 0.0)
            self.set_motor_speed(self.motor_b_backward, 1.0)
        else:
            self.set_motor_speed(self.motor_a_forward, 0.0)  # Stop
            self.set_motor_speed(self.motor_a_backward, 0.0)
            self.set_motor_speed(self.motor_b_forward, 0.0)
            self.set_motor_speed(self.motor_b_backward, 0.0)

        # Handle turning by controlling the motor speeds (turning left or right)
        if angular_z > 0:
            # Turn right: slow down left motor and speed up right motor
            self.set_motor_speed(self.motor_a_forward, 0.5)
            self.set_motor_speed(self.motor_a_backward, 0.0)
            self.set_motor_speed(self.motor_b_forward, 1.0)
            self.set_motor_speed(self.motor_b_backward, 0.0)
        elif angular_z < 0:
            # Turn left: slow down right motor and speed up left motor
            self.set_motor_speed(self.motor_a_forward, 1.0)
            self.set_motor_speed(self.motor_a_backward, 0.0)
            self.set_motor_speed(self.motor_b_forward, 0.5)
            self.set_motor_speed(self.motor_b_backward, 0.0)

    def set_motor_speed(self, motor_channel, speed):
        """Set the speed of a motor by adjusting the PWM signal"""
        self.pca.channels[motor_channel].duty_cycle = int(speed * 65535)  # Convert to 16-bit duty cycle

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)

    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
