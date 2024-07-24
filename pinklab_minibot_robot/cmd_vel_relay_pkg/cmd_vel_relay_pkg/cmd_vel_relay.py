import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.get_logger().info('CmdVelRelay node has been started')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.left_motor_pin = 17
        self.right_motor_pin = 18
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_motor_pin, GPIO.OUT)
        GPIO.setup(self.right_motor_pin, GPIO.OUT)
        self.left_pwm = GPIO.PWM(self.left_motor_pin, 100)
        self.right_pwm = GPIO.PWM(self.right_motor_pin, 100)
        self.left_pwm.start(0)
        self.right_pwm.start(0)

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        left_speed = linear - angular
        right_speed = linear + angular
        self.set_motor_speed(self.left_pwm, left_speed)
        self.set_motor_speed(self.right_pwm, right_speed)
        self.get_logger().info(f'Received cmd_vel: linear={linear} angular={angular}')

    def set_motor_speed(self, pwm, speed):
        duty_cycle = max(min(speed * 100, 100), 0)
        pwm.ChangeDutyCycle(duty_cycle)

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_relay = CmdVelRelay()
    rclpy.spin(cmd_vel_relay)
    cmd_vel_relay.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
