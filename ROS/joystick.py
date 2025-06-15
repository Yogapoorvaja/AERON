!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickController(Node):
    def _init_(self):
        super()._init_('joystick_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        self.linear_speed = 0.5  # m/s

    def joy_callback(self, msg):
        twist = Twist()
        # Map joystick Y-axis (axes[1], forward/backward) to linear X velocity
        twist.linear.x = -msg.axes[1] * self.linear_speed
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if _name_ == '_main_':
    main()
