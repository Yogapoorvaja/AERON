import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import yaml

class RobotController(Node):
    def _init_(self):
        super()._init_('robot_controller')
        with open('config/aeron_params.yaml', 'r') as f:
            params = yaml.safe_load(f)['aeron']['robot']
        self.publisher = self.create_publisher(Twist, params['cmd_vel_topic'], 10)
        self.subscription = self.create_subscription(String, '/intent', self.handle_intent, 10)
        self.max_speed = params['max_speed']
        self.max_angular_speed = params['max_angular_speed']
        self.get_logger().info('Robot Controller Node Started')

    def handle_intent(self, msg):
        twist = Twist()
        if msg.data == 'move_forward':
            twist.linear.x = self.max_speed
        elif msg.data == 'turn_left':
            twist.angular.z = self.max_angular_speed
        elif msg.data == 'turn_right':
            twist.angular.z = -self.max_angular_speed
        self.publisher.publish(twist)
        self.get_logger().info(f'Command: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if _name_ == '_main_':
    main()
