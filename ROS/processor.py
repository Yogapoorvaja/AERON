import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from transformer_model import TransformerModel
import yaml

class NLPProcessor(Node):
    def _init_(self):
        super()._init_('nlp_processor')
        with open('config/aeron_params.yaml', 'r') as f:
            params = yaml.safe_load(f)['aeron']['nlp']
        self.model = TransformerModel(params['vocab_size'], params['embedding_dim'], params['max_seq_len'])
        self.subscription = self.create_subscription(String, '/speech_input', self.process_speech, 10)
        self.publisher = self.create_publisher(String, '/intent', 10)
        self.get_logger().info('NLP Processor Node Started')

    def process_speech(self, msg):
        # Placeholder: Process text with transformer model
        intent = "move_forward"  # Replace with model output
        intent_msg = String()
        intent_msg.data = intent
        self.publisher.publish(intent_msg)
        self.get_logger().info(f'Intent: {intent}')

def main(args=None):
    rclpy.init(args=args)
    node = NLPProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if _name_ == '_main_':
    main()
