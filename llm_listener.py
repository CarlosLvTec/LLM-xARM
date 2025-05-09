# llm_listener.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LLMListener(Node):
    def __init__(self):
        super().__init__('llm_listener')
        self.subscription = self.create_subscription(
            String,
            'llm_response',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        print(f"[LLM Listener] Recibido: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = LLMListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

