import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pexpect
import threading
import re

class DeepseekPublisher(Node):
    def __init__(self):
        super().__init__('deepseek_publisher')
        self.publisher_ = self.create_publisher(String, 'llm_response', 10)

        # Inicia el proceso de ollama
        self.child = pexpect.spawn("ollama run deepseek-r1", encoding='utf-8', echo=False)
        self.child.expect(">>>")  # Esperamos hasta que esté listo

        self.get_logger().info("Deepseek iniciado. Puedes escribir tu prompt directamente aquí.")

        # Inicia hilo para manejar entrada y salida
        self.reader_thread = threading.Thread(target=self.chat_loop)
        self.reader_thread.daemon = True
        self.reader_thread.start()

    def chat_loop(self):
        while True:
            try:
                user_input = input("🟢 Tú: ")
                self.child.sendline(user_input)

                # Esperar la respuesta hasta que reaparezca el prompt `>>>`
                self.child.expect(">>>", timeout=60)

                # Extraer solo la respuesta (todo lo que está antes de '>>>')
                response = self.child.before.strip()

                # Limpieza de caracteres ANSI
                response_clean = self.remove_ansi(response)

                # Imprimir y publicar
                print(f"🤖 Deepseek:\n{response_clean}")
                msg = String()
                msg.data = response_clean
                self.publisher_.publish(msg)

            except pexpect.exceptions.TIMEOUT:
                self.get_logger().error("Timeout esperando respuesta del modelo.")
            except Exception as e:
                self.get_logger().error(f"Error general: {str(e)}")
                break

    def remove_ansi(self, text):
        ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
        return ansi_escape.sub('', text)

def main(args=None):
    rclpy.init(args=args)
    node = DeepseekPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

