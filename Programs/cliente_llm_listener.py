# llm_listener.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket

SERVER_IP = '192.168.1.181'  # Cambia por la IP real del servidor
PORT = 12345

class LLMListener(Node):
    def __init__(self):
        super().__init__('llm_listener')
        self.subscription = self.create_subscription(
            String,
            'llm_response',
            self.listener_callback,
            10
        )
        self.socket = None
        self.connect_to_server()

    def connect_to_server(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((SERVER_IP, PORT))
            print(f"[✓] Conectado al servidor: {SERVER_IP}:{PORT}")
        except Exception as e:
            print(f"[✗] Error al conectar con el servidor: {e}")
            self.socket = None

    def listener_callback(self, msg):
        message = msg.data.strip()

        # Solo enviar si hay texto válido
        if not message:
            print("[!] Mensaje vacío recibido, ignorado.")
            return

        # Mostrar lo recibido
        print(f"[📥] Mensaje recibido del LLM: {message}")

        # Intentar enviar por socket
        if self.socket:
            try:
                self.socket.sendall(message.encode())
                print(f"[📤] Enviado al servidor: {message}")
            except Exception as e:
                print(f"[✗] Error al enviar por socket: {e}")
                self.socket.close()
                self.socket = None
        else:
            print("[!] Socket no disponible, intentando reconectar...")
            self.connect_to_server()

def main(args=None):
    rclpy.init(args=args)
    node = LLMListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[🛑] Interrumpido por el usuario.")
    finally:
        if node.socket:
            node.socket.close()
            print("[✓] Socket cerrado correctamente.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
