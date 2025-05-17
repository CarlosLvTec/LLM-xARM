# cliente.py
import socket
from llm_testing import translate

SERVER_IP = '192.168.1.181'  # Reemplaza con la IP del servidor
PORT = 12345

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((SERVER_IP, PORT))
    print("Conectado al servidor")
    while True:
        prompt = input("Escribe un mensaje: ")
        mensaje = translate(prompt)
        if mensaje.lower() == 'salir':
            break
        s.sendall(mensaje.encode())
        respuesta = s.recv(1024)
        print("Respuesta del servidor:", respuesta.decode())
