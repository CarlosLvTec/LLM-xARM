# cliente.py
import socket

SERVER_IP = '192.168.1.200'  # Reemplaza con la IP del servidor
PORT = 12345

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((SERVER_IP, PORT))
    print("Conectado al servidor")
    while True:
        mensaje = input("Escribe un mensaje: ")
        if mensaje.lower() == 'salir':
            break
        s.sendall(mensaje.encode())
        respuesta = s.recv(1024)
        print("Respuesta del servidor:", respuesta.decode())
