# servidor.py
import socket

HOST = ''  # Escucha en todas las interfaces disponibles
PORT = 12345  # Puerto arbitrario no privilegiado (>1024)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print(f"Servidor escuchando en el puerto {PORT}...")
    
    conn, addr = s.accept()
    with conn:
        print(f"Conexión desde {addr}")
        while True:
            data = conn.recv(1024)
            if not data:
                break
            print("Mensaje recibido:", data.decode())
            conn.sendall(b"Mensaje recibido")
