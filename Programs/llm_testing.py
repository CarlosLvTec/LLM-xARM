from ollama import chat
from ollama import ChatResponse
from xarm_complete import move
from xarm.wrapper import XArmAPI
import socket
import ast

def translate(text):
    response: ChatResponse = chat(model='carken_UC', messages=[
    {
        'role': 'user',
        'content': text,
    },
    ])

    real = response['message']['content'].split("</think>")
    print(real[0])
    return(real[0])
# or access fields directly from the response object

def llm_to_xarm(text):

    '''
    RESPONSE:
        <Protocol_1>
        ["action_1"]
        ["red"]
        ["top"]
        </Protocol_1>
    '''
    response = text.split('\n')
    actions = []
    characteristics = []
    end_pos = []

    for i in range(len(response)):
        if "ction_1" in response[i]:
            actions.append("move")
            characteristics.append(response[i+1][response[i+2].find('"')+1:len(response[i+1])-2])
            end_pos.append(response[i+2][response[i+2].find('"')+1:len(response[i+2])-2])

    #print(response)
    print(actions)
    print(characteristics)
    print(end_pos)

    return actions, characteristics, end_pos


ip = "192.168.1.173"
arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
arm.move_gohome(wait=True)

SERVER_IP = '192.168.1.200'  # Reemplaza con la IP del servidor
PORT = 12345

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((SERVER_IP, PORT))
    print("Conectado al servidor")
    while True:
        actions, characteristics, end_pos = llm_to_xarm(translate(input(">>> ")))

        mensaje = "Activate"

        s.sendall(mensaje.encode())
        respuesta = s.recv(1024)
        print("Respuesta del servidor:", respuesta.decode())
        string = respuesta.decode()
        layout = ast.literal_eval(string)
        for i in range(len(actions)):
            if actions[i] == 'move':
                move(characteristics[i],end_pos[i],arm,layout)

            
