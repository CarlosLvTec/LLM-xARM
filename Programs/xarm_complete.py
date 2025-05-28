from random import randint
from xarm_basic import set_position, grab, drop
from xarm.wrapper import XArmAPI
from ollama import chat
from ollama import ChatResponse
import pickle
import socket

def translate_to_mm(pos):
    
    with open("/home/teccem/llm_xarm/calibration_values.txt", "rb") as file:
      calibration_values = pickle.load(file)
      file.close()
    
    robot = 270
    #robot - valor de Y del centro del workspace
    dif = robot - 224 * abs(calibration_values[1][4])
    x,y = pos
    #height = int(frame.shape[0])
    #width = int(frame.shape[1])

    height = 480
    width = 640

    if y > height/3:
      if y > height*2/3:
        qy = [1,2,3]
      else:
        qy = [4,5,6]
    else:
      qy = [7,8,9]
    if x > width/3:
      if x > width*2/3:
        qx = [3,6,9]
      else:
        qx = [2,5,8]
    else:
      qx = [1,4,7]

    for i in range(3):
      if qx[i] in qy:
        Q = i
    #A el valor de x restar el valor del centro del robot(Entre 4 esquinas de hojas)
    pos_mm = (y*abs(calibration_values[1][Q])+dif, (x-(323))*abs(calibration_values[0][Q]))
    return pos_mm

def move(characteristics, end_position,arm,layout):
  '''
  Is a movement protocol that takes one specific 
  object to a specific position.
  '''
  #n,c,pos = get_layout()
  end_position = translate_end_position(end_position)
  arm.move_gohome(wait=True)

  #pos = [(157, 133), (175, 318), (245, 197), (286, 76), (328, 229), (331, 397), (426, 306), (435, 148)]
  #c = ['red', 'red', 'green', 'red', 'red', 'green', 'red', 'green']
  pos, c = layout

  for i in range (0,len(pos)):
    pos_mm = translate_to_mm(pos[i])
    if c[i] == characteristics:
    #if True:
      set_position(pos_mm,arm)
      grab(arm)
      print("grab")
      set_position(end_position,arm)
      '''
      El número random es para que no ponga todos los objetos
      en la misma posición, posible rutina para mejorar
      '''
      drop(arm)
  arm.move_gohome(wait=True)
  return "Complete" 

def translate_end_position(given):
  if given in "ToptopTOP":
    end_position = (randint(300,370),randint(-50,50))
  if given in "BottombottomBOTTOM":
    end_position = (randint(150,225),randint(-50,50))
  if given in "LEFTleftLeft":
    end_position = (randint(230,300),randint(120,220))
  if given in "rightRIGHTRight":
    end_position = (randint(230,300),randint(-220,-120))
  return end_position






