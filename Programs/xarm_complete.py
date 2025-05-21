from random import randint
from xarm_basic import set_position, grab, drop
from quaroni_vision import get_layout
from xarm.wrapper import XArmAPI

def translate_to_mm(pos):
    pos1 = (319,408)
    pos2 = (325,151)
    dif = pos1[1]-pos2[1]
    mm = 190/dif
    pos_mm = ((pos[1])*1.037, (pos[0]-359)*mm)
    return pos_mm

def move(characteristics, end_position,arm):
  '''
  Is a movement protocol that takes one specific 
  object to a specific position.
  '''
  #n,c,pos = get_layout()

  pos = [(191, 422), (517, 456)]

  c = ['red', 'red', 'red']  

  print(len(pos))

  for i in range (0,len(pos)):
    pos_mm = translate_to_mm(pos[i])
    print(i)
    print(pos_mm)
    if c[i] == characteristics:
    #if True:
      set_position(pos_mm,arm)
      grab()
      print("grab")
      set_position(end_position,arm)
      '''
      El número random es para que no ponga todos los objetos
      en la misma posición, posible rutina para mejorar
      '''
      drop()

  return "Complete"


ip = "192.168.1.173"
arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)


print(move("red",(130,-90),arm))