import os
import sys
import time

from xarm.wrapper import XArmAPI

def set_position(position,arm):
  '''
  Toma la posición actual del robot con respecto
  a la esquina inferior izquierda y lo lleva a otra
  posición determinada
  '''
  print("Going to: ", position)
  arm.set_position(x = position[0], y = position[1],wait=True)
  print(arm.get_position())


def grab():
  '''
  Rutina para tomar un objeto
  '''
  pass

def drop():
  '''
  Rutina para dejar un objeto
  '''
  pass
