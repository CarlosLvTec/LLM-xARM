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


def grab(arm):
  '''
  Rutina para tomar un objeto [90]
  '''
  z_actual = arm.get_position()
  z_actual = z_actual[1][2]
  #arm.set_suction_cup(True)
  arm.set_position(z = 86,wait=True)
  arm.set_position(z = z_actual, wait=True)

def drop(arm):
  '''
  Rutina para dejar un objeto
  '''
  z_actual = arm.get_position()
  z_actual = z_actual[1][2]
  #No baja pq choca revisar si hay sensor
  #arm.set_position(z = 90,wait=True)
  arm.set_suction_cup(False)
  arm.set_position(z = z_actual, wait=True)
