#!/usr/bin/python
from core_tool import *
def Help():
  return '''First script.
  Usage: IK'''
def Run(ct,*args):
  a=ct.robot.GripperRange()
  print(a)
  a=ct.robot.EndEff()
  print(a)
  a=ct.robot.DQ()
  print(a)
  a=ct.robot.J()
  print(a)
  a=ct.robot.()
  print(a)