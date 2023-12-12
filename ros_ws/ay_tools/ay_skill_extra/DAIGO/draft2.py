#!/usr/bin/python
from core_tool import *
def Help():
  return '''First script.
  Usage: IK'''
def Run(ct,*args):
  x= ct.robot.FK()
  y=args[0]
  print(x)
  print(y)
  x[0]+=y[0]
  x[1]+=y[1]
  x[2]+=y[2]
  print(x)
  q_fin=ct.robot.IK(x)
  print(q_fin)
  ct.robot.MoveToQ(q_fin, 0.2)