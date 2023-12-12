#!/usr/bin/python
from core_tool import *
def Help():
  return '''First script.
  Usage: A.first2'''

def Run(ct,*args):
  x= ct.robot.FK()
  print (x)
  for i in range(4):
    x[0]+= 0.04
    x[2]+= 0.04
    ct.robot.MoveToX(x, 0.2, blocking=True)
    x[0]-= 0.04
    x[2]-= 0.04
    ct.robot.MoveToX(x, 0.2, blocking=True)
