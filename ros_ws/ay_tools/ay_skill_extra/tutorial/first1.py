#!/usr/bin/python
from core_tool import *
def Help():
  return '''First script.
  Usage: A.first1'''
def Run(ct,*args):
  q= ct.robot.Q()
  print(q)
  q[3]+= 0.2
  print(q)
  ct.robot.MoveToQ(q, 2.0)

