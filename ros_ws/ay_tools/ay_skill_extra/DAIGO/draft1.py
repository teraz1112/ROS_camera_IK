#!/usr/bin/python
from core_tool import *
def Help():
  return '''First script.
  Usage: FK'''
def Run(ct,*args):
  x_trg= args[0]
  dt= args[1] if len(args)>1 else 4.0
  x= list(ct.robot.FK())
  if len(x_trg)==3:  x_trg= list(x_trg)+x[3:]
  elif len(x_trg)==4:  x_trg= x[:3]+list(x_trg)
  assert(len(x_trg)==7)
  print ('Move to x:',x_trg)
  ct.robot.MoveToX(x_trg, dt)
  # x=args
  # ct.robot.MoveToX(x,2.0,blocking=True)