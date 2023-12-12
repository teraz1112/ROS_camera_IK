#!/usr/bin/python
from core_tool import *
import math
import copy
import numpy as np
def Help():
  return '''First script.
  Usage: A.first4'''

def Run(ct,*args):
    r= 0.03
    x= ct.robot.FK()
    x1= copy.deepcopy(x)
    x1[1]-= r
    ct.robot.MoveToX(x1, 1.0, blocking=True)
    x_traj= []
    t_traj= []
    for t in np.arange(0,2*math.pi,0.1):
        t_traj.append(t)
        x2= copy.deepcopy(x)
        x2[1]+= -r*math.cos(1.0*t)
        x2[2]+= r*math.sin(1.0*t)
        x_traj.append(x2)
    print (t_traj)
    print (x_traj)
    ct.robot.FollowXTraj(x_traj, t_traj)