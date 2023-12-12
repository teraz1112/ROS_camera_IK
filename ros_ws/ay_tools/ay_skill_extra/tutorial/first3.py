#!/usr/bin/python
from core_tool import *
import math
import copy
import numpy as np
def Help():
  return '''First script.
  Usage: A.first3'''

def Run(ct,*args):
  q= ct.robot.Q()
  q_traj= []
  t_traj= []
  q_traj.append(q)
  t_traj.append(0.0)
  for i in range(1,50):
      t_traj.append(0.1*i)
      q2= copy.deepcopy(q)
      q2[2]+= 0.2*math.sin(3.0*0.1*i)
      q_traj.append(np.array(q2))
  print (t_traj)
  print (q_traj)
  ct.robot.FollowQTraj(q_traj, t_traj)