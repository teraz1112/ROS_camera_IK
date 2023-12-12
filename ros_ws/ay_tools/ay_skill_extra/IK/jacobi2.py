#!/usr/bin/python
from core_tool import *
import math
import copy
import numpy as np
import tf.transformations as tft

def Help():
  return '''First script.
  Usage: A.first4'''

def QtoE(FK):
   quat=np.array(FK[3:])
   euler= np.array(tft.euler_from_quaternion(quat))
   FK2= copy.deepcopy(FK)
   FK2[3:7]=euler
   return np.array(FK2)

def WtoC(world_array):
   world=np.array(world_array[:3])
   camera=np.zeros(5)
   camera[2:]=world_array[3:]
   camera[0]=(2*world[0])/(-world[1]+np.sqrt(3)*world[2]+8)
   camera[1]=(np.sqrt(3)*world[1]+world[2])/(-world[1]+np.sqrt(3)*world[2]+8)
   return np.array(camera)

def Run(ct,*args):
    g_quat=args[0]
    g_euler=QtoE(g_quat)
    g_camera=WtoC(g_euler)
    print(g_quat)
    print(g_euler)
    print(g_camera)

    # q=ct.robot.Q()
    # # print 'firstq:',q
    # q_traj= []
    # t_traj= []
    # q_traj.append(q)
    # t_traj.append(0.0)

    # dt=0.01
    
    # for i in range(1,500):
    #     t_traj.append(dt*i)
        
    #     x_quat= ct.robot.FK(q)
        
    #     x_euler=QtoE(x_quat)
        

    #     J=np.array(ct.robot.J(q))
        
        
    #     J_inv = np.linalg.pinv(J)
        

    #     e=g_euler-x_euler
        
    #     P_e=e*0.1
        
    #     vw=np.dot(J_inv,P_e)
        
    #     pre_q=q_traj[-1]
    #     q=pre_q+vw*dt

    #     q_traj.append(q)
        

    #     # print 'x_quat:',x_quat
    #     # print 'x_euler:',x_euler
    #     # print 'J:',J
    #     # print 'J_inv:',J_inv
    #     # print 'e:',e
    #     # print 'P_e:',P_e
    #     # print 'vw:',vw
    #     # print 'pre_q:',pre_q
    #     # print 'q:',q

    # # print (t_traj)
    # # print (q_traj)
    # ct.robot.FollowQTraj(q_traj, t_traj)
    