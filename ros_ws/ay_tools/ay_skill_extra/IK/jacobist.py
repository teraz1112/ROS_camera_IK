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

def MakeCameraAxis(range,roll,pitch,yaw,euler):
   dis=np.array([0,range,0])
   R=np.linalg.pinv(tft.euler_matrix(roll,pitch,yaw))[:3, :3]
   camera=np.zeros(5)
   camera[2:]=euler[3:]
   p=np.dot(R,euler[:3])+dis
   camera[0]=p[0]/p[1]
   camera[1]=p[2]/p[1]
   return camera


def Run(ct,*args):
   #camera1
   range1=4
   roll1=-np.pi/3
   pitch1=0
   yaw1=0

   g_quat=args[0]
   g_euler=QtoE(g_quat)
   g_camera=MakeCameraAxis(range1,roll1,pitch1,yaw1,g_euler)
   print(g_camera)

#    q=ct.robot.Q()
#    #  # print 'firstq:',q
#    q_traj= []
#    t_traj= []
#    x_camera_traj=[]
#    e_norm_traj=[]
#    q_traj.append(q)
#    t_traj.append(0.0)

#    dt=0.1
#    gain=0.6
 
#    for i in range(1,100):
#       t_traj.append(dt*i)
      
#       x_quat= ct.robot.FK(q)
         
#       x_euler=QtoE(x_quat)

#       x_camera=WtoC(x_euler)

#       x_camera_traj.append(x_camera[:2])    

#       J=np.array(ct.robot.J(q))

#       J_camera=np.zeros((5,7))
#       J_camera[2:]=J[3:]

#       for j in range(7):
#          J_camera[0,j]=(2/(x_euler[1]-np.sqrt(3)*x_euler[2]+8))*J[0,j]
#          +(-2*x_euler[0]/np.square(x_euler[1]-np.sqrt(3)*x_euler[2]+8))*J[1,j]
#          +(2*np.sqrt(3)*x_euler[0]/np.square(-x_euler[1]+np.sqrt(3)*x_euler[2]+8))*J[2,j]

#          J_camera[1,j]=((-4*x_euler[2]+8*np.sqrt(3))/np.square(x_euler[1]-np.sqrt(3)*x_euler[2]+8))*J[1,j]
#          +((4*x_euler[1]+8)/np.square(-x_euler[1]+np.sqrt(3)*x_euler[2]+8))*J[2,j]

#       J_inv = np.linalg.pinv(J_camera)

#       e=g_camera-x_camera

#       e_norm = np.linalg.norm(e)

#       e_norm_traj.append(e_norm)

#       P_e=e*gain

#       vw=np.dot(J_inv,P_e)

      
#       pre_q=q_traj[-1]

#       q=pre_q+vw*dt

#       q_traj.append(q)
      

#       # print 'x_quat:',x_quat
#       # print 'x_euler:',x_euler
#       # print 'J:',J
#       # print 'J_inv:',J_inv
#       # print 'e:',e
#       # print 'P_e:',P_e
#       # print 'vw:',vw
#       # print 'pre_q:',pre_q
#       # print 'q:',q

#    # print (t_traj)
#    # print (q_traj)
   
#    ct.robot.FollowQTraj(q_traj, t_traj)
#    # print(e_norm_traj)
#    # print(x_camera_traj)
#    print(g_camera)
#    print(x_camera)
    