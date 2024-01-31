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

def EtoM(roll,pitch,yaw):
   R=np.linalg.pinv(tft.euler_matrix(roll,pitch,yaw))[:3, :3]
   return R

def MakeCameraAxis(range,roll,pitch,yaw,euler):
   dis=np.array([0,range,0])
   R=EtoM(roll,pitch,yaw)
   camera=np.zeros(5)
   camera[2:]=euler[3:]
   p=np.dot(R,euler[:3])+dis
   camera[0]=p[0]/p[1]
   camera[1]=p[2]/p[1]
   return camera

def make1raw(a,b,c):
   x=[]
   x.append(a[0])
   x.append(a[1])
   x.append(b[0])
   x.append(b[1])
   x.append(c[3])
   x.append(c[4])
   x.append(c[5])
   return np.array(x)


def Run(ct,*args):
   #camera1
   range1=4
   roll1=0
   pitch1=0
   yaw1=0
   R1=EtoM(roll1,pitch1,yaw1)
   # print(R1)

   #camera2
   range2=4
   roll2=0
   pitch2=0
   yaw2=np.pi/2
   R2=EtoM(roll2,pitch2,yaw2)
   # print(R2)

   #camera_miss
   range_miss=range1
   roll_miss=roll1
   pitch_miss=pitch1+np.pi/6
   yaw_miss=yaw1
   R_miss=EtoM(roll_miss,pitch_miss,yaw_miss)
   # print(R2)


   g_quat=args[0]

   g_tp=np.array(g_quat[:3])

   g_euler=QtoE(g_quat)
   

   g_camera1=MakeCameraAxis(range1,roll1,pitch1,yaw1,g_euler)
   g_camera2=MakeCameraAxis(range2,roll2,pitch2,yaw2,g_euler)
   g_camera_miss=MakeCameraAxis(range_miss,roll_miss,pitch_miss,yaw_miss,g_euler)
   g=make1raw(g_camera1,g_camera2,g_euler)
   g_miss=make1raw(g_camera_miss,g_camera2,g_euler)
   print(g)

#    print(g_camera1)
#    print(g_camera2)
#    print(g_camera_miss)

   q=ct.robot.Q()
   #  # print 'firstq:',q
   q_traj= []
   t_traj= []
   x_camera_traj=[]
   e_norm_traj=[]
   e_norm_3d_traj=[]
   q_traj.append(q)
   t_traj.append(0.0)

   dt=0.1
   gain=0.6


 
   for i in range(1,200):
      t_traj.append(dt*i)
      
      x_quat= ct.robot.FK(q)
      x_tp=np.array(x_quat[:3])
      e_3d=g_tp-x_tp
      e_norm_3d = np.linalg.norm(e_3d)
      e_norm_3d_traj.append(e_norm_3d)

      x_euler=QtoE(x_quat)

      x_camera1=MakeCameraAxis(range1,roll1,pitch1,yaw1,x_euler)
      x_camera2=MakeCameraAxis(range2,roll2,pitch2,yaw2,x_euler)
      x_camera_miss=MakeCameraAxis(range_miss,roll_miss,pitch_miss,yaw_miss,x_euler)
      x=make1raw(x_camera1,x_camera2,x_euler)
      x_miss=make1raw(x_camera_miss,x_camera2,x_euler)
      x_camera_traj.append(x[:4])
      # print(x)

      
    #   print(x_camera1)
    #   print(x_camera2)
    #   print(x_camera_miss)

      J=np.array(ct.robot.J(q))
      # print(J)



      J_camera=np.zeros((7,7))
      J_camera[4:]=J[3:]
      # print(J_camera)

      for j in range(7):
         J_camera[0,j]=((R1[0,0]*(R1[1,0]*x_euler[0]+R1[2,1]*x_euler[1]+R1[1,2]*x_euler[2]+range1)-R1[1,0]*(R1[0,0]*x_euler[0]+R1[0,1]*x_euler[1]+R1[0,2]*x_euler[2]))/np.square(R1[1,0]*x_euler[0]+R1[1,1]*x_euler[1]+R1[1,2]*x_euler[2]+range1))*J[0,j]+((R1[0,1]*(R1[1,0]*x_euler[0]+R1[2,1]*x_euler[1]+R1[1,2]*x_euler[2]+range1)-R1[1,1]*(R1[0,0]*x_euler[0]+R1[0,1]*x_euler[1]+R1[0,2]*x_euler[2]))/np.square(R1[1,0]*x_euler[0]+R1[1,1]*x_euler[1]+R1[1,2]*x_euler[2]+range1))*J[1,j]+((R1[0,2]*(R1[1,0]*x_euler[0]+R1[2,1]*x_euler[1]+R1[1,2]*x_euler[2]+range1)-R1[1,2]*(R1[0,0]*x_euler[0]+R1[0,1]*x_euler[1]+R1[0,2]*x_euler[2]))/np.square(R1[1,0]*x_euler[0]+R1[1,1]*x_euler[1]+R1[1,2]*x_euler[2]+range1))*J[2,j]
         
         J_camera[1,j]=((R1[2,0]*(R1[1,0]*x_euler[0]+R1[2,1]*x_euler[1]+R1[1,2]*x_euler[2]+range1)-R1[1,0]*(R1[2,0]*x_euler[0]+R1[2,1]*x_euler[1]+R1[2,2]*x_euler[2]))/np.square(R1[1,0]*x_euler[0]+R1[1,1]*x_euler[1]+R1[1,2]*x_euler[2]+range1))*J[0,j]+((R1[2,1]*(R1[1,0]*x_euler[0]+R1[2,1]*x_euler[1]+R1[1,2]*x_euler[2]+range1)-R1[1,1]*(R1[2,0]*x_euler[0]+R1[2,1]*x_euler[1]+R1[2,2]*x_euler[2]))/np.square(R1[1,0]*x_euler[0]+R1[1,1]*x_euler[1]+R1[1,2]*x_euler[2]+range1))*J[1,j]+((R1[2,2]*(R1[1,0]*x_euler[0]+R1[2,1]*x_euler[1]+R1[1,2]*x_euler[2]+range1)-R1[1,2]*(R1[2,0]*x_euler[0]+R1[2,1]*x_euler[1]+R1[2,2]*x_euler[2]))/np.square(R1[1,0]*x_euler[0]+R1[1,1]*x_euler[1]+R1[1,2]*x_euler[2]+range1))*J[2,j]
         
         
         J_camera[2,j]=((R2[0,0]*(R2[1,0]*x_euler[0]+R2[2,1]*x_euler[1]+R2[1,2]*x_euler[2]+range2)-R2[1,0]*(R2[0,0]*x_euler[0]+R2[0,1]*x_euler[1]+R2[0,2]*x_euler[2]))/np.square(R2[1,0]*x_euler[0]+R2[1,1]*x_euler[1]+R2[1,2]*x_euler[2]+range2))*J[0,j]+((R2[0,1]*(R2[1,0]*x_euler[0]+R2[2,1]*x_euler[1]+R2[1,2]*x_euler[2]+range2)-R2[1,1]*(R2[0,0]*x_euler[0]+R2[0,1]*x_euler[1]+R2[0,2]*x_euler[2]))/np.square(R2[1,0]*x_euler[0]+R2[1,1]*x_euler[1]+R2[1,2]*x_euler[2]+range2))*J[1,j]+((R2[0,2]*(R2[1,0]*x_euler[0]+R2[2,1]*x_euler[1]+R2[1,2]*x_euler[2]+range2)-R2[1,2]*(R2[0,0]*x_euler[0]+R2[0,1]*x_euler[1]+R2[0,2]*x_euler[2]))/np.square(R2[1,0]*x_euler[0]+R2[1,1]*x_euler[1]+R2[1,2]*x_euler[2]+range2))*J[2,j]

         J_camera[3,j]=((R2[2,0]*(R2[1,0]*x_euler[0]+R2[2,1]*x_euler[1]+R2[1,2]*x_euler[2]+range2)-R2[1,0]*(R2[2,0]*x_euler[0]+R2[2,1]*x_euler[1]+R2[2,2]*x_euler[2]))/np.square(R2[1,0]*x_euler[0]+R2[1,1]*x_euler[1]+R2[1,2]*x_euler[2]+range2))*J[0,j]+((R2[2,1]*(R2[1,0]*x_euler[0]+R2[2,1]*x_euler[1]+R2[1,2]*x_euler[2]+range2)-R2[1,1]*(R2[2,0]*x_euler[0]+R2[2,1]*x_euler[1]+R2[2,2]*x_euler[2]))/np.square(R2[1,0]*x_euler[0]+R2[1,1]*x_euler[1]+R2[1,2]*x_euler[2]+range2))*J[1,j]+((R2[2,2]*(R2[1,0]*x_euler[0]+R2[2,1]*x_euler[1]+R2[1,2]*x_euler[2]+range2)-R2[1,2]*(R2[2,0]*x_euler[0]+R2[2,1]*x_euler[1]+R2[2,2]*x_euler[2]))/np.square(R2[1,0]*x_euler[0]+R2[1,1]*x_euler[1]+R2[1,2]*x_euler[2]+range2))*J[2,j]
   
      # print(J_camera)

      J_inv = np.linalg.pinv(J_camera)
      # print(J_inv)

      e=g-x_miss

      e_norm = np.linalg.norm(e)

      e_norm_traj.append(e_norm)

      P_e=e*gain

      vw=np.dot(J_inv,P_e)
      # print(vw)


      pre_q=q_traj[-1]

      q=pre_q+vw*dt

      q_traj.append(q)
      

      # print 'x_quat:',x_quat
      # print 'x_euler:',x_euler
      # print 'J:',J
      # print 'J_inv:',J_inv
      # print 'e:',e
      # print 'P_e:',P_e
      # print 'vw:',vw
      # print 'pre_q:',pre_q
      # print 'q:',q

   # print (t_traj)
   # print (q_traj)
   
   ct.robot.FollowQTraj(q_traj, t_traj)
#    print(x_camera1)
#    print(x_camera2)
#    print(x_camera_miss)
#    print(e_norm_traj)
#    print(e_norm_3d_traj)
#    print(x_camera_traj)
   print(g)
   print(x)
   print(g_quat)
   print(x_quat)