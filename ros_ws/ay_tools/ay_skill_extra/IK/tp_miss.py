# -*- coding: utf-8 -*-
#!/usr/bin/python
from core_tool import *
import math
import copy
import numpy as np
import tf.transformations as tft
import sympy as sp
from tqdm import tqdm


def Help():
  return '''First script.
  Usage: A.first4'''

def QtoE(FK):
   quat=np.array(FK[3:])
   euler= np.array(tft.euler_from_quaternion(quat))
   FK2= copy.deepcopy(FK)
   FK2[3:7]=euler
   return np.array(FK2)

def QtoM(quat):
   R=tft.quaternion_matrix(quat)*0.2
   return R

def WtoC(world_array,R,L):
   world=world_array
   camera=np.zeros(2)
   camera[0]=(R[0,0]*(world[0])+R[0,1]*(world[1])+R[0,2]*(world[2]))/(R[1,0]*(world[0])+R[1,1]*(world[1])+R[1,2]*(world[2])+L)
   camera[1]=(R[2,0]*(world[0])+R[2,1]*(world[1])+R[2,2]*(world[2]))/(R[1,0]*(world[0])+R[1,1]*(world[1])+R[1,2]*(world[2])+L)
   return np.array(camera)

def CulP(R0,euler):
   p1=copy.deepcopy(euler)
   p2=copy.deepcopy(euler)
   p3=copy.deepcopy(euler)
   p1[0]+=R0[0,0]*10
   p1[1]+=R0[1,0]*10
   p1[2]+=R0[2,0]*10
   p2[0]+=R0[0,1]*10
   p2[1]+=R0[1,1]*10
   p2[2]+=R0[2,1]*10
   p3[0]+=R0[0,2]*10
   p3[1]+=R0[1,2]*10
   p3[2]+=R0[2,2]*10
   return p1,p2,p3

def EtoM(roll,pitch,yaw):
   R=np.linalg.pinv(tft.euler_matrix(roll,pitch,yaw))[:3, :3]
   return R

def MakeCameraAxis(range,roll,pitch,yaw,euler):
   dis=np.array(range)
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
   x.append(c[0])
   x.append(c[1])
   return np.array(x)

def Run(ct,*args):
    #camera1
    t=[0,4,0]
    roll1=-np.pi/4
    pitch1=0
    yaw1=0
    R=EtoM(roll1,pitch1,yaw1)

    #camera_miss
    t_miss=[0.1,4,0.02]
    roll1_miss=-np.pi/3
    pitch1_miss=0.01
    yaw1_miss=0.02
    R_miss=EtoM(roll1_miss,pitch1_miss,yaw1_miss)

    g_quat=args[0]
    g_tp=np.array(g_quat[:3])
    R0=QtoM(g_quat[3:])
    g_euler=QtoE(g_quat)
    g1,g2,g3=CulP(R0,g_euler)
    g_camera1=MakeCameraAxis(t,roll1,pitch1,yaw1,g1)
    g_camera2=MakeCameraAxis(t,roll1,pitch1,yaw1,g2)
    g_camera3=MakeCameraAxis(t,roll1,pitch1,yaw1,g3)
    g_camera1_miss=MakeCameraAxis(t_miss,roll1_miss,pitch1_miss,yaw1_miss,g1)
    g_camera2_miss=MakeCameraAxis(t_miss,roll1_miss,pitch1_miss,yaw1_miss,g2)
    g_camera3_miss=MakeCameraAxis(t_miss,roll1_miss,pitch1_miss,yaw1_miss,g3)

    g=make1raw(g_camera1,g_camera2,g_camera3)
    g_miss=make1raw(g_camera1_miss,g_camera2_miss,g_camera3_miss)
    # print(g)
    # print(g_miss)

   #  print(g_quat)
    # print(g_euler)
    #    print(g_camera)
    #    print(g_camera1)
    #    print(g_camera2)
    #    print(g_camera3)


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
    gain=0.3
    
 
    for i in tqdm(range(500), desc="Processing", unit="iteration"):
        t_traj.append(dt*(i+1))
        
        x_quat= ct.robot.FK(q)
        x_tp=np.array(x_quat[:3])
        e_3d=g_tp-x_tp
        e_norm_3d = np.linalg.norm(e_3d)
        e_norm_3d_traj.append(e_norm_3d)
            
        x_euler=QtoE(x_quat)

        Rq=QtoM(x_quat[3:])

        x1,x2,x3=CulP(Rq,x_euler)

        x_camera1=MakeCameraAxis(t,roll1,pitch1,yaw1,x1)
        x_camera2=MakeCameraAxis(t,roll1,pitch1,yaw1,x2)
        x_camera3=MakeCameraAxis(t,roll1,pitch1,yaw1,x3)
        x_camera1_miss=MakeCameraAxis(t_miss,roll1_miss,pitch1_miss,yaw1_miss,x1)
        x_camera2_miss=MakeCameraAxis(t_miss,roll1_miss,pitch1_miss,yaw1_miss,x2)
        x_camera3_miss=MakeCameraAxis(t_miss,roll1_miss,pitch1_miss,yaw1_miss,x3)
        x=make1raw(x_camera1,x_camera2,x_camera3)
        x_miss=make1raw(x_camera1_miss,x_camera2_miss,x_camera3_miss)
        # print(x)

        x_camera_traj.append(x)    

        J=np.array(ct.robot.J(q))

        J_tp=np.zeros((6,7))
        


        diff_f=np.array([[R[0,0]/(R[1,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[1,2]*(x_euler[2] - np.sin(x_euler[4])) + t[1]) - R[1,0]*(R[0,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[0,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[0,2]*(x_euler[2] - np.sin(x_euler[4])) + t[0])/(R[1,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[1,2]*(x_euler[2] - np.sin(x_euler[4])) + t[1])**2], [-R[1,0]*(R[2,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[2,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[2,2]*(x_euler[2] - np.sin(x_euler[4])) + t[2])/(R[1,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[1,2]*(x_euler[2] - np.sin(x_euler[4])) + t[1])**2 + R[2,0]/(R[1,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[1,2]*(x_euler[2] - np.sin(x_euler[4])) + t[1])], [R[0,0]/(R[1,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[1]) - R[1,0]*(R[0,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[0,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[0,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[0])/(R[1,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[1])**2], [-R[1,0]*(R[2,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[2,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[2,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[2])/(R[1,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[1])**2 + R[2,0]/(R[1,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[1])], [R[0,0]/(R[1,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[1]) - R[1,0]*(R[0,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[0,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[0,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[0])/(R[1,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[1])**2], [-R[1,0]*(R[2,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[2,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[2,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[2])/(R[1,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[1])**2 + R[2,0]/(R[1,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[1])]])
        diff_g=np.array([[R[0,1]/(R[1,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[1,2]*(x_euler[2] - np.sin(x_euler[4])) + t[1]) - R[1,1]*(R[0,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[0,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[0,2]*(x_euler[2] - np.sin(x_euler[4])) + t[0])/(R[1,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[1,2]*(x_euler[2] - np.sin(x_euler[4])) + t[1])**2], [-R[1,1]*(R[2,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[2,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[2,2]*(x_euler[2] - np.sin(x_euler[4])) + t[2])/(R[1,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[1,2]*(x_euler[2] - np.sin(x_euler[4])) + t[1])**2 + R[2,1]/(R[1,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[1,2]*(x_euler[2] - np.sin(x_euler[4])) + t[1])], [R[0,1]/(R[1,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[1]) - R[1,1]*(R[0,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[0,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[0,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[0])/(R[1,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[1])**2], [-R[1,1]*(R[2,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[2,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[2,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[2])/(R[1,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[1])**2 + R[2,1]/(R[1,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[1])], [R[0,1]/(R[1,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[1]) - R[1,1]*(R[0,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[0,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[0,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[0])/(R[1,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[1])**2], [-R[1,1]*(R[2,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[2,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[2,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[2])/(R[1,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[1])**2 + R[2,1]/(R[1,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[1])]])
        diff_h=np.array([[R[0,2]/(R[1,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[1,2]*(x_euler[2] - np.sin(x_euler[4])) + t[1]) - R[1,2]*(R[0,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[0,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[0,2]*(x_euler[2] - np.sin(x_euler[4])) + t[0])/(R[1,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[1,2]*(x_euler[2] - np.sin(x_euler[4])) + t[1])**2], [-R[1,2]*(R[2,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[2,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[2,2]*(x_euler[2] - np.sin(x_euler[4])) + t[2])/(R[1,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[1,2]*(x_euler[2] - np.sin(x_euler[4])) + t[1])**2 + R[2,2]/(R[1,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[1,2]*(x_euler[2] - np.sin(x_euler[4])) + t[1])], [R[0,2]/(R[1,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[1]) - R[1,2]*(R[0,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[0,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[0,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[0])/(R[1,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[1])**2], [-R[1,2]*(R[2,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[2,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[2,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[2])/(R[1,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[1])**2 + R[2,2]/(R[1,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[1])], [R[0,2]/(R[1,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[1]) - R[1,2]*(R[0,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[0,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[0,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[0])/(R[1,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[1])**2], [-R[1,2]*(R[2,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[2,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[2,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[2])/(R[1,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[1])**2 + R[2,2]/(R[1,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[1])]])
        diff_i=np.array([[0], [0], [(R[0,0]*(np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[0,1]*(-np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[0,2]*np.cos(x_euler[3])*np.cos(x_euler[4]))/(R[1,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[1]) + (-R[1,0]*(np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) - R[1,1]*(-np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) - R[1,2]*np.cos(x_euler[3])*np.cos(x_euler[4]))*(R[0,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[0,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[0,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[0])/(R[1,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[1])**2], [(-R[1,0]*(np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) - R[1,1]*(-np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) - R[1,2]*np.cos(x_euler[3])*np.cos(x_euler[4]))*(R[2,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[2,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[2,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[2])/(R[1,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[1])**2 + (R[2,0]*(np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[2,1]*(-np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[2,2]*np.cos(x_euler[3])*np.cos(x_euler[4]))/(R[1,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[1])], [(R[0,0]*(-np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[0,1]*(-np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) - np.cos(x_euler[3])*np.cos(x_euler[5])) - R[0,2]*np.sin(x_euler[3])*np.cos(x_euler[4]))/(R[1,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[1]) + (-R[1,0]*(-np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) - R[1,1]*(-np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) - np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*np.sin(x_euler[3])*np.cos(x_euler[4]))*(R[0,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[0,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[0,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[0])/(R[1,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[1])**2], [(-R[1,0]*(-np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) - R[1,1]*(-np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) - np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*np.sin(x_euler[3])*np.cos(x_euler[4]))*(R[2,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[2,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[2,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[2])/(R[1,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[1])**2 + (R[2,0]*(-np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[2,1]*(-np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) - np.cos(x_euler[3])*np.cos(x_euler[5])) - R[2,2]*np.sin(x_euler[3])*np.cos(x_euler[4]))/(R[1,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[1])]])
        diff_j=np.array([[(-R[0,0]*np.sin(x_euler[4])*np.cos(x_euler[5]) - R[0,1]*np.sin(x_euler[4])*np.sin(x_euler[5]) - R[0,2]*np.cos(x_euler[4]))/(R[1,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[1,2]*(x_euler[2] - np.sin(x_euler[4])) + t[1]) + (R[1,0]*np.sin(x_euler[4])*np.cos(x_euler[5]) + R[1,1]*np.sin(x_euler[4])*np.sin(x_euler[5]) + R[1,2]*np.cos(x_euler[4]))*(R[0,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[0,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[0,2]*(x_euler[2] - np.sin(x_euler[4])) + t[0])/(R[1,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[1,2]*(x_euler[2] - np.sin(x_euler[4])) + t[1])**2], [(R[1,0]*np.sin(x_euler[4])*np.cos(x_euler[5]) + R[1,1]*np.sin(x_euler[4])*np.sin(x_euler[5]) + R[1,2]*np.cos(x_euler[4]))*(R[2,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[2,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[2,2]*(x_euler[2] - np.sin(x_euler[4])) + t[2])/(R[1,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[1,2]*(x_euler[2] - np.sin(x_euler[4])) + t[1])**2 + (-R[2,0]*np.sin(x_euler[4])*np.cos(x_euler[5]) - R[2,1]*np.sin(x_euler[4])*np.sin(x_euler[5]) - R[2,2]*np.cos(x_euler[4]))/(R[1,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[1,2]*(x_euler[2] - np.sin(x_euler[4])) + t[1])], [(R[0,0]*np.sin(x_euler[3])*np.cos(x_euler[4])*np.cos(x_euler[5]) + R[0,1]*np.sin(x_euler[3])*np.sin(x_euler[5])*np.cos(x_euler[4]) - R[0,2]*np.sin(x_euler[3])*np.sin(x_euler[4]))/(R[1,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[1]) + (-R[1,0]*np.sin(x_euler[3])*np.cos(x_euler[4])*np.cos(x_euler[5]) - R[1,1]*np.sin(x_euler[3])*np.sin(x_euler[5])*np.cos(x_euler[4]) + R[1,2]*np.sin(x_euler[3])*np.sin(x_euler[4]))*(R[0,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[0,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[0,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[0])/(R[1,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[1])**2], [(-R[1,0]*np.sin(x_euler[3])*np.cos(x_euler[4])*np.cos(x_euler[5]) - R[1,1]*np.sin(x_euler[3])*np.sin(x_euler[5])*np.cos(x_euler[4]) + R[1,2]*np.sin(x_euler[3])*np.sin(x_euler[4]))*(R[2,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[2,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[2,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[2])/(R[1,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[1])**2 + (R[2,0]*np.sin(x_euler[3])*np.cos(x_euler[4])*np.cos(x_euler[5]) + R[2,1]*np.sin(x_euler[3])*np.sin(x_euler[5])*np.cos(x_euler[4]) - R[2,2]*np.sin(x_euler[3])*np.sin(x_euler[4]))/(R[1,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[1])], [(R[0,0]*np.cos(x_euler[3])*np.cos(x_euler[4])*np.cos(x_euler[5]) + R[0,1]*np.sin(x_euler[5])*np.cos(x_euler[3])*np.cos(x_euler[4]) - R[0,2]*np.sin(x_euler[4])*np.cos(x_euler[3]))/(R[1,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[1]) + (-R[1,0]*np.cos(x_euler[3])*np.cos(x_euler[4])*np.cos(x_euler[5]) - R[1,1]*np.sin(x_euler[5])*np.cos(x_euler[3])*np.cos(x_euler[4]) + R[1,2]*np.sin(x_euler[4])*np.cos(x_euler[3]))*(R[0,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[0,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[0,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[0])/(R[1,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[1])**2], [(-R[1,0]*np.cos(x_euler[3])*np.cos(x_euler[4])*np.cos(x_euler[5]) - R[1,1]*np.sin(x_euler[5])*np.cos(x_euler[3])*np.cos(x_euler[4]) + R[1,2]*np.sin(x_euler[4])*np.cos(x_euler[3]))*(R[2,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[2,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[2,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[2])/(R[1,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[1])**2 + (R[2,0]*np.cos(x_euler[3])*np.cos(x_euler[4])*np.cos(x_euler[5]) + R[2,1]*np.sin(x_euler[5])*np.cos(x_euler[3])*np.cos(x_euler[4]) - R[2,2]*np.sin(x_euler[4])*np.cos(x_euler[3]))/(R[1,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[1])]])
        diff_k=np.array([[(-R[0,0]*np.sin(x_euler[5])*np.cos(x_euler[4]) + R[0,1]*np.cos(x_euler[4])*np.cos(x_euler[5]))/(R[1,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[1,2]*(x_euler[2] - np.sin(x_euler[4])) + t[1]) + (R[1,0]*np.sin(x_euler[5])*np.cos(x_euler[4]) - R[1,1]*np.cos(x_euler[4])*np.cos(x_euler[5]))*(R[0,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[0,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[0,2]*(x_euler[2] - np.sin(x_euler[4])) + t[0])/(R[1,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[1,2]*(x_euler[2] - np.sin(x_euler[4])) + t[1])**2], [(R[1,0]*np.sin(x_euler[5])*np.cos(x_euler[4]) - R[1,1]*np.cos(x_euler[4])*np.cos(x_euler[5]))*(R[2,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[2,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[2,2]*(x_euler[2] - np.sin(x_euler[4])) + t[2])/(R[1,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[1,2]*(x_euler[2] - np.sin(x_euler[4])) + t[1])**2 + (-R[2,0]*np.sin(x_euler[5])*np.cos(x_euler[4]) + R[2,1]*np.cos(x_euler[4])*np.cos(x_euler[5]))/(R[1,0]*(x_euler[0] + np.cos(x_euler[4])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] + np.sin(x_euler[5])*np.cos(x_euler[4])) + R[1,2]*(x_euler[2] - np.sin(x_euler[4])) + t[1])], [(R[0,0]*(-np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) - np.cos(x_euler[3])*np.cos(x_euler[5])) + R[0,1]*(np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])))/(R[1,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[1]) + (-R[1,0]*(-np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) - np.cos(x_euler[3])*np.cos(x_euler[5])) - R[1,1]*(np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])))*(R[0,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[0,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[0,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[0])/(R[1,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[1])**2], [(-R[1,0]*(-np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) - np.cos(x_euler[3])*np.cos(x_euler[5])) - R[1,1]*(np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])))*(R[2,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[2,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[2,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[2])/(R[1,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[1])**2 + (R[2,0]*(-np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) - np.cos(x_euler[3])*np.cos(x_euler[5])) + R[2,1]*(np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])))/(R[1,0]*(x_euler[0] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.cos(x_euler[5]) - np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,1]*(x_euler[1] + np.sin(x_euler[3])*np.sin(x_euler[4])*np.sin(x_euler[5]) + np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,2]*(x_euler[2] + np.sin(x_euler[3])*np.cos(x_euler[4])) + t[1])], [(R[0,0]*(-np.sin(x_euler[3])*np.cos(x_euler[5]) - np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[0,1]*(np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])))/(R[1,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[1]) + (-R[1,0]*(-np.sin(x_euler[3])*np.cos(x_euler[5]) - np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) - R[1,1]*(np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])))*(R[0,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[0,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[0,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[0])/(R[1,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[1])**2], [(-R[1,0]*(-np.sin(x_euler[3])*np.cos(x_euler[5]) - np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) - R[1,1]*(np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])))*(R[2,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[2,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[2,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[2])/(R[1,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[1])**2 + (R[2,0]*(-np.sin(x_euler[3])*np.cos(x_euler[5]) - np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[2,1]*(np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])))/(R[1,0]*(x_euler[0] - np.sin(x_euler[3])*np.sin(x_euler[5]) + np.sin(x_euler[4])*np.cos(x_euler[3])*np.cos(x_euler[5])) + R[1,1]*(x_euler[1] - np.sin(x_euler[3])*np.cos(x_euler[5]) + np.sin(x_euler[4])*np.sin(x_euler[5])*np.cos(x_euler[3])) + R[1,2]*(x_euler[2] + np.cos(x_euler[3])*np.cos(x_euler[4])) + t[1])]])



        for j in range(7):
            J_tp[0,j]=diff_f[0]*J[0,j]+diff_g[0]*J[1,j]+diff_h[0]*J[2,j]+diff_i[0]*J[3,j]+diff_j[0]*J[4,j]+diff_k[0]*J[5,j]
            J_tp[1,j]=diff_f[1]*J[0,j]+diff_g[1]*J[1,j]+diff_h[1]*J[2,j]+diff_i[1]*J[3,j]+diff_j[1]*J[4,j]+diff_k[1]*J[5,j]
            J_tp[2,j]=diff_f[2]*J[0,j]+diff_g[2]*J[1,j]+diff_h[2]*J[2,j]+diff_i[2]*J[3,j]+diff_j[2]*J[4,j]+diff_k[2]*J[5,j]
            J_tp[3,j]=diff_f[3]*J[0,j]+diff_g[3]*J[1,j]+diff_h[3]*J[2,j]+diff_i[3]*J[3,j]+diff_j[3]*J[4,j]+diff_k[3]*J[5,j]
            J_tp[4,j]=diff_f[4]*J[0,j]+diff_g[4]*J[1,j]+diff_h[4]*J[2,j]+diff_i[4]*J[3,j]+diff_j[4]*J[4,j]+diff_k[4]*J[5,j]
            J_tp[5,j]=diff_f[5]*J[0,j]+diff_g[5]*J[1,j]+diff_h[5]*J[2,j]+diff_i[5]*J[3,j]+diff_j[5]*J[4,j]+diff_k[5]*J[5,j]

        # print(J_tp)

        J_inv = np.linalg.pinv(J_tp)
        # print(J_inv)

        e=g-x
        e_miss=g_miss-x_miss
        # print(e)

        e_norm = np.linalg.norm(e)

        e_norm_traj.append(e_norm)

        P_e=e_miss*gain

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
    print(x)
    # print(g)
    # print(x_miss)
    print(g_miss)
    print(x_quat)
    print(g_quat)
   #  print(e_norm_3d_traj)
    # print(e_norm_traj)
    # print(x_camera_traj)
