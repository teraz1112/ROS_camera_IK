# -*- coding: utf-8 -*-
#!/usr/bin/python
from core_tool import *
import math
import copy
import numpy as np
import tf.transformations as tft
import sympy as sp
from tqdm import tqdm
import random

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


def MakeCameraAxis_Rand(range,roll,pitch,yaw,euler):
   dis=np.array(range)
   R=EtoM(roll,pitch,yaw)
   camera=np.zeros(2)
   p=np.dot(R,euler[:3])+dis
   random_number1 = random.uniform(-0.005, 0.005)
   random_number2 = random.uniform(-0.005, 0.005)
   camera[0]=p[0]/p[1]+random_number1
   camera[1]=p[2]/p[1]+random_number2
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

def LST(A,q):
   A_transposed = A.T
   B=np.dot(A_transposed,A)
   C=np.linalg.inv(B)
   D=np.dot(C,A_transposed)
   solution = np.dot(D, q)
   return solution


def CulP6(R0,euler):
   p1=copy.deepcopy(euler)
   p2=copy.deepcopy(euler)
   p3=copy.deepcopy(euler)
   p4=copy.deepcopy(euler)
   p5=copy.deepcopy(euler)
   p6=copy.deepcopy(euler)
   p1[0]+=R0[0,0]*5
   p1[1]+=R0[1,0]*5
   p1[2]+=R0[2,0]*5
   p2[0]+=R0[0,1]*5
   p2[1]+=R0[1,1]*5
   p2[2]+=R0[2,1]*5
   p3[0]+=R0[0,2]*5
   p3[1]+=R0[1,2]*5
   p3[2]+=R0[2,2]*5
   p4[0]+=R0[0,0]*10
   p4[1]+=R0[1,0]*10
   p4[2]+=R0[2,0]*10
   p5[0]+=R0[0,1]*10
   p5[1]+=R0[1,1]*10
   p5[2]+=R0[2,1]*10
   p6[0]+=R0[0,2]*10
   p6[1]+=R0[1,2]*10
   p6[2]+=R0[2,2]*10
   return p1,p2,p3,p4,p5,p6

def Run(ct,*args):
    #camera1
    t=[0,4,0]
    roll1=-np.pi/4
    pitch1=0
    yaw1=0
    R=EtoM(roll1,pitch1,yaw1)
    print(roll1,pitch1,yaw1,t)

    #camera_miss
    t_miss=[0.02,4,0.01]
    roll1_miss=-np.pi/3
    pitch1_miss=0.01
    yaw1_miss=0.025
    R_miss=EtoM(roll1_miss,pitch1_miss,yaw1_miss)
    print(roll1_miss,pitch1_miss,yaw1_miss,t_miss)

    ####################################################################################################################################################################################################################
    q=ct.robot.Q()
    x_quat= ct.robot.FK(q)
    Rq=QtoM(x_quat[3:])
    x_euler=QtoE(x_quat)
    g_tp1,g_tp2,g_tp3,g_tp4,g_tp5,g_tp6=CulP6(Rq,x_euler[:3])

    camera1=MakeCameraAxis_Rand(t_miss,roll1_miss,pitch1_miss,yaw1_miss,g_tp1)
    camera2=MakeCameraAxis_Rand(t_miss,roll1_miss,pitch1_miss,yaw1_miss,g_tp2)
    camera3=MakeCameraAxis_Rand(t_miss,roll1_miss,pitch1_miss,yaw1_miss,g_tp3)
    camera4=MakeCameraAxis_Rand(t_miss,roll1_miss,pitch1_miss,yaw1_miss,g_tp4)
    camera5=MakeCameraAxis_Rand(t_miss,roll1_miss,pitch1_miss,yaw1_miss,g_tp5)  
    camera6=MakeCameraAxis_Rand(t_miss,roll1_miss,pitch1_miss,yaw1_miss,g_tp6)  

    # 係数行列とベクトルの用意
    B = np.array([[g_tp1[0],g_tp1[1],g_tp1[2] , -camera1[0]*g_tp1[0],-camera1[0]*g_tp1[1],-camera1[0]*g_tp1[2],0,0,0,1,0],
                [0,0,0,-camera1[1]*g_tp1[0],-camera1[1]*g_tp1[1],-camera1[1]*g_tp1[2],g_tp1[0],g_tp1[1],g_tp1[2] , 0,1],
                [g_tp2[0],g_tp2[1],g_tp2[2] , -camera2[0]*g_tp2[0],-camera2[0]*g_tp2[1],-camera2[0]*g_tp2[2],0,0,0,1,0],
                [0,0,0,-camera2[1]*g_tp2[0],-camera2[1]*g_tp2[1],-camera2[1]*g_tp2[2],g_tp2[0],g_tp2[1],g_tp2[2] , 0,1],
                [g_tp3[0],g_tp3[1],g_tp3[2] , -camera3[0]*g_tp3[0],-camera3[0]*g_tp3[1],-camera3[0]*g_tp3[2],0,0,0,1,0],
                [0,0,0,-camera3[1]*g_tp3[0],-camera3[1]*g_tp3[1],-camera3[1]*g_tp3[2],g_tp3[0],g_tp3[1],g_tp3[2] , 0,1],
                [g_tp4[0],g_tp4[1],g_tp4[2] , -camera4[0]*g_tp4[0],-camera4[0]*g_tp4[1],-camera4[0]*g_tp4[2],0,0,0,1,0],
                [0,0,0,-camera4[1]*g_tp4[0],-camera4[1]*g_tp4[1],-camera4[1]*g_tp4[2],g_tp4[0],g_tp4[1],g_tp4[2] , 0,1],
                [g_tp5[0],g_tp5[1],g_tp5[2] , -camera5[0]*g_tp5[0],-camera5[0]*g_tp5[1],-camera5[0]*g_tp5[2],0,0,0,1,0],
                [0,0,0,-camera5[1]*g_tp5[0],-camera5[1]*g_tp5[1],-camera5[1]*g_tp5[2],g_tp5[0],g_tp5[1],g_tp5[2] , 0,1],
                [g_tp6[0],g_tp6[1],g_tp6[2] , -camera6[0]*g_tp6[0],-camera6[0]*g_tp6[1],-camera6[0]*g_tp6[2],0,0,0,1,0],
                [0,0,0,-camera6[1]*g_tp6[0],-camera6[1]*g_tp6[1],-camera6[1]*g_tp6[2],g_tp6[0],g_tp6[1],g_tp6[2] , 0,1]])
    
    xc = np.array([camera1[0]*4,camera1[1]*4,camera2[0]*4,camera2[1]*4,camera3[0]*4,camera3[1]*4,camera4[0]*4,camera4[1]*4,camera5[0]*4,camera5[1]*4,camera6[0]*4,camera6[1]*4])

    #最小二乗法
    solution = LST(B,xc)
    
    R_guess=np.array([[solution[0],solution[1],solution[2]],
                        [solution[3],solution[4],solution[5]],
                        [solution[6],solution[7],solution[8]],])
    t_guess=np.array([solution[9],4,solution[10]])

    r_guess,p_guess,y_guess=tft.euler_from_matrix(np.linalg.pinv(R_guess))
    # print(r_guess,p_guess,y_guess,t_guess)

    ###########################################################################################################################################################################################################################

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

    q=ct.robot.Q()
    q_traj= []
    t_traj= []
    x_camera_traj=[]
    e_norm_traj=[]
    e_norm_3d_traj=[]
    q_traj.append(q)
    t_traj.append(0.0)

    dt=0.06
    gain=0.1
    

    for i in tqdm(range(600), desc="Processing", unit="iteration"):
        t_traj.append(dt*(i+1))
        
        x_quat= ct.robot.FK(q)
        x_tp=np.array(x_quat[:3])
        e_3d=g_tp-x_tp
        e_norm_3d = np.linalg.norm(e_3d)
        e_norm_3d_traj.append(e_norm_3d)
            
        x_euler=QtoE(x_quat)

        Rq=QtoM(x_quat[3:])

        ###########################################################################################################################################################################################################################
        g_tp1,g_tp2,g_tp3,g_tp4,g_tp5,g_tp6=CulP6(Rq,x_euler[:3])

        camera1=MakeCameraAxis_Rand(t_miss,roll1_miss,pitch1_miss,yaw1_miss,g_tp1)
        camera2=MakeCameraAxis_Rand(t_miss,roll1_miss,pitch1_miss,yaw1_miss,g_tp2)
        camera3=MakeCameraAxis_Rand(t_miss,roll1_miss,pitch1_miss,yaw1_miss,g_tp3)
        camera4=MakeCameraAxis_Rand(t_miss,roll1_miss,pitch1_miss,yaw1_miss,g_tp4)
        camera5=MakeCameraAxis_Rand(t_miss,roll1_miss,pitch1_miss,yaw1_miss,g_tp5)  
        camera6=MakeCameraAxis_Rand(t_miss,roll1_miss,pitch1_miss,yaw1_miss,g_tp6)  

        # 係数行列とベクトルの用意
        A = np.array([[g_tp1[0],g_tp1[1],g_tp1[2] , -camera1[0]*g_tp1[0],-camera1[0]*g_tp1[1],-camera1[0]*g_tp1[2],0,0,0,1,0],
                    [0,0,0,-camera1[1]*g_tp1[0],-camera1[1]*g_tp1[1],-camera1[1]*g_tp1[2],g_tp1[0],g_tp1[1],g_tp1[2] , 0,1],
                    [g_tp2[0],g_tp2[1],g_tp2[2] , -camera2[0]*g_tp2[0],-camera2[0]*g_tp2[1],-camera2[0]*g_tp2[2],0,0,0,1,0],
                    [0,0,0,-camera2[1]*g_tp2[0],-camera2[1]*g_tp2[1],-camera2[1]*g_tp2[2],g_tp2[0],g_tp2[1],g_tp2[2] , 0,1],
                    [g_tp3[0],g_tp3[1],g_tp3[2] , -camera3[0]*g_tp3[0],-camera3[0]*g_tp3[1],-camera3[0]*g_tp3[2],0,0,0,1,0],
                    [0,0,0,-camera3[1]*g_tp3[0],-camera3[1]*g_tp3[1],-camera3[1]*g_tp3[2],g_tp3[0],g_tp3[1],g_tp3[2] , 0,1],
                    [g_tp4[0],g_tp4[1],g_tp4[2] , -camera4[0]*g_tp4[0],-camera4[0]*g_tp4[1],-camera4[0]*g_tp4[2],0,0,0,1,0],
                    [0,0,0,-camera4[1]*g_tp4[0],-camera4[1]*g_tp4[1],-camera4[1]*g_tp4[2],g_tp4[0],g_tp4[1],g_tp4[2] , 0,1],
                    [g_tp5[0],g_tp5[1],g_tp5[2] , -camera5[0]*g_tp5[0],-camera5[0]*g_tp5[1],-camera5[0]*g_tp5[2],0,0,0,1,0],
                    [0,0,0,-camera5[1]*g_tp5[0],-camera5[1]*g_tp5[1],-camera5[1]*g_tp5[2],g_tp5[0],g_tp5[1],g_tp5[2] , 0,1],
                    [g_tp6[0],g_tp6[1],g_tp6[2] , -camera6[0]*g_tp6[0],-camera6[0]*g_tp6[1],-camera6[0]*g_tp6[2],0,0,0,1,0],
                    [0,0,0,-camera6[1]*g_tp6[0],-camera6[1]*g_tp6[1],-camera6[1]*g_tp6[2],g_tp6[0],g_tp6[1],g_tp6[2] , 0,1]])

        B = np.vstack((B, A))                

        b = np.array([camera1[0]*4,camera1[1]*4,camera2[0]*4,camera2[1]*4,camera3[0]*4,camera3[1]*4,camera4[0]*4,camera4[1]*4,camera5[0]*4,camera5[1]*4,camera6[0]*4,camera6[1]*4])

        xc=np.append(xc,b)
        ###########################################################################################################################################################################################################################

        x1,x2,x3=CulP(Rq,x_euler)

        x_camera1=MakeCameraAxis_Rand(t,roll1,pitch1,yaw1,x1)
        x_camera2=MakeCameraAxis_Rand(t,roll1,pitch1,yaw1,x2)
        x_camera3=MakeCameraAxis_Rand(t,roll1,pitch1,yaw1,x3)
        x_camera1_miss=MakeCameraAxis_Rand(t_miss,roll1_miss,pitch1_miss,yaw1_miss,x1)
        x_camera2_miss=MakeCameraAxis_Rand(t_miss,roll1_miss,pitch1_miss,yaw1_miss,x2)
        x_camera3_miss=MakeCameraAxis_Rand(t_miss,roll1_miss,pitch1_miss,yaw1_miss,x3)
        x=make1raw(x_camera1,x_camera2,x_camera3)
        x_miss=make1raw(x_camera1_miss,x_camera2_miss,x_camera3_miss)

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


        J_inv = np.linalg.pinv(J_tp)

        e=g-x
        e_miss=g_miss-x

        e_norm = np.linalg.norm(e)

        e_norm_traj.append(e_norm)

        P_e=e_miss*gain

        vw=np.dot(J_inv,P_e)
        
        pre_q=q_traj[-1]

        q=pre_q+vw*dt

        q_traj.append(q)
    
    solution = LST(B,xc)
    R_guess=np.array([[solution[0],solution[1],solution[2]],
                    [solution[3],solution[4],solution[5]],
                    [solution[6],solution[7],solution[8]],])
    t_guess=np.array([solution[9],4,solution[10]])
    r_guess,p_guess,y_guess=tft.euler_from_matrix(np.linalg.pinv(R_guess))
    print(r_guess,p_guess,y_guess,t_guess)
    dr=roll1-r_guess
    dp=pitch1-p_guess
    dy=yaw1-y_guess
    dT=t-t_guess
    print(dr,dp,dy,dT)
    t_miss[0]+=dT[0]
    t_miss[2]+=dT[2]
    roll1_miss+=dr
    pitch1_miss+=dp
    yaw1_miss+=dy

    print(x)
# print(g)
# print(x_miss)
    print(g_miss)
    print(x_quat)
    print(g_quat)
    print(e_norm_3d_traj)
    #   print(e_norm_3d)
    print(e_norm_traj)
    # print(x_camera_traj)

# print(np.size(q_traj))
    ct.robot.FollowQTraj(q_traj, t_traj)
