# -*- coding: utf-8 -*-
#!/usr/bin/python
from core_tool import *
import math
import copy
import numpy as np
from numpy.linalg import svd, matrix_rank
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


def EtoM(roll,pitch,yaw):
   R=np.linalg.pinv(tft.euler_matrix(roll,pitch,yaw))[:3, :3]
   return R

def MakeCameraAxis(range,roll,pitch,yaw,euler):
   dis=np.array(range)
   R=EtoM(roll,pitch,yaw)
   camera=np.zeros(2)
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


def CulP(R0,euler):
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

def LST(A,q):
   A_transposed = A.T
   B=np.dot(A_transposed,A)
   C=np.linalg.inv(B)
   D=np.dot(C,A_transposed)
   solution = np.dot(D, q)
   return solution

def Run(ct,*args):

   t=[0,4,0]
   roll1=-np.pi/4
   pitch1=0
   yaw1=0
   R=EtoM(roll1,pitch1,yaw1)
#    print(roll1,pitch1,yaw1,t)
   
   t_miss=[0.72,4,0.89]
   roll1_miss=0.5
   pitch1_miss=0.2
   yaw1_miss=-0.1
   R_miss=EtoM(roll1,pitch1,yaw1)
   print(roll1_miss,pitch1_miss,yaw1_miss,t_miss)

 
   for i in range(500):


    q=ct.robot.Q()
    x_quat= ct.robot.FK(q)
    Rq=QtoM(x_quat[3:])
    x_euler=QtoE(x_quat)
    g_tp1,g_tp2,g_tp3,g_tp4,g_tp5,g_tp6=CulP(Rq,x_euler[:3])

    g_camera1=MakeCameraAxis_Rand(t_miss,roll1_miss,pitch1_miss,yaw1_miss,g_tp1)
    g_camera2=MakeCameraAxis_Rand(t_miss,roll1_miss,pitch1_miss,yaw1_miss,g_tp2)
    g_camera3=MakeCameraAxis_Rand(t_miss,roll1_miss,pitch1_miss,yaw1_miss,g_tp3)
    g_camera4=MakeCameraAxis_Rand(t_miss,roll1_miss,pitch1_miss,yaw1_miss,g_tp4)
    g_camera5=MakeCameraAxis_Rand(t_miss,roll1_miss,pitch1_miss,yaw1_miss,g_tp5)  
    g_camera6=MakeCameraAxis_Rand(t_miss,roll1_miss,pitch1_miss,yaw1_miss,g_tp6)  

    
    #    print(g_camera1)
    #    print(g_camera2)
    #    print(g_camera3)
    #    print(g_camera4)
    #    print(g_camera5)
    #    print(g_camera6)

    
    # 係数行列とベクトルの用意
    A = np.array([[g_tp1[0],g_tp1[1],g_tp1[2] , -g_camera1[0]*g_tp1[0],-g_camera1[0]*g_tp1[1],-g_camera1[0]*g_tp1[2],0,0,0,1,0],
                [0,0,0,-g_camera1[1]*g_tp1[0],-g_camera1[1]*g_tp1[1],-g_camera1[1]*g_tp1[2],g_tp1[0],g_tp1[1],g_tp1[2] , 0,1],
                [g_tp2[0],g_tp2[1],g_tp2[2] , -g_camera2[0]*g_tp2[0],-g_camera2[0]*g_tp2[1],-g_camera2[0]*g_tp2[2],0,0,0,1,0],
                [0,0,0,-g_camera2[1]*g_tp2[0],-g_camera2[1]*g_tp2[1],-g_camera2[1]*g_tp2[2],g_tp2[0],g_tp2[1],g_tp2[2] , 0,1],
                [g_tp3[0],g_tp3[1],g_tp3[2] , -g_camera3[0]*g_tp3[0],-g_camera3[0]*g_tp3[1],-g_camera3[0]*g_tp3[2],0,0,0,1,0],
                [0,0,0,-g_camera3[1]*g_tp3[0],-g_camera3[1]*g_tp3[1],-g_camera3[1]*g_tp3[2],g_tp3[0],g_tp3[1],g_tp3[2] , 0,1],
                [g_tp4[0],g_tp4[1],g_tp4[2] , -g_camera4[0]*g_tp4[0],-g_camera4[0]*g_tp4[1],-g_camera4[0]*g_tp4[2],0,0,0,1,0],
                [0,0,0,-g_camera4[1]*g_tp4[0],-g_camera4[1]*g_tp4[1],-g_camera4[1]*g_tp4[2],g_tp4[0],g_tp4[1],g_tp4[2] , 0,1],
                [g_tp5[0],g_tp5[1],g_tp5[2] , -g_camera5[0]*g_tp5[0],-g_camera5[0]*g_tp5[1],-g_camera5[0]*g_tp5[2],0,0,0,1,0],
                [0,0,0,-g_camera5[1]*g_tp5[0],-g_camera5[1]*g_tp5[1],-g_camera5[1]*g_tp5[2],g_tp5[0],g_tp5[1],g_tp5[2] , 0,1],
                [g_tp6[0],g_tp6[1],g_tp6[2] , -g_camera6[0]*g_tp6[0],-g_camera6[0]*g_tp6[1],-g_camera6[0]*g_tp6[2],0,0,0,1,0],
                [0,0,0,-g_camera6[1]*g_tp6[0],-g_camera6[1]*g_tp6[1],-g_camera6[1]*g_tp6[2],g_tp6[0],g_tp6[1],g_tp6[2] , 0,1]])                
    
    b = np.array([g_camera1[0]*4,g_camera1[1]*4,g_camera2[0]*4,g_camera2[1]*4,g_camera3[0]*4,g_camera3[1]*4,g_camera4[0]*4,g_camera4[1]*4,g_camera5[0]*4,g_camera5[1]*4,g_camera6[0]*4,g_camera6[1]*4])
    
    # #連立方程式
    # solution = np.linalg.solve(A, b)
    #最小二乗法
    solution = LST(A,b)
    # print(solution)
    
    R_guess=np.array([[solution[0],solution[1],solution[2]],
                        [solution[3],solution[4],solution[5]],
                        [solution[6],solution[7],solution[8]],])
    t_guess=np.array([solution[9],4,solution[10]])

    # print(R_miss)
    #    print(R_guess)
    r_guess,p_guess,y_guess=tft.euler_from_matrix(np.linalg.pinv(R_guess))
    print(r_guess,p_guess,y_guess,t_guess)

    dr=roll1-r_guess
    dp=pitch1-p_guess
    dy=yaw1-y_guess
    dt=t-t_guess
    #    print(dr,dp,dy,dt)

