# -*- coding: utf-8 -*-
#!/usr/bin/python
from core_tool import *
import math
import copy
import numpy as np
import tf.transformations as tft
import sympy as sp

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

def WtoC(world_array):
   world=world_array
   camera=np.zeros(2)
   camera[0]=(2*world[0])/(world[1]-np.sqrt(3)*world[2]+8)
   camera[1]=(np.sqrt(3)*world[1]+world[2])/(world[1]-np.sqrt(3)*world[2]+8)
   return np.array(camera)

def CulP(R0,euler):
   euler_only=np.array(euler[:3])
   p1=copy.deepcopy(euler_only)
   p2=copy.deepcopy(euler_only)
   p3=copy.deepcopy(euler_only)
   p1[0]+=R0[0,0]
   p1[1]+=R0[1,0]
   p1[2]+=R0[2,0]
   p2[0]+=R0[0,1]
   p2[1]+=R0[1,1]
   p2[2]+=R0[2,1]
   p3[0]+=R0[0,2]
   p3[1]+=R0[1,2]
   p3[2]+=R0[2,2]
   return p1,p2,p3

def dXdp(x):
    # 変数を定義
    (f,g,h,i,j,k) = sp.symbols("f g h i j k")
    # 行列を定義
    A = sp.Matrix([
        [2*(f+sp.cos(j)*sp.cos(k))/(g+sp.cos(j)*sp.sin(k)-sp.sqrt(3)*(h-sp.sin(j))+8),(sp.sqrt(3)*(g+sp.cos(j)*sp.sin(k))+h-sp.sin(j))/(g+sp.cos(j)*sp.sin(k)-sp.sqrt(3)*(h-sp.sin(j))+8),
        2*(f+sp.sin(i)*sp.sin(j)*sp.cos(k)-sp.cos(i)*sp.sin(k))/(g+sp.sin(i)*sp.sin(j)*sp.sin(k)+sp.cos(i)*sp.cos(k)-sp.sqrt(3)*(h+sp.sin(i)*sp.cos(j))+8),(sp.sqrt(3)*(g+sp.sin(i)*sp.sin(j)*sp.sin(k)+sp.cos(i)*sp.cos(k))+h+sp.sin(i)*sp.cos(j))/(g+sp.sin(i)*sp.sin(j)*sp.sin(k)+sp.cos(i)*sp.cos(k)-sp.sqrt(3)*(h+sp.sin(i)*sp.cos(j))+8),
        2*(f+sp.cos(i)*sp.sin(j)*sp.cos(k)-sp.sin(i)*sp.sin(k))/(g+sp.cos(i)*sp.sin(j)*sp.sin(k)-sp.sin(i)*sp.cos(k)-sp.sqrt(3)*(h+sp.cos(i)*sp.cos(j))+8),(sp.sqrt(3)*(g+sp.cos(i)*sp.sin(j)*sp.sin(k)-sp.sin(i)*sp.cos(k))+h+sp.cos(i)*sp.cos(j))/(g+sp.cos(i)*sp.sin(j)*sp.sin(k)-sp.sin(i)*sp.cos(k)-sp.sqrt(3)*(h+sp.cos(i)*sp.cos(j))+8)]
    ])

    dmdf=float(sp.diff(A[0], f).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dmdg=float(sp.diff(A[0], g).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dmdh=float(sp.diff(A[0], h).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dmdi=float(sp.diff(A[0], i).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dmdj=float(sp.diff(A[0], j).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dmdk=float(sp.diff(A[0], k).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dndf=float(sp.diff(A[1], f).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dndg=float(sp.diff(A[1], g).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dndh=float(sp.diff(A[1], h).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dndi=float(sp.diff(A[1], i).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dndj=float(sp.diff(A[1], j).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dndk=float(sp.diff(A[1], k).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dodf=float(sp.diff(A[2], f).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dodg=float(sp.diff(A[2], g).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dodh=float(sp.diff(A[2], h).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dodi=float(sp.diff(A[2], i).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dodj=float(sp.diff(A[2], j).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dodk=float(sp.diff(A[2], k).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dpdf=float(sp.diff(A[3], f).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dpdg=float(sp.diff(A[3], g).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dpdh=float(sp.diff(A[3], h).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dpdi=float(sp.diff(A[3], i).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dpdj=float(sp.diff(A[3], j).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dpdk=float(sp.diff(A[3], k).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dqdf=float(sp.diff(A[4], f).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dqdg=float(sp.diff(A[4], g).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dqdh=float(sp.diff(A[4], h).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dqdi=float(sp.diff(A[4], i).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dqdj=float(sp.diff(A[4], j).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    dqdk=float(sp.diff(A[4], k).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    drdf=float(sp.diff(A[5], f).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    drdg=float(sp.diff(A[5], g).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    drdh=float(sp.diff(A[5], h).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    drdi=float(sp.diff(A[5], i).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    drdj=float(sp.diff(A[5], j).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    drdk=float(sp.diff(A[5], k).subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
    return dmdf,dmdg,dmdh,dmdi,dmdj,dmdk,dndf,dndg,dndh,dndi,dndj,dndk,dodf,dodg,dodh,dodi,dodj,dodk,dpdf,dpdg,dpdh,dpdi,dpdj,dpdk,dqdf,dqdg,dqdh,dqdi,dqdj,dqdk,drdf,drdg,drdh,drdi,drdj,drdk

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
    g_quat=args[0]
    R0=QtoM(g_quat[3:])
    g_euler=QtoE(g_quat)
    g1,g2,g3=CulP(R0,g_euler)
    g_camera1=WtoC(g1)
    g_camera2=WtoC(g2)
    g_camera3=WtoC(g3)
    g=make1raw(g_camera1,g_camera2,g_camera3)
    print(g)
    # print(g_quat)
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
    q_traj.append(q)
    t_traj.append(0.0)

    dt=0.1
    gain=0.3
 
    for i in range(1,500):
        t_traj.append(dt*i)
        
        x_quat= ct.robot.FK(q)
            
        x_euler=QtoE(x_quat)

        Rq=QtoM(x_quat[3:])

        x1,x2,x3=CulP(Rq,x_euler)

        x_camera1=WtoC(x1)
        x_camera2=WtoC(x2)
        x_camera3=WtoC(x3)
        x=make1raw(x_camera1,x_camera2,x_camera3)
        # print(x)

        x_camera_traj.append(x)    

        J=np.array(ct.robot.J(q))

        J_tp=np.zeros((6,7))

        dmdf,dmdg,dmdh,dmdi,dmdj,dmdk,dndf,dndg,dndh,dndi,dndj,dndk,dodf,dodg,dodh,dodi,dodj,dodk,dpdf,dpdg,dpdh,dpdi,dpdj,dpdk,dqdf,dqdg,dqdh,dqdi,dqdj,dqdk,drdf,drdg,drdh,drdi,drdj,drdk=dXdp(x_euler)

        for j in range(7):
            J_tp[0,j]=dmdf*J[0,j]+dmdg*J[1,j]+dmdh*J[2,j]+dmdi*J[3,j]+dmdj*J[4,j]+dmdk*J[5,j]
            J_tp[1,j]=dndf*J[0,j]+dndg*J[1,j]+dndh*J[2,j]+dndi*J[3,j]+dndj*J[4,j]+dndk*J[5,j]
            J_tp[2,j]=dodf*J[0,j]+dodg*J[1,j]+dodh*J[2,j]+dodi*J[3,j]+dodj*J[4,j]+dodk*J[5,j]
            J_tp[3,j]=dpdf*J[0,j]+dpdg*J[1,j]+dpdh*J[2,j]+dpdi*J[3,j]+dpdj*J[4,j]+dpdk*J[5,j]
            J_tp[4,j]=dqdf*J[0,j]+dqdg*J[1,j]+dqdh*J[2,j]+dqdi*J[3,j]+dqdj*J[4,j]+dqdk*J[5,j]
            J_tp[5,j]=drdf*J[0,j]+drdg*J[1,j]+drdh*J[2,j]+drdi*J[3,j]+drdj*J[4,j]+drdk*J[5,j]

        # print(J_tp)

        J_inv = np.linalg.pinv(J_tp)
        # print(J_inv)

        e=g-x
        # print(e)

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
    print(x)
    ct.robot.FollowQTraj(q_traj, t_traj)

    # print(e_norm_traj)
    print(x_camera_traj)
