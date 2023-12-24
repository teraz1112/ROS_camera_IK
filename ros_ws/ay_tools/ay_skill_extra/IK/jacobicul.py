
import sympy as sp
from IPython.display import Math, display

# 変数を定義
(f,g,h,i,j,k) = sp.symbols("f g h i j k")
# 行列を定義
A = sp.Matrix([
    [2*(f+sp.cos(j)*sp.cos(k))/(g+sp.cos(j)*sp.sin(k)-sp.sqrt(3)*(h-sp.sin(j))+8),(sp.sqrt(3)*(g+sp.cos(j)*sp.sin(k))+h-sp.sin(j))/(g+sp.cos(j)*sp.sin(k)-sp.sqrt(3)*(h-sp.sin(j))+8),
    2*(f+sp.sin(i)*sp.sin(j)*sp.cos(k)-sp.cos(i)*sp.sin(k))/(g+sp.sin(i)*sp.sin(j)*sp.sin(k)+sp.cos(i)*sp.cos(k)-sp.sqrt(3)*(h+sp.sin(i)*sp.cos(j))+8),(sp.sqrt(3)*(g+sp.sin(i)*sp.sin(j)*sp.sin(k)+sp.cos(i)*sp.cos(k))+h+sp.sin(i)*sp.cos(j))/(g+sp.sin(i)*sp.sin(j)*sp.sin(k)+sp.cos(i)*sp.cos(k)-sp.sqrt(3)*(h+sp.sin(i)*sp.cos(j))+8),
    2*(f+sp.cos(i)*sp.sin(j)*sp.cos(k)-sp.sin(i)*sp.sin(k))/(g+sp.cos(i)*sp.sin(j)*sp.sin(k)-sp.sin(i)*sp.cos(k)-sp.sqrt(3)*(h+sp.cos(i)*sp.cos(j))+8),(sp.sqrt(3)*(g+sp.cos(i)*sp.sin(j)*sp.sin(k)-sp.sin(i)*sp.cos(k))+h+sp.cos(i)*sp.cos(j))/(g+sp.cos(i)*sp.sin(j)*sp.sin(k)-sp.sin(i)*sp.cos(k)-sp.sqrt(3)*(h+sp.cos(i)*sp.cos(j))+8)]
])

y=[10,10,20,20,20,4]

dmdf=float(sp.diff(A[0], f).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dmdg=float(sp.diff(A[0], g).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dmdh=float(sp.diff(A[0], h).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dmdi=float(sp.diff(A[0], i).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dmdj=float(sp.diff(A[0], j).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dmdk=float(sp.diff(A[0], k).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dndf=float(sp.diff(A[1], f).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dndg=float(sp.diff(A[1], g).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dndh=float(sp.diff(A[1], h).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dndi=float(sp.diff(A[1], i).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dndj=float(sp.diff(A[1], j).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dndk=float(sp.diff(A[1], k).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dodf=float(sp.diff(A[2], f).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dodg=float(sp.diff(A[2], g).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dodh=float(sp.diff(A[2], h).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dodi=float(sp.diff(A[2], i).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dodj=float(sp.diff(A[2], j).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dodk=float(sp.diff(A[2], k).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dpdf=float(sp.diff(A[3], f).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dpdg=float(sp.diff(A[3], g).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dpdh=float(sp.diff(A[3], h).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dpdi=float(sp.diff(A[3], i).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dpdj=float(sp.diff(A[3], j).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dpdk=float(sp.diff(A[3], k).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dqdf=float(sp.diff(A[4], f).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dqdg=float(sp.diff(A[4], g).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dqdh=float(sp.diff(A[4], h).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dqdi=float(sp.diff(A[4], i).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dqdj=float(sp.diff(A[4], j).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
dqdk=float(sp.diff(A[4], k).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
drdf=float(sp.diff(A[5], f).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
drdg=float(sp.diff(A[5], g).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
drdh=float(sp.diff(A[5], h).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
drdi=float(sp.diff(A[5], i).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
drdj=float(sp.diff(A[5], j).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
drdk=float(sp.diff(A[5], k).subs({f:y[0], g: y[1],h:y[2], i: y[3],j:y[4], k: y[5]}))
print(dmdf,dmdg,dmdh,dmdi,dmdj,dmdk,dndf,dndg,dndh,dndi,dndj,dndk,dodf,dodg,dodh,dodi,dodj,dodk,dpdf,dpdg,dpdh,dpdi,dpdj,dpdk,dqdf,dqdg,dqdh,dqdi,dqdj,dqdk,drdf,drdg,drdh,drdi,drdj,drdk)

