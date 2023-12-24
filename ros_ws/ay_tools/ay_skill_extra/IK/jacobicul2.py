
import sympy as sp
from IPython.display import Math, display

# 変数を定義
(f,g,h,i,j,k) = sp.symbols("f g h i j k")

x=[10,10,20,20,20,4]

a1=2/(g - sp.sqrt(3)*(h - sp.sin(j)) + sp.sin(k)*sp.cos(j) + 8)
a2=-(2*f + 2*sp.cos(j)*sp.cos(k))/(g - sp.sqrt(3)*(h - sp.sin(j)) + sp.sin(k)*sp.cos(j) + 8)**2
a3=sp.sqrt(3)*(2*f + 2*sp.cos(j)*sp.cos(k))/(g - sp.sqrt(3)*(h - sp.sin(j)) + sp.sin(k)*sp.cos(j) + 8)**2
a4=0
a5=(2*f + 2*sp.cos(j)*sp.cos(k))*(sp.sin(j)*sp.sin(k) - sp.sqrt(3)*sp.cos(j))/(g - sp.sqrt(3)*(h - sp.sin(j)) + sp.sin(k)*sp.cos(j) + 8)**2 - 2*sp.sin(j)*sp.cos(k)/(g - sp.sqrt(3)*(h - sp.sin(j)) + sp.sin(k)*sp.cos(j) + 8)
a6=-(2*f + 2*sp.cos(j)*sp.cos(k))*sp.cos(j)*sp.cos(k)/(g - sp.sqrt(3)*(h - sp.sin(j)) + sp.sin(k)*sp.cos(j) + 8)**2 - 2*sp.sin(k)*sp.cos(j)/(g - sp.sqrt(3)*(h - sp.sin(j)) + sp.sin(k)*sp.cos(j) + 8)

b1=0
b2=-(h + sp.sqrt(3)*(g + sp.sin(k)*sp.cos(j)) - sp.sin(j))/(g - sp.sqrt(3)*(h - sp.sin(j)) + sp.sin(k)*sp.cos(j) + 8)**2 + sp.sqrt(3)/(g - sp.sqrt(3)*(h - sp.sin(j)) + sp.sin(k)*sp.cos(j) + 8)
b3=sp.sqrt(3)*(h + sp.sqrt(3)*(g + sp.sin(k)*sp.cos(j)) - sp.sin(j))/(g - sp.sqrt(3)*(h - sp.sin(j)) + sp.sin(k)*sp.cos(j) + 8)**2 + 1/(g - sp.sqrt(3)*(h - sp.sin(j)) + sp.sin(k)*sp.cos(j) + 8)
b4=0
b5=(sp.sin(j)*sp.sin(k) - sp.sqrt(3)*sp.cos(j))*(h + sp.sqrt(3)*(g + sp.sin(k)*sp.cos(j)) - sp.sin(j))/(g - sp.sqrt(3)*(h - sp.sin(j)) + sp.sin(k)*sp.cos(j) + 8)**2 + (-sp.sqrt(3)*sp.sin(j)*sp.sin(k) - sp.cos(j))/(g - sp.sqrt(3)*(h - sp.sin(j)) + sp.sin(k)*sp.cos(j) + 8)
b6=-(h + sp.sqrt(3)*(g + sp.sin(k)*sp.cos(j)) - sp.sin(j))*sp.cos(j)*sp.cos(k)/(g - sp.sqrt(3)*(h - sp.sin(j)) + sp.sin(k)*sp.cos(j) + 8)**2 + sp.sqrt(3)*sp.cos(j)*sp.cos(k)/(g - sp.sqrt(3)*(h - sp.sin(j)) + sp.sin(k)*sp.cos(j) + 8)

c1=2/(g - sp.sqrt(3)*(h + sp.sin(i)*sp.cos(j)) + sp.sin(i)*sp.sin(j)*sp.sin(k) + sp.cos(i)*sp.cos(k) + 8)
c2=-(2*f + 2*sp.sin(i)*sp.sin(j)*sp.cos(k) - 2*sp.sin(k)*sp.cos(i))/(g - sp.sqrt(3)*(h + sp.sin(i)*sp.cos(j)) + sp.sin(i)*sp.sin(j)*sp.sin(k) + sp.cos(i)*sp.cos(k) + 8)**2
c3=sp.sqrt(3)*(2*f + 2*sp.sin(i)*sp.sin(j)*sp.cos(k) - 2*sp.sin(k)*sp.cos(i))/(g - sp.sqrt(3)*(h + sp.sin(i)*sp.cos(j)) + sp.sin(i)*sp.sin(j)*sp.sin(k) + sp.cos(i)*sp.cos(k) + 8)**2
c4=(2*sp.sin(i)*sp.sin(k) + 2*sp.sin(j)*sp.cos(i)*sp.cos(k))/(g - sp.sqrt(3)*(h + sp.sin(i)*sp.cos(j)) + sp.sin(i)*sp.sin(j)*sp.sin(k) + sp.cos(i)*sp.cos(k) + 8) + (2*f + 2*sp.sin(i)*sp.sin(j)*sp.cos(k) - 2*sp.sin(k)*sp.cos(i))*(sp.sin(i)*sp.cos(k) - sp.sin(j)*sp.sin(k)*sp.cos(i) + sp.sqrt(3)*sp.cos(i)*sp.cos(j))/(g - sp.sqrt(3)*(h + sp.sin(i)*sp.cos(j)) + sp.sin(i)*sp.sin(j)*sp.sin(k) + sp.cos(i)*sp.cos(k) + 8)**2
c5=(-sp.sqrt(3)*sp.sin(i)*sp.sin(j) - sp.sin(i)*sp.sin(k)*sp.cos(j))*(2*f + 2*sp.sin(i)*sp.sin(j)*sp.cos(k) - 2*sp.sin(k)*sp.cos(i))/(g - sp.sqrt(3)*(h + sp.sin(i)*sp.cos(j)) + sp.sin(i)*sp.sin(j)*sp.sin(k) + sp.cos(i)*sp.cos(k) + 8)**2 + 2*sp.sin(i)*sp.cos(j)*sp.cos(k)/(g - sp.sqrt(3)*(h + sp.sin(i)*sp.cos(j)) + sp.sin(i)*sp.sin(j)*sp.sin(k) + sp.cos(i)*sp.cos(k) + 8)
c6=(-2*sp.sin(i)*sp.sin(j)*sp.sin(k) - 2*sp.cos(i)*sp.cos(k))/(g - sp.sqrt(3)*(h + sp.sin(i)*sp.cos(j)) + sp.sin(i)*sp.sin(j)*sp.sin(k) + sp.cos(i)*sp.cos(k) + 8) + (-sp.sin(i)*sp.sin(j)*sp.cos(k) + sp.sin(k)*sp.cos(i))*(2*f + 2*sp.sin(i)*sp.sin(j)*sp.cos(k) - 2*sp.sin(k)*sp.cos(i))/(g - sp.sqrt(3)*(h + sp.sin(i)*sp.cos(j)) + sp.sin(i)*sp.sin(j)*sp.sin(k) + sp.cos(i)*sp.cos(k) + 8)**2

d1=0
d2=-(h + sp.sqrt(3)*(g + sp.sin(i)*sp.sin(j)*sp.sin(k) + sp.cos(i)*sp.cos(k)) + sp.sin(i)*sp.cos(j))/(g - sp.sqrt(3)*(h + sp.sin(i)*sp.cos(j)) + sp.sin(i)*sp.sin(j)*sp.sin(k) + sp.cos(i)*sp.cos(k) + 8)**2 + sp.sqrt(3)/(g - sp.sqrt(3)*(h + sp.sin(i)*sp.cos(j)) + sp.sin(i)*sp.sin(j)*sp.sin(k) + sp.cos(i)*sp.cos(k) + 8)
d3=sp.sqrt(3)*(h + sp.sqrt(3)*(g + sp.sin(i)*sp.sin(j)*sp.sin(k) + sp.cos(i)*sp.cos(k)) + sp.sin(i)*sp.cos(j))/(g - sp.sqrt(3)*(h + sp.sin(i)*sp.cos(j)) + sp.sin(i)*sp.sin(j)*sp.sin(k) + sp.cos(i)*sp.cos(k) + 8)**2 + 1/(g - sp.sqrt(3)*(h + sp.sin(i)*sp.cos(j)) + sp.sin(i)*sp.sin(j)*sp.sin(k) + sp.cos(i)*sp.cos(k) + 8)
d4=(sp.sqrt(3)*(-sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i)) + sp.cos(i)*sp.cos(j))/(g - sp.sqrt(3)*(h + sp.sin(i)*sp.cos(j)) + sp.sin(i)*sp.sin(j)*sp.sin(k) + sp.cos(i)*sp.cos(k) + 8) + (h + sp.sqrt(3)*(g + sp.sin(i)*sp.sin(j)*sp.sin(k) + sp.cos(i)*sp.cos(k)) + sp.sin(i)*sp.cos(j))*(sp.sin(i)*sp.cos(k) - sp.sin(j)*sp.sin(k)*sp.cos(i) + sp.sqrt(3)*sp.cos(i)*sp.cos(j))/(g - sp.sqrt(3)*(h + sp.sin(i)*sp.cos(j)) + sp.sin(i)*sp.sin(j)*sp.sin(k) + sp.cos(i)*sp.cos(k) + 8)**2
d5=(-sp.sin(i)*sp.sin(j) + sp.sqrt(3)*sp.sin(i)*sp.sin(k)*sp.cos(j))/(g - sp.sqrt(3)*(h + sp.sin(i)*sp.cos(j)) + sp.sin(i)*sp.sin(j)*sp.sin(k) + sp.cos(i)*sp.cos(k) + 8) + (-sp.sqrt(3)*sp.sin(i)*sp.sin(j) - sp.sin(i)*sp.sin(k)*sp.cos(j))*(h + sp.sqrt(3)*(g + sp.sin(i)*sp.sin(j)*sp.sin(k) + sp.cos(i)*sp.cos(k)) + sp.sin(i)*sp.cos(j))/(g - sp.sqrt(3)*(h + sp.sin(i)*sp.cos(j)) + sp.sin(i)*sp.sin(j)*sp.sin(k) + sp.cos(i)*sp.cos(k) + 8)**2
d6=(-sp.sin(i)*sp.sin(j)*sp.cos(k) + sp.sin(k)*sp.cos(i))*(h + sp.sqrt(3)*(g + sp.sin(i)*sp.sin(j)*sp.sin(k) + sp.cos(i)*sp.cos(k)) + sp.sin(i)*sp.cos(j))/(g - sp.sqrt(3)*(h + sp.sin(i)*sp.cos(j)) + sp.sin(i)*sp.sin(j)*sp.sin(k) + sp.cos(i)*sp.cos(k) + 8)**2 + sp.sqrt(3)*(sp.sin(i)*sp.sin(j)*sp.cos(k) - sp.sin(k)*sp.cos(i))/(g - sp.sqrt(3)*(h + sp.sin(i)*sp.cos(j)) + sp.sin(i)*sp.sin(j)*sp.sin(k) + sp.cos(i)*sp.cos(k) + 8)

e1=2/(g - sp.sqrt(3)*(h + sp.cos(i)*sp.cos(j)) - sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i) + 8)
e2=-(2*f - 2*sp.sin(i)*sp.sin(k) + 2*sp.sin(j)*sp.cos(i)*sp.cos(k))/(g - sp.sqrt(3)*(h + sp.cos(i)*sp.cos(j)) - sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i) + 8)**2
e3=sp.sqrt(3)*(2*f - 2*sp.sin(i)*sp.sin(k) + 2*sp.sin(j)*sp.cos(i)*sp.cos(k))/(g - sp.sqrt(3)*(h + sp.cos(i)*sp.cos(j)) - sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i) + 8)**2
e4=(-2*sp.sin(i)*sp.sin(j)*sp.cos(k) - 2*sp.sin(k)*sp.cos(i))/(g - sp.sqrt(3)*(h + sp.cos(i)*sp.cos(j)) - sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i) + 8) + (2*f - 2*sp.sin(i)*sp.sin(k) + 2*sp.sin(j)*sp.cos(i)*sp.cos(k))*(sp.sin(i)*sp.sin(j)*sp.sin(k) - sp.sqrt(3)*sp.sin(i)*sp.cos(j) + sp.cos(i)*sp.cos(k))/(g - sp.sqrt(3)*(h + sp.cos(i)*sp.cos(j)) - sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i) + 8)**2
e5=(-sp.sqrt(3)*sp.sin(j)*sp.cos(i) - sp.sin(k)*sp.cos(i)*sp.cos(j))*(2*f - 2*sp.sin(i)*sp.sin(k) + 2*sp.sin(j)*sp.cos(i)*sp.cos(k))/(g - sp.sqrt(3)*(h + sp.cos(i)*sp.cos(j)) - sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i) + 8)**2 + 2*sp.cos(i)*sp.cos(j)*sp.cos(k)/(g - sp.sqrt(3)*(h + sp.cos(i)*sp.cos(j)) - sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i) + 8)
e6=(-sp.sin(i)*sp.sin(k) - sp.sin(j)*sp.cos(i)*sp.cos(k))*(2*f - 2*sp.sin(i)*sp.sin(k) + 2*sp.sin(j)*sp.cos(i)*sp.cos(k))/(g - sp.sqrt(3)*(h + sp.cos(i)*sp.cos(j)) - sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i) + 8)**2 + (-2*sp.sin(i)*sp.cos(k) - 2*sp.sin(j)*sp.sin(k)*sp.cos(i))/(g - sp.sqrt(3)*(h + sp.cos(i)*sp.cos(j)) - sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i) + 8)

f1=0
f2=-(h + sp.sqrt(3)*(g - sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i)) + sp.cos(i)*sp.cos(j))/(g - sp.sqrt(3)*(h + sp.cos(i)*sp.cos(j)) - sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i) + 8)**2 + sp.sqrt(3)/(g - sp.sqrt(3)*(h + sp.cos(i)*sp.cos(j)) - sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i) + 8)
f3=sp.sqrt(3)*(h + sp.sqrt(3)*(g - sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i)) + sp.cos(i)*sp.cos(j))/(g - sp.sqrt(3)*(h + sp.cos(i)*sp.cos(j)) - sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i) + 8)**2 + 1/(g - sp.sqrt(3)*(h + sp.cos(i)*sp.cos(j)) - sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i) + 8)
f4=(sp.sqrt(3)*(-sp.sin(i)*sp.sin(j)*sp.sin(k) - sp.cos(i)*sp.cos(k)) - sp.sin(i)*sp.cos(j))/(g - sp.sqrt(3)*(h + sp.cos(i)*sp.cos(j)) - sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i) + 8) + (h + sp.sqrt(3)*(g - sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i)) + sp.cos(i)*sp.cos(j))*(sp.sin(i)*sp.sin(j)*sp.sin(k) - sp.sqrt(3)*sp.sin(i)*sp.cos(j) + sp.cos(i)*sp.cos(k))/(g - sp.sqrt(3)*(h + sp.cos(i)*sp.cos(j)) - sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i) + 8)**2
f5=(-sp.sin(j)*sp.cos(i) + sp.sqrt(3)*sp.sin(k)*sp.cos(i)*sp.cos(j))/(g - sp.sqrt(3)*(h + sp.cos(i)*sp.cos(j)) - sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i) + 8) + (-sp.sqrt(3)*sp.sin(j)*sp.cos(i) - sp.sin(k)*sp.cos(i)*sp.cos(j))*(h + sp.sqrt(3)*(g - sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i)) + sp.cos(i)*sp.cos(j))/(g - sp.sqrt(3)*(h + sp.cos(i)*sp.cos(j)) - sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i) + 8)**2
f6=(-sp.sin(i)*sp.sin(k) - sp.sin(j)*sp.cos(i)*sp.cos(k))*(h + sp.sqrt(3)*(g - sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i)) + sp.cos(i)*sp.cos(j))/(g - sp.sqrt(3)*(h + sp.cos(i)*sp.cos(j)) - sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i) + 8)**2 + sp.sqrt(3)*(sp.sin(i)*sp.sin(k) + sp.sin(j)*sp.cos(i)*sp.cos(k))/(g - sp.sqrt(3)*(h + sp.cos(i)*sp.cos(j)) - sp.sin(i)*sp.cos(k) + sp.sin(j)*sp.sin(k)*sp.cos(i) + 8)

dmdf=float(a1.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dmdg=float(a2.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dmdh=float(a3.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dmdi=0
dmdj=float(a5.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dmdk=float(a6.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dndf=0
dndg=float(b2.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dndh=float(b3.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dndi=0
dndj=float(b5.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dndk=float(b6.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dodf=float(c1.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dodg=float(c2.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dodh=float(c3.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dodi=float(c4.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dodj=float(c5.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dodk=float(c6.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dpdf=0
dpdg=float(d2.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dpdh=float(d3.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dpdi=float(d4.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dpdj=float(d5.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dpdk=float(d6.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dqdf=float(e1.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dqdg=float(e2.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dqdh=float(e3.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dqdi=float(e4.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dqdj=float(e5.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
dqdk=float(e6.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
drdf=0
drdg=float(f2.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
drdh=float(f3.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
drdi=float(f4.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
drdj=float(f5.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))
drdk=float(f6.subs({f:x[0], g: x[1],h:x[2], i: x[3],j:x[4], k: x[5]}))


print(dmdf,dmdg,dmdh,dmdi,dmdj,dmdk,dndf,dndg,dndh,dndi,dndj,dndk,dodf,dodg,dodh,dodi,dodj,dodk,dpdf,dpdg,dpdh,dpdi,dpdj,dpdk,dqdf,dqdg,dqdh,dqdi,dqdj,dqdk,drdf,drdg,drdh,drdi,drdj,drdk)
