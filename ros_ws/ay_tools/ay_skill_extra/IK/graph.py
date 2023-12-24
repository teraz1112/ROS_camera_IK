#!/usr/bin/python
from core_tool import *
import math
import copy
import numpy as np
import tf.transformations as tft
import matplotlib.pyplot as plt
import numpy as np


def Help():
  return '''First script.
  Usage: A.first4'''


def Run(ct,*args):
  euler=[-np.pi/3,0,0]
  R=tft.euler_matrix(euler[0],euler[1],euler[2])
  R_inv=np.linalg.pinv(R)
  print(R_inv)