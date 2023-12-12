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
    a=ct.robot.FK()
    print(a)
    m=np.array(a[:3])
    print(m)
    f=np.zeros(2)
    print(f)
    f[0]=m[1]
    print(f)