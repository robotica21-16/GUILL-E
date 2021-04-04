"""
Vectors are np.array[x,y,th]
"""

import numpy as np
import math
import sys


def writeLog(f_log, data, sep="\t"):
    """
    Writes a row of the log (t, x, y, th, v, w, deltaTh, deltaSi)
    (each with 2 decimal positions, v multiplied by 100 to gain precision)
    """
    data = "\t".join(['{0:.3f}'.format(e) for e in data])+"\n"
    f_log.write(data)
    f_log.flush()

def getMappedW(wTarget, d, dmin, dmax, eps=15):
    """
    Returns a mapped angular speed in (-wTarget, wTarget), given a distance 'd' in pixels and its POSIBLE range '(dmin, dmax)'
    """
    if (abs(d)<eps):
        return 0
    return np.interp(d, [dmin, dmax], [-wTarget, wTarget])

def getMappedV(vTarget, A, targetArea):
    """
    Returns a mapped linear speed, given a value 'A' in pixels and its DESIRED value 'targetArea'
    """
    # cuando A=0 (en el infinito) -> targetArea-A = targetArea -> v=vmax
    # cuando A=a (en el objetivo) -> targetArea-A = 0 -> v = 0
    # A - targetArea: Domain (-targetArea, 0) -> Range (0, vTarget-0.1) -> (v, 0.1)
    return vTarget-np.interp(A-targetArea, [-targetArea, 0], [0, vTarget-0.1])


def reached(x, target, greater):
    """
    Returns true if x has reached the target according to greater
    """
    if greater:
        return x>=target
    else:
        return x<=target

def reachedAngle(th, target, w):
    """
    Returns true if th has reached the target angle (both in rad),
    taking into account the sign of w
    """
    if w > 0:
        if target < 0 and th > 0:
            return th >= (2*math.pi + target)
        else:
            return th >= target
    elif w < 0:
        if target > 0 and th < 0:
            return (2*math.pi+th) <= target
        else:
            return th <= target
