"""
Vectors are np.array[x,y,th]
"""

import numpy as np
import math
import sys


# import pandas as pd
import matplotlib.pyplot as plt
 
""" def plotFile(fileName, sep="\t"):
    df  = pd.read_csv(fileName, sep=sep)
    print(df)
    #df.plot()  # plots all columns against index
    ax = df.plot(kind='scatter',x='x',y='y',
        xlim=[-1.5,0.2], ylim=[-0.3,0.3], title="Odometría - video 1",
        grid=True) # scatter plot
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect('equal')
    #df.plot(kind='density')  # estimate density function
    plt.show() """

def writeLog(f_log, data, sep="\t"):
    """
    Writes a row of the log (t, x, y, th, v, w, deltaTh, deltaSi)
    (each with 2 decimal positions, v multiplied by 100 to gain precision)
    """
    data = "\t".join(['{0:.3f}'.format(e) for e in data])+"\n"
    #print(fila)
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
    #print(A, "A, target", targetArea, "Resta: ", targetArea-A)
    # A - targetArea: Domain (-targetArea, 0) -> Range (0, vTarget-0.1) -> (v, 0.1)
    return vTarget-np.interp(A-targetArea, [-targetArea, 0], [0, vTarget-0.1])

def reached(x, target, greater):
    if greater:
        return x>=target
    else:
        return x<=target

def reachedAngle(th, target, w):
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


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("necesito el csv")
        exit(1)
    plotFile(sys.argv[1])
