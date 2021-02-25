import numpy as np
import math


from geometry import *


# Simula movimiento del robot con vc=[v,w] en t seg. desde xWR
def simubot(vc,xWR,t):
    """
    devuelve xWR, xRk
    """
    if vc[1]==0:   # w=0
      xRk=np.array([vc[0]*t, 0, 0])
    else:
      R=vc[0]/vc[1]
      dtitak=vc[1]*t
      titak=norm_pi(dtitak);
      xRk=np.array([R*np.sin(titak), R*(1-np.cos(titak)), titak])

    xWRp=loc(np.dot(hom(xWR),hom(xRk)))   # nueva localizaci�n xWR
    return xWRp, xRk
