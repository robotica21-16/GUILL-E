"""
Vectors are np.array[x,y,th]
"""

import numpy as np
import math



def mapf(val, mind, maxd, minr, maxr):
    """
    Cambia val del dominio [mind,maxd] al rango [minr,maxr]
    """
    return (val - mind) * (maxr - minr) / (maxd - mind) + minr


def norm_pi_deg(th):
    """
    Normaliza el angulo th (en grados) a 0..PI o 0..-PI (-PI..+PI)
    """
    # Nota: crrreo que se podria hacer mas eficiente evitando el if/else
    deg = th%360 # se evitan vueltas de mas
    if deg <= 180: # mitad superior
        return mapf(deg, 0, 180, 0, math.pi)
    else: # inferior
        return mapf(deg, 180, 360, -math.pi, 0)


def norm_pi(th):
    """
    Normaliza el angulo th (en grados) a 0..PI o 0..-PI (-PI..+PI)
    """
    # Nota: crrreo que se podria hacer mas eficiente evitando el if/else
    th_out = th % (2.0 * math.pi)  # se evitan vueltas de mas
    if th_out <= math.pi: # mitad superior
        return th_out # mapf(deg, 0, 180, 0, math.pi)
    else: # inferior
        return -(2*math.pi-th_out)# mapf(deg, 180, 360, -math.pi, 0)



def rot(th):
    """
    Devuelve la matriz de rotacion de th
    """
    return np.array([[np.cos(th), -np.sin(th), 0],
                     [np.sin(th),  np.cos(th), 0],
                     [0, 0, 1]])

def rotDeg(deg):
    """
    Devuelve la matriz de rotacion para deg grados
    """
    return rot(norm_pi(deg))

def loc(T):
    """
    Devuelve x a partir de T
    """
    #print(T[0,2], T[1,2])
    return np.array([T[0,2], T[1,2], math.atan2(T[1,0],T[0,0])]) # TODO: test

# TODO: test
def hom(x):
    """
    In: x np.array(3x1)
    Devuelve la matriz T (3x3) que transforma el origen en x
    """
    th = x[2]
    return np.array([[np.cos(th), -np.sin(th), x[0]],
                     [np.sin(th),  np.cos(th), x[1]],
                     [0, 0, 1]])


def trayectoriaCircular(r, th):
    """
    Devuelve el punto relativo al inicial tras girar el angulo th con radio r
    """
    return np.array([r * np.sin(th), r * (1-np.cos(th)), th])


def trayectoriaCircularDeg(r, deg):
    """
    Devuelve el punto relativo al inicial tras girar el angulo th (GRADOS) con radio r
    """
    return trayectoriaCircular(r, norm_pi(deg))

def radioYThTrayectoriaCirc(xFin):
    """
    Devuelve el radio y el angulo th necesarios para llegar a xFin (local) en un arco de th grados
    """
    x, y = xFin[0], xFin[1]
    return ((x**2+y**2)/(2.0*y), # Radio
    math.atan2(2.0*x*y, x**2-y**2)) # angulo

def wVelFromVR(v, r):
    """
    Devuelve la velocidad angular a partir de v y r
    """
    return v/r


def longArco(r, th):
    """
    Devuelve la longitud del arco de angulo th y radio r, r*th
    """
    return r*th


def fromPosToTarget(pos, target, vTarget, wTarget, eps=0.001):
    pos_target = loc(np.dot(hom(pos), hom(target))) # target en coord de pos
    r, th = radioYThTrayectoriaCirc(pos_target)
    if -eps<r<eps: # r close to 0
        v = 0
        w = wTarget if (pos_target[2])>0 else -wTarget
    else:
        v = vTarget
        w = vTarget/r
    return v, w


def vWiFromIzqDcha(r, L, wI, wD):
    """
    Devuelve [v,w] a partir del radio de la rueda <r>, la dist entre ruedas <L>,
    y las velocidades angulares en rad/s de las ruedas (wI, wD)
    """
    return np.dot(
        np.array([[r/2, r/2],
             [r/L, -r/L]]),
        np.array([wD, wI]))


def izqDchaFromVW(r, L, v, w):
    """
    r: radio de las ruedas
    L: dist entre ruedas
    v: v lineal
    w: v angular
    devuelve [wD, wI]
    """
    return np.dot(
        np.array([[1/r, L/(2*r)],
                [1/r, -L/(2*r)]]),
        np.array([v, w]))



def horizontalDistance(kp, obj=[0,0]):
    return -kp.pt[0]+obj[0]
