#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time

import sys

sys.path.append('..')
from geometry.geometry import *
from geometry.utilsbot import *
from geometry.Trajectory import Trajectory


def simularTrayectoria1(d):
    pltEscenario(10,10)
    wxr = np.array([5,1,norm_pi_deg(90)]) # pos inicial
    fps = 30 # para las animaciones
    # 1) girar 90º grados dcha (-90) sobre si mismo
    v = 0
    t = 2
    w = -math.pi/(2*t)
    wxr,_ = animarBot(wxr, [v,w], t, fps)
    # 2) Trayectoria circular con R=d y th=pi
    t = 4
    w = math.pi/t
    v = w * d
    wxr,_ = animarBot(wxr, [v,w], t, fps)
    # 3) Circular con R=-d y th = 2pi
    # misma v (v3=v2), w = -w
    w = -w
    t = 2*t
    wxr,_ = animarBot(wxr, [v,w], t, fps)
    # 4) = 2
    t = 4
    w = math.pi/t
    v = w * d
    wxr,_ = animarBot(wxr, [v,w], t, fps, last=True)



def Trayectoria1(d):
    """
    Devuelve la trayectoria 1 (secuencia de movimientos)
    """
    #wxr = np.array([5,1,norm_pi_deg(90)]) # pos inicial
    t1 = Trajectory()#wxr=wxr)
    # 1) girar 90º grados dcha (-90) sobre si mismo
    v = 0
    t = 1
    w = -math.pi/(2*t)
    t1.addMove([v, w], t)
    # 2) Trayectoria circular con R=d y th=pi
    t = 4
    w = math.pi/t
    v = w * d

    t1.addMove([v, w], t)
    # 3) Circular con R=-d y th = 2pi
    # misma v (v3=v2), w = -w
    w = -w
    t = 2*t

    t1.addMove([v, w], t)
    # 4) = 2
    t = 4
    w = math.pi/t
    v = w * d

    t1.addMove([v, w], t)
    return t1


def Trayectoria1Velocidades(d):
    """
    Devuelve la trayectoria 1 (secuencia de movimientos)
    """
    v = [0, math.pi/4*0.2,  math.pi/4*0.2, math.pi/4*0.2, math.pi/4*0.2]
    w = [math.pi/2, -math.pi/4, math.pi/4,math.pi/4, -math.pi/4]
    t1 = Trajectory()
    # 1) girar 90º grados dcha (-90) sobre si mismo
    pos1 = np.array([None, None, -math.pi / 2])
    # 2) primera semicircunferencia
    pos2 = np.array([None, None, math.pi / 2])
    pos23 = np.array([None, None, -math.pi / 2])
    # 3) circunferencia
    pos3 = np.array([None, None, math.pi / 2])
    # 4) segunda semicircunferencia
    pos4 = np.array([None, None, -math.pi / 2])
    t1.setTargetPositionsAndSpeeds([pos1, pos2, pos23, pos3, pos4], v, w)

    return t1

def Trayectoria2Velocidades(rIzq, rDcha, dist):
    """
    Devuelve la trayectoria 1 (secuencia de movimientos)
    """
    
    angDcha = norm_pi(np.arctan((rDcha-rIzq)/dist))
    vs = []
    ws = []
    v = 0
    w = math.pi/2
    
    vs += [v]
    ws += [w]
    # 2) w = pi/2-r1 (donde r1 es el angulo ese de la dcha)
    w = -(math.pi/2.0-angDcha)/2
    v = w * -rIzq #(donde a es el radio de la izq)
    vs += [v]
    ws += [w]
    # 3) w =0
    w=0
    v = dist / 4 #(donde r2 es la dist entre los dos tramos)
    vs += [v]
    ws += [w]
    # 4) t4 = 4s
    w = -(math.pi+2*angDcha)/3
    v = w*-rDcha
    vs += [v]
    ws += [w]
    # 5) = (3)
    w=0
    v = dist / 4 #(donde r2 es la dist entre los dos tramos)
    vs += [v]
    ws += [w]
    # 6) = (2)
    w = -(math.pi/2-angDcha)
    v = w * -rIzq #(donde a es el radio de la izq)
    vs += [v]
    ws += [w]
    t2 = Trajectory()
     # 0) girar 90º grados dcha (-90) sobre si mismo
    pos0 = np.array([None, None, -math.pi/2])
   
    x_temp = rIzq * np.sin(-math.pi/2 - angDcha)
    y_temp = rIzq * (1-np.cos(-math.pi/2 - angDcha))
    pos1 = np.array([None, None, angDcha])
    # 2) 
    x_temp += np.cos(angDcha)*dist
    y_temp += np.sin(angDcha)*dist
    pos2 = np.array([x_temp, y_temp, None])
    
    x_temp -= rDcha * np.sin(-math.pi/2 - angDcha)
    y_temp -= rDcha * (1-np.cos(-math.pi/2 - angDcha))
    pos3 = np.array([None, None, (-math.pi + angDcha)])
    
    pos4 = np.array([x_temp, y_temp, None])
    # 4) segunda semicircunferencia
    pos5 = np.array([None, None, math.pi / 2])
    #for i in range(len(vs)):
    #    vs[i] = vs[i]/3
    #    ws[i] = ws[i]/3
    t2.setTargetPositionsAndSpeeds([pos0,pos1, pos2, pos3, pos4,pos5], vs, ws)
    #print("wehe")
    #sys.exit(1)
    return t2


def simularTrayectoria2(rIzq, rDcha, angDcha, dist,fps=30):
    wxr = np.array([1,5,norm_pi_deg(0)]) # pos inicial
    # 1) v= 0, w = -w_circ_1_1
    v = 0
    t = 1
    w = math.pi/(2*t)
    wxr,_ = animarBot(wxr, [v,w], t, fps)
    # 2) w = pi/2-r1 (donde r1 es el angulo ese de la dcha)
    t = 1
    w = -(math.pi/2.0-angDcha)/t
    v = w * -rIzq #(donde a es el radio de la izq)
    wxr,_ = animarBot(wxr, [v,w], t, fps)
    # 3) w =0
    t = 2
    w=0
    v = dist / t #(donde r2 es la dist entre los dos tramos)
    wxr,_ = animarBot(wxr, [v,w], t, fps)
    # 4) t4 = 4s
    t = 3
    w = -(math.pi+2*angDcha)/t
    v = w*-rDcha
    wxr,_ = animarBot(wxr, [v,w], t, fps)
    # 5) = (3)
    t = 2
    w=0
    v = dist / t #(donde r2 es la dist entre los dos tramos)
    wxr,_ = animarBot(wxr, [v,w], t, fps)
    # 6) = (2)
    t = 1
    w = -(math.pi/2-angDcha)/t
    v = w * -rIzq #(donde a es el radio de la izq)
    wxr,_ = animarBot(wxr, [v,w], t, fps)


def Trayectoria2(rIzq, rDcha, dist,fps=30):
    wxr = np.array([1,5,norm_pi_deg(0)]) # pos inicial
    # angulo, trigonometria (tan(th)=opuesto/adyacente)
    # angDcha = norm_pi(np.arcsin((rDcha-rIzq)/dist))
    angDcha = norm_pi(np.arctan((rDcha-rIzq)/dist))
    # 1) v= 0, w = -w_circ_1_1
    t2 = Trajectory(wxr=wxr)
    v = 0
    t = 1
    w = math.pi/(2*t)
    t2.addMove([v,w], t)
    # 2) w = pi/2-r1 (donde r1 es el angulo ese de la dcha)
    t = 2
    w = -(math.pi/2.0-angDcha)/t
    v = w * -rIzq #(donde a es el radio de la izq)
    t2.addMove([v,w], t)
    # 3) w =0
    t = 4
    w=0
    v = dist / t #(donde r2 es la dist entre los dos tramos)
    t2.addMove([v,w], t)
    # 4) t4 = 4s
    t = 3
    w = -(math.pi+2*angDcha)/t
    v = w*-rDcha
    t2.addMove([v,w], t)
    # 5) = (3)
    t = 4
    w=0
    v = dist / t #(donde r2 es la dist entre los dos tramos)
    t2.addMove([v,w], t)
    # 6) = (2)
    t = 1
    w = -(math.pi/2.0-angDcha)/t
    v = w * -rIzq #(donde a es el radio de la izq)
    t2.addMove([v,w], t)
    return t2


def Trayectoria3(d, fps=30):
    wxr = np.array([1,5,norm_pi_deg(0)]) # pos inicial
    # angulo, trigonometria (tan(th)=opuesto/adyacente)
    # angDcha = norm_pi(np.arcsin((rDcha-rIzq)/dist))
    t3 = Trajectory(wxr=wxr)
    t = 5
    w = 0
    v = d/t
    t3.addMove([v,w], t)
    t = 2
    v = 0
    w = -math.pi/t
    t3.addMove([v,w], t)
    t = 5
    w = 0
    v = d/t
    t3.addMove([v,w], t)

    t = 2
    v = 0
    w = math.pi/t
    t3.addMove([v,w], t)
    return t3


def main(args):
    pltEscenario(10,10)
    if args.trayectoria == 1:
        d = 0.2
        t1 = Trayectoria1(d)
        t1.getEndPosition()
        # t1.draw()
    else:
        rI = 1.5
        rD = 2.5
        dist = 4
        # norm_pi_deg(10)
        t2 = Trayectoria2(rI, rD, dist)

        print(t2)
        print("end position: ", t2.getEndPosition())
        t2.draw()

if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--radioD", help="Radio to perform the 8-trajectory (mm)",
                        type=float, default=40.0)
    parser.add_argument("-t", "--trayectoria", help="Elige la trayectoria a dibujar (1 o 2)",
                                            type=int, default=1)
    args = parser.parse_args()

    main(args)
