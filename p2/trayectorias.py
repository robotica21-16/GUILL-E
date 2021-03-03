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
    t = 1
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
    wxr = np.array([5,1,norm_pi_deg(90)]) # pos inicial
    t1 = Trajectory(wxr=wxr)
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
    t = 1
    w = -(math.pi/2.0-angDcha)/t
    v = w * -rIzq #(donde a es el radio de la izq)
    t2.addMove([v,w], t)
    # 3) w =0
    t = 1.5
    w=0
    v = dist / t #(donde r2 es la dist entre los dos tramos)
    t2.addMove([v,w], t)
    # 4) t4 = 4s
    t = 3
    w = -(math.pi+2*angDcha)/t
    v = w*-rDcha
    t2.addMove([v,w], t)
    # 5) = (3)
    t = 1.5
    w=0
    v = dist / t #(donde r2 es la dist entre los dos tramos)
    t2.addMove([v,w], t)
    # 6) = (2)
    t = 1
    w = -(math.pi/2.0-angDcha)/t
    v = w * -rIzq #(donde a es el radio de la izq)
    t2.addMove([v,w], t)
    return t2

def main(args):
    pltEscenario(10,10)
    if args.trayectoria == 1:
        d = 2
        t1 = Trayectoria1(d)
        t1.draw()
    else:
        rI = 1.5
        rD = 2.5
        dist = 4
        # norm_pi_deg(10)
        t2 = Trayectoria2(rI, rD, dist)

        print(t2)
        print("end position: ", t2.getEndPosition())
        t2.draw()
    #
    # pltEscenario(10,10)
    # # d = 2
    # # simularTrayectoria1(d)
    # rI = 1.5
    # rD = 3
    # dist = 4
    # angDcha = norm_pi_deg(10)
    # simularTrayectoria2(rI, rD,angDcha, dist)

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
