import numpy as np
import math
import matplotlib.pyplot as plt


from geometry.geometry import *


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


# Dibuja robot en location_eje con color (c) y tamano (p/g)
def dibrobot(loc_eje,c='r',tamano='p'):
    if tamano=='p':
        largo=0.1
        corto=0.05
        descentre=0.01
    else:
        largo=0.5
        corto=0.25
        descentre=0.05

    trasera_dcha=np.array([-largo,-corto,1])
    trasera_izda=np.array([-largo,corto,1])
    delantera_dcha=np.array([largo,-corto,1])
    delantera_izda=np.array([largo,corto,1])
    frontal_robot=np.array([largo,0,1])
    tita=loc_eje[2]
    Hwe=np.array([[np.cos(tita), -np.sin(tita), loc_eje[0]],
             [np.sin(tita), np.cos(tita), loc_eje[1]],
              [0,        0 ,        1]])
    Hec=np.array([[1,0,descentre],
              [0,1,0],
              [0,0,1]])
    extremos=np.array([trasera_izda, delantera_izda, delantera_dcha, trasera_dcha, trasera_izda, frontal_robot, trasera_dcha])
    robot=np.dot(Hwe,np.dot(Hec,np.transpose(extremos)))
    return plt.plot(robot[0,:], robot[1,:], c)[0]

  # plt.show()

def animarBot(wxr, vc, t, fps=10, playSpeed=1, last=True):
    """
    Dibuja el robot en distintas posiciones a partir de wxr (global) en f de
    vc=[v,w], en t segundos con fps.
    """
    # t = t/5
    fps=fps
    for i in range(int(fps*(t))):
        posW, posR = simubot(vc, wxr, i/fps)
        # if (i % fps == 0):
        #     print('t =', i/fps, 's:')
        #     print('\tLocalizacion en W:', posW,
        #         '\n\trelativa:         ', posR)

        pltbot = dibrobot(posW, tamano='g')
        plt.pause(1.0/(playSpeed*float(fps)))
        #print(pltbot)
        if not last or (last and i/fps<=t) :
            pltbot.remove()
    return posW, posR



def pltEscenario(dimx=6, dimy=11):
    fig1, ax = plt.subplots()

    ax.set_xlim(0, dimx)
    ax.set_ylim(0, dimy)

    # frecuencia de los labels:
    ax.set_xticks(np.arange(0, dimx, 1))
    ax.set_yticks(np.arange(0, dimy, 1))

    ax.set_aspect('equal')
    # ax.set_box_aspect(2)
    # Cuadricula:
    plt.grid(True)
