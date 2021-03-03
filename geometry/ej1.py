import numpy as np
import matplotlib.pyplot as plt
from utilsbot import *
from geometry import *
# from simubot import *

def printsep(s, sep="------------"):
    print(sep, s, sep)

def robotFromPuerta(rxp, wxp):
    """
    Devuelve wxr (pos del robot global) a partir de la pos global de la puerta
    (wxp) y su pos relativa respecto al robot (rxp)
    """
    # return wxp-np.linalg.inv(hom(rxp))
    return loc(np.dot(hom(wxp), np.linalg.inv(hom(rxp))))
    #return loc(np.dot(hom(rxp),wxp))


def dibPunto(w_x_p):
    plt.plot(w_x_p[0], w_x_p[1], 'o')
    #plt.plot(w_x_p[1], w_x_p[0], 'o')


if __name__ == '__main__':
    # ------------------------- 1 -------------------------------
    printsep('Apartado 1')
    wxp = np.array([2.5, 10, 0]) # pos global de la puerta
    rxp = np.array([5.2,-3,norm_pi_deg(-125)]) # pos de la puerta para el robot
    wxr = robotFromPuerta(rxp, wxp) # pos robot en mundo
    print('Robot en:', wxr)
    pltEscenario()
    # dibrobot(wxr, tamano='g')
    dibPunto(wxp)
    # plt.show()
    # exit(0)
    # ------------------------- 2 -------------------------------
    printsep('Apartado 2')
    rxc = np.array([4.17,0.73,norm_pi_deg(-35)]) # pos del cuadro para el robot
    wxc = loc(np.dot((hom(wxr)),hom(rxc))) # cuadro en mundo
    print('Cuadro en:', wxc)
    dibPunto(wxc)
    # plt.show()
    # ------------------------- 3 -------------------------------
    printsep('Apartado 3')
    pxc = loc(np.dot(np.linalg.inv(hom(wxp)),hom(wxc))) # cuadro para puerta
    print('Cuadro para la puerta:', pxc)
    # ------------------------- 4 -------------------------------
    printsep('Apartado 4')
    t = 8 # 8 segundos
    r, th = radioYThTrayectoriaCirc(rxp) # radio y th
    # print(r, th)
    long = longArco(r, th) # longitud del arco = r*th
    #print(long)
    v = long / t # velocidad tangencial, distancia/t
    w = wVelFromVR(v, r) # vel angular, r/v
    print('Velocidad lineal, angular:', v, ",", w)
    posrel = trayectoriaCircular(r, th)#
    print('Localizacion final relativa:', posrel)
    print('Localizacion final w:', loc(np.dot(hom(wxr), hom(posrel))))

    # ------------------------- 5 -------------------------------
    printsep('Apartado 5')

    print('A los 4 segundos:')
    posW, posR = simubot([v,w], wxr, 4)
    print('\tLocalizacion en W:', posW,
        '\n\trelativa:         ', posR)

    print('Cada segundo del movimiento:')
    fps = 4
    for i in range(fps*(t)+1):
        posW, posR = simubot([v,w], wxr, i/fps)
        if (i % fps == 0):
            print('t =', i/fps, 's:')
            print('\tLocalizacion en W:', posW,
                '\n\trelativa:         ', posR)

        pltbot = dibrobot(posW, tamano='g')
        plt.pause(1/fps)
        #print(pltbot)
        if i/fps<t:
            pltbot.remove()

    plt.show()
