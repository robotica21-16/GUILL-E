import numpy as np
import matplotlib.pyplot as plt
import argparse
from geometry.utilsbot import *
from geometry.geometry import *
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

MAXV = MAXW = 3.0

def dibPunto(w_x_p):
    plt.plot(w_x_p[0], w_x_p[1], 'o')
    #plt.plot(w_x_p[1], w_x_p[0], 'o')

class RobotSim:
    def __init__(self,K,x=np.array([0.0,0.0,0.0])):
        self.x = x
        self.K = K

    def plt(self):
        dibrobot(self.x, tamano='g')

    def setObjetivo(self, goal, updateK=False):
        if updateK:
            self.objetivo, self.K = toPolaresUpdateK(goal, MAXV, MAXW)
        else:
            self.objetivo = toPolares(goal)
    # def updatePos(vw):
    #     M = np.array([np.cos(th), 0],
    #                 [np.sin(th), 0],
    #                 [0, 1])
    #     self.x =

    def updateObjetivo(self,vw):
        alfa = self.objetivo[1]
        p = self.objetivo[0]
        M = np.array([-np.cos(alfa), 0],
                    [np.sin(alfa)/p, -1],
                    [np.sin(alfa)/p, 0])
        self.objetivo = np.dot(M, vw)

    def control(self):
        """
        Devuelve [v,w]
        """
        return np.dot(self.K, self.objetivo)

    def setPos(self,x):
        self.x=x

def checkVC(vc):
    if vc[0] >= MAXV:
        print("v mal: ", vc[0], ">=", MAXV, "!!!!!")
    if vc[1] >= MAXW:
        print("w mal: ", vc[1], ">=", MAXW, "!!!!!")


def main(args):
    # Matriz de control:
    kp =0.25
    ka = kb = 1.0
    K = np.array([
        [kp, 0.0, 0.0],
        [0.0,  ka, kb]
    ])
    # Puntos de la ruta:
    ptos = [(2,3,90), (6,4,45), (10,5,-45), (7,-3,180), (2,3,90)]
    ptos = [np.array(pto) for pto in ptos]

    for i in range(len(ptos)):
        pto = ptos[i]
        ptos[i] = [float(e) for e in pto]
        ptos[i][2] = norm_pi(math.radians(ptos[i][2]))
        print(pto)
        dibPunto(pto)
    # plt.show()




    # Bucle:

    rbt = RobotSim(K)
    t = 0.3
    instante = 0.0
    eps = 0.01
    for goal in ptos:
        # first = True
        first = False # poner a True para actualizar K para cada goal (sale mal)
        reached = False
        while not reached:
            G_X_R = loc(np.dot(np.linalg.inv(hom(goal)), hom(rbt.x)))
            # print("-------------------")
            # print(goal, rbt.x, G_X_R, sep="\n")
            # print("-------------------")
            reached = abs(G_X_R[0]) < eps and abs(G_X_R[1]) < eps
            if not reached:
                rbt.setObjetivo(G_X_R, first)
                vc = rbt.control()
                checkVC(vc)
                x, xr = simubot(vc,rbt.x,t)
                pltbot = dibrobot(x, tamano='g')

                plt.pause(0.1)
                pltbot.remove()
                # x, xr = animarBot(rbt.x,vc,100)
                rbt.setPos(x)
                #print(x)
                first = False





if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mapfile", help="path to find map file",
                        default="mapa1.txt")
    parser.add_argument("-g", "--test_go", help="test go function",
                        default="")
    parser.add_argument("-u", "--test_ultrasound", help="test ultrasound sensor",
                        default=False)
    args = parser.parse_args()
    main(args)
