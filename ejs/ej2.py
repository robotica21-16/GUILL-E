import numpy as np
import matplotlib.pyplot as plt
import argparse

import pandas as pd

from geometry.utilsbot import *
from geometry.geometry import *

# from simubot import *

def plotVariables(log):
    df  = pd.read_csv(log, sep="\t")
    print(df)
    ax = df.plot(x = 't', y = 'v', marker='o')
    ax.set_xlabel("t (s)")
    ax.set_ylabel("v (m/s)")
    ax = df.plot(x = 't', y = 'w', marker='o')
    ax.set_xlabel("t (s)")
    ax.set_ylabel("w (rad/s)")

    ax = df.plot(x = 't', y = ["x", "y", "th"], marker='o')
    ax.set_xlabel("t (s)")
    ax.set_ylabel("x,y(m); th(rad)")
    plt.show()


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
        self.f = open('log.txt', 'w')
        self.f.write("\t".join(['t', 'v', 'w', 'x', 'y', 'th'])+"\n")

    def plot(self, args):
        if args.animate or args.plot_trajectory:
            pltbot = dibrobot(self.x, tamano='g')
            if args.animate:
                plt.pause(0.05) # borrar pause y remove para tener directamente el final
                pltbot.remove()

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

    def log(self, vc, t):
        l = [t, vc[0], vc[1], self.x[0], self.x[1],self.x[2]]
        l = ['{0:.3f}'.format(s) for s in l]

        l = "\t".join(l)
        self.f.write(l+"\n")

def checkVC(vc):
    if vc[0] >= MAXV:
        print("v mal: ", vc[0], ">=", MAXV, "!!!!!")
    if vc[1] >= MAXW:
        print("w mal: ", vc[1], ">=", MAXW, "!!!!!")

def fixVC(vc, maxv, maxw):
    if vc[0] > maxv:
        rel = vc[1]/vc[0] # relacion entre v y w, se debe mantener
        vc[0] = maxv
        vc[1] = vc[0]*rel
        #print("v mal: ", vc[0], ">=", MAXV, "!!!!!")
    if abs(vc[1]) > maxw:
        rel = vc[0]/vc[1] # relacion entre v y w, se debe mantener
        vc[1] = maxw if vc[1] > 0 else -maxw
        vc[0] = vc[1]*rel
        # print("w mal: ", vc[1], ">=", MAXW, "!!!!!")
    return vc

def main(args):
    if args.plotlog=="": # no se hace el plot de las variables
        # Matriz de control:
        kp =0.5
        ka = kb = 4.0
        K = np.array([
            [kp, 0.0, 0.0],
            [0.0,  ka, kb]
        ])
        # Puntos de la ruta:
        ptos = [(2,3,90), (6,4,45), (10,5,-45), (7,-3,180), (2,3,90)]
        ptos = [np.array(pto) for pto in ptos]
        # plot points:
        for i in range(len(ptos)):
            pto = ptos[i]
            ptos[i] = [float(e) for e in pto]
            ptos[i][2] = norm_pi(math.radians(ptos[i][2]))
            print(pto)
            dibPunto(pto)

        # Bucle:
        rbt = RobotSim(K)
        periodo = 0.3
        t = 0.0
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
                    vc = fixVC(vc, MAXV, MAXW)
                    x, xr = simubot(vc,rbt.x,periodo)
                    rbt.setPos(x)
                    rbt.plot(args) #plot position
                    rbt.log(vc, t)
                    t+=periodo
                    first = False
        if args.plot_trajectory:
            plt.show()
    else: # plot log
        plotVariables(args.plotlog)




if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument('-a','--animatebot', help="animates the trajectory", dest='animate', action='store_true')
    # parser.add_argument('-na', '--no-animatebot', dest='animate', action='store_false')
    parser.set_defaults(animate=False)
    parser.add_argument('-pt','--plottrajectory', help="shows the whole trajectory (no animation)", dest='plot_trajectory', action='store_true')
    # parser.add_argument('-npt', '--no-plottrajectory', dest='plot_trajectory', action='store_false')
    parser.set_defaults(plot_trajectory=False)
    parser.add_argument("-pl", "--plotlog", help="plots the variables from the given log",
                        default="")
    args = parser.parse_args()
    main(args)
