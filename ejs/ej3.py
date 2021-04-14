import numpy as np
import matplotlib.pyplot as plt
import argparse


from geometry.utilsbot import *
from geometry.geometry import *

from robotsim import RobotSim, plotVariables

# from simubot import *


MAXV = MAXW = 3.0

class Movil:
    """
    Movil con una posicion variable y unas velocidades constantes
    """
    def __init__(self, vc, x=np.array([0.0,0.0,0.0]), fijarEjes=True):
        self.x = x
        self.vc = vc
        fig = plt.figure()
        self.ax = fig.add_subplot(111)
        if fijarEjes:
            self.ax.set_xlim(-11,11)
            self.ax.set_ylim(-11,11)
            self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.lastFrame = None

    def simular(self, t):
        self.x, _ = simubot(self.vc,self.x,t)

    def plot(self, animar):
        if animar and self.lastFrame is not None:
            self.lastFrame.remove()
            # pass
        self.lastFrame = dibrobot(self.x, tamano='g', c='b')
        # self.last = self.ax.plot(self.x[0], self.x[1], 'o')

        # ax.set_aspect('equal')


def checkVC(vc):
    if vc[0] >= MAXV:
        print("v mal: ", vc[0], ">=", MAXV, "!!!!!")
    if vc[1] >= MAXW:
        print("w mal: ", vc[1], ">=", MAXW, "!!!!!")

def fixVC(vc, maxv, maxw):
    if vc[0] > maxv:
        print("v mal: ", vc[0], ">=", MAXV, "!!!!!")
        rel = vc[1]/vc[0] # relacion entre v y w, se debe mantener
        vc[0] = maxv
        vc[1] = vc[0]*rel
    if abs(vc[1]) > maxw:
        print("w mal: ", vc[1], ">=", MAXW, "!!!!!")
        rel = vc[0]/vc[1] # relacion entre v y w, se debe mantener
        vc[1] = maxw if vc[1] > 0 else -maxw
        vc[0] = vc[1]*rel
    return vc

def main(args):
    if args.plotlog=="": # no se hace el plot de las variables
        # Matriz de control:

        kp = 0.5#0.3
        ka = 2.0
        kb = 0.5
        K = np.array([
            [kp, 0.0, 0.0],
            [0.0,  ka, kb]
        ])
        # Plot:
        fijarEjes = True
        animar = args.animate

        # Robot:

        xBot = np.array([0.0,-5.0,-math.pi/2.0])
        rbt = RobotSim(K, x=xBot)
        # movil:
        vmovil = 2 # 2 m/s
        rmovil = 10 # 10 m radio
        wmovil = vmovil/rmovil
        xmovil = np.array([0,-10,0])
        movil = Movil(np.array([vmovil, wmovil]), xmovil, fijarEjes)
        periodo = 0.5
        t = 0.0
        eps = 0.25
        # first = True
        first = False # poner a True para actualizar K para cada goal (sale mal)
        reached = False
        while not reached:
            # plot
            rbt.plot(args) #plot position
            # actualizar movil:
            movil.simular(periodo)
            # pos robot relativa a movil:
            M_X_R = loc(np.dot(np.linalg.inv(hom(movil.x)), hom(rbt.x)))

            movil.plot(animar)
            # nuevas velocidades:
            rbt.setObjetivo(M_X_R)
            vc = rbt.control()
            vc = fixVC(vc, MAXV, MAXW)
            # Actualizar bot:
            x, xr = simubot(vc,rbt.x,periodo)
            rbt.setPos(x)
            rbt.log(vc, t, M_X_R)
            t+=periodo
            # if animar:
            #     plt.pause(periodo*0.01)

            # condicion de fin:
            M_X_R = loc(np.dot(np.linalg.inv(hom(movil.x)), hom(rbt.x)))
            # print(M_X_R)
            reached = abs(M_X_R[0]) < eps and abs(M_X_R[1]) < eps

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
