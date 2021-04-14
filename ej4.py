import numpy as np
import matplotlib.pyplot as plt
import argparse

import pandas as pd

from geometry.utilsbot import *
from geometry.geometry import *

from robotsim import RobotSim, plotVariables



def dibPunto(w_x_p):
    plt.plot(w_x_p[0], w_x_p[1], 'o')
    #plt.plot(w_x_p[1], w_x_p[0], 'o')

def checkVC(vc, MAXV=3.0, MAXW=3.0):
    if vc[0] >= MAXV:
        print("v mal: ", vc[0], ">=", MAXV, "!!!!!")
    if abs(vc[1]) >= MAXW:
        print("w mal: ", vc[1], ">=", MAXW, "!!!!!")


def main(args):
    if args.plotlog=="": # no se hace el plot de las variables
        # Matriz de control:
        # kp =0.3
        # ka = kb = 4.0
        kp = 3.0#0.3
        ka = 0.05
        kb = -2.1
        K = np.array([
            [kp, 0.0, 0.0],
            [0.0,  ka, kb]
        ])
        # Pared:
        yPared = 0
        dConsigna = 1
        fig = plt.figure(figsize=(40,5))

        ax = fig.add_subplot(111)
        ax.set_xlim(-2,40)
        ax.set_ylim(-5,5)
        ax.set_aspect('equal')
        ax.grid(True)
        plt.axhline(y=yPared, color='b', linestyle='-')

        # Robot:
        rbt = RobotSim(K, x=np.array([0.0,1.5,math.pi/7]), dc = dConsigna)


        periodo = 0.05
        t = 0.0
        rotateSpeed = np.array([0, 0.5])
        eps = 0.05

        reached = 0
        while reached <= 0:
            # plot
            rbt.plot(args) #plot position
            # "sensor":
            d,reached  = rbt.sensorSim(yPared)
            if reached == -1:
                vc = rotateSpeed
            elif reached == -2:
                vc = -rotateSpeed
            elif reached == 0:
                vc = rbt.vcFromD(d)
                checkVC(vc)
            # Actualizar bot:
            x, xr = simubot(vc,rbt.x,periodo)
            rbt.setPos(x)
            rbt.log(vc, t)
            t+=periodo
            # TODO: lo de la pared "frontal"

        t0 = t

        while reached == 1 and t - t0 < 2:
            rbt.sensorSim(yPared)
            # plot
            rbt.plot(args, c='b') #plot position
            x, xr = simubot(vc,rbt.x,periodo)
            rbt.setPos(x)
            rbt.log(vc, t)
            t+=periodo


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
