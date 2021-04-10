import numpy as np
import matplotlib.pyplot as plt
import argparse

import pandas as pd

from geometry.utilsbot import *
from geometry.geometry import *

from robotsim import RobotSim



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
        ka = 1.5
        kb = -1.0
        K = np.array([
            [kp, 0.0, 0.0],
            [0.0,  ka, kb]
        ])
        # Pared:
        yPared = 0
        dConsigna = 1
        plt.axhline(y=yPared, color='b', linestyle='-')

        # Robot:
        rbt = RobotSim(K, x=np.array([0.0,3.0,math.pi/4]), dc = dConsigna)


        periodo = 0.05
        t = 0.0
        eps = 0.25

        reached = False
        while not reached:
            # plot
            rbt.plot(args) #plot position
            # "sensor":
            d,reached  = rbt.sensorSim(yPared)
            if not reached:
                vc = rbt.vcFromD(d)
                checkVC(vc)
                # Actualizar bot:
                x, xr = simubot(vc,rbt.x,periodo)
                rbt.setPos(x)
                rbt.log(vc, t)
                t+=periodo
            # TODO: lo de la pared "frontal"


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
