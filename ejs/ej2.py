import numpy as np
import matplotlib.pyplot as plt
import argparse


from geometry.utilsbot import *
from geometry.geometry import *

from robotsim import RobotSim, plotVariables

# from simubot import *

MAXV = MAXW = 3.0

def dibPunto(w_x_p):
    plt.plot(w_x_p[0], w_x_p[1], 'o')

def checkVC(vc):
    if vc[0] >= MAXV:
        print("v mal: ", vc[0], ">=", MAXV, "!!!!!")
    if vc[1] >= MAXW:
        print("w mal: ", vc[1], ">=", MAXW, "!!!!!")

def fixVC(vc, maxv, maxw):
    if vc[0] > maxv:
        # print("v mal: ", vc[0], ">=", MAXV, "!!!!!")

        rel = vc[1]/vc[0] # relacion entre v y w, se debe mantener
        vc[0] = maxv
        vc[1] = vc[0]*rel
    if abs(vc[1]) > maxw:
        # print("w mal: ", vc[1], ">=", MAXW, "!!!!!")

        rel = vc[0]/vc[1] # relacion entre v y w, se debe mantener
        vc[1] = maxw if vc[1] > 0 else -maxw
        vc[0] = vc[1]*rel
    return vc

def main(args):
    if args.plotlog=="": # no se hace el plot de las variables
        # Matriz de control:
        kp = 0.3
        ka = 2.0
        kb = 0.5
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
                    # if args.vcte:
                    #     # rel = vc[1]/vc[0]
                    #     vc[0] = 0.1
                        # vc[1] = vc[0]*rel
                    x, xr = simubot(vc,rbt.x,periodo)
                    rbt.setPos(x)
                    rbt.plot(args) #plot position
                    rbt.log(vc, t, G_X_R)
                    t+=periodo
                    first = False
                    # plt.show()
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
    parser.add_argument('-vcte','--vcte', help="keeps v constant", dest='vcte', action='store_true')
    # parser.add_argument('-na', '--no-animatebot', dest='animate', action='store_false')
    parser.set_defaults(vcte=False)
    parser.add_argument('-pt','--plottrajectory', help="shows the whole trajectory (no animation)", dest='plot_trajectory', action='store_true')
    # parser.add_argument('-npt', '--no-plottrajectory', dest='plot_trajectory', action='store_false')
    parser.set_defaults(plot_trajectory=False)
    parser.add_argument("-pl", "--plotlog", help="plots the variables from the given log",
                        default="")
    args = parser.parse_args()
    main(args)
