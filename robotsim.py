import numpy as np
import math

from geometry.utilsbot import *
class RobotSim:
    def __init__(self,K,x=np.array([0.0,0.0,0.0]), dc=1):
        self.x = x
        self.K = K
        self.f = open('log.txt', 'w')
        self.f.write("\t".join(['t', 'v', 'w', 'x', 'y', 'th', 'xr', 'yr', 'thr'])+"\n")
        self.MAXV = self.MAXW = 3.0
        # ej4:
        self.deltad = 0
        self.dc = dc
        self.lastd = 0 #TODO: Revisar

    def plot(self, args, c='r'):
        if args.animate or args.plot_trajectory:
            pltbot = dibrobot(self.x, c=c, tamano='g')
            if args.animate:
                plt.pause(0.002) # borrar pause y remove para tener directamente el final
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

    def log(self, vc, t, rels=None):
        l = [t, vc[0], vc[1], self.x[0], self.x[1],self.x[2]]
        if rels is not None:
            l += [rels[0], rels[1],rels[2]]
        l = ['{0:.3f}'.format(s) for s in l]

        l = "\t".join(l)
        self.f.write(l+"\n")

    def fixVC(self, vc):
        if vc[0] > self.MAXV:
            rel = vc[1]/vc[0] # relacion entre v y w, se debe mantener
            vc[0] = self.MAXV
            vc[1] = vc[0]*rel
        if abs(vc[1]) > self.MAXW:
            rel = vc[0]/vc[1] # relacion entre v y w, se debe mantener
            vc[1] = self.MAXW if vc[1] > 0 else -self.MAXW
            vc[0] = vc[1]*rel
        return vc


    # EJ 4:
    def vcFromD(self, d):
        v = self.MAXV/2
        k1 = self.K[1,1]
        k2 = self.K[1,2]
        self.deltad = d-self.lastd
        # print("Deltad: ", self.deltad)
        # print("Ant.d: ", self.lastd)
        w = k1*(self.dc-d)+k2*self.deltad
        # self.deltad = d-self.lastd
        self.lastd = d
        if w>self.MAXW:
            w= self.MAXW
        elif w<-self.MAXW:
            w = -self.MAXW


        # if self.dc-d<0 and not w<0:
            # print("Ley mal")
            # print("dc-d: ", self.dc-d, "d: ", d, "w: ", w)
        # if self.dc-d>0 and not w>0:
            # print("Ley mal")
            # print("dc: ", self.dc, "d: ", d, "w: ", w)
        return np.array([v,w])

    def sensorSim(self, yPared, eps=0.002, epsd=0.002):
        """
        Devuelve la distancia a la pared en yPared en el eje y del robot
        """
        d = (self.x[1]-yPared)/np.cos(self.x[2])
        reached = abs(self.deltad) < epsd and abs(d-self.dc) < eps

        if d > 10 or np.sin(self.x[2]) > 0.15:
            ret = -2
        elif d < 0.5 or np.sin(self.x[2]) < -0.15:
            ret = -1
        elif reached:
            ret = 1
        else:
            ret = 0
        return d, ret
