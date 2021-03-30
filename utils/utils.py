"""
Vectors are np.array[x,y,th]
"""

import numpy as np
import math
import sys


import pandas as pd
import matplotlib.pyplot as plt
 
def plotFile(fileName, sep="\t"):
    df  = pd.read_csv(fileName, sep=sep)
    print(df)
    #df.plot()  # plots all columns against index
    ax = df.plot(kind='scatter',x='x',y='y',
        xlim=[-1.5,0.2], ylim=[-0.3,0.3], title="Odometría - video 1",
        grid=True) # scatter plot
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect('equal')
    #df.plot(kind='density')  # estimate density function
    plt.show()






if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("necesito el csv")
        exit(1)
    plotFile(sys.argv[1])
