import matplotlib as mpl
import matplotlib.animation as animation
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colorbar import Colorbar
import sys


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


# Set labels and title

def animate(i, colorbar=False):
    data = pd.read_csv(sys.argv[1], sep=';', header=None)
    data = pd.DataFrame(data)
    data2 = pd.read_csv(sys.argv[2], sep=';', header=None)
    data2 = pd.DataFrame(data2)

    # filter out all times that are not in both DataFrames
    common_ids = set(data2[0]).intersection(set(data[0]))
    data = data[data[0].isin(common_ids)]
    data2 = data2[data2[0].isin(common_ids)]

    # Shrink both DataFrames to the same size
    min_length = min(len(data), len(data2))
    data = data.iloc[:min_length]
    data2 = data2.iloc[:min_length]

    ax.clear()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    if len(sys.argv) == 5:
        ax.set_title(sys.argv[4])
    scatter = ax.scatter(data[1], data[2], data[3], c=data2[int(sys.argv[3])], cmap='viridis')

    if colorbar:
        plt.colorbar(scatter)

def Main():
    if len(sys.argv) != 4 and len(sys.argv) != 5:
        print("Usage: python plot_3d_over_time.py <file> <file2> <column> [<title>]")
        return
    animate(0, colorbar=True)
    ani = animation.FuncAnimation(fig, animate, interval=1000)
    plt.show()
    print("done")

Main()
