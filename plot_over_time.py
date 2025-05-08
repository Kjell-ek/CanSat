import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd

fig = plt.figure()
ax = fig.add_subplot(1,1,1)

def animate(i):
    data = pd.read_csv(sys.argv[1], sep=';', header=None)
    data = pd.DataFrame(data)
    ax.clear()
    ax.plot(data[0],data[int(sys.argv[2])])

def Main():
    if len(sys.argv) != 3:
        print("Usage: python plot_over_time.py <file> <column>")
        return
    ani = animation.FuncAnimation(fig, animate, interval=1000)
    plt.show()
    print("done")

Main()

