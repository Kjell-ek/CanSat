#!/usr/bin/env python3
import numpy as np
from time import sleep, perf_counter

"""Data serialization for Arduino Uno - PC Interface"""

if __name__ == "__main__":
    var = 0
    theta = np.linspace(-4 * np.pi, 4 * np.pi, 500)
    z = np.linspace(-2, 2, 500)
    r = z**2 + 1
    x = r * np.sin(theta)
    y = r * np.cos(theta)

    c = np.linspace(start=0, stop=499, num=500)

    for i in range(500):
        # Sample data
          # Fourth variable for color coding
        with open("test3d.csv", "a") as myfile:
            myfile.write(str(x[i]) + ";" + str(y[i]) + ";" + str(z[i]) + ";" + str(c[i])+ "\n")
            myfile.flush()
        sleep(0.5) # 1 second
