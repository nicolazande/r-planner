import os
import sys
import numpy as np
import matplotlib.pyplot as plt



data_dir = os.path.join("../data", "replan.npy")
results = np.load(data_dir, allow_pickle=True)

time = []
cost = []

for l in results:
    for p in l:
        time.append(1000 * p[0])
        cost.append(p[1])


plt.scatter(cost, time)
plt.ylabel("time [ms]")
plt.xlabel("cost to goal []")
plt.title("Dynamic Replanning")
plt.grid()
plt.show()


