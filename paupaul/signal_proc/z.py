import numpy as np
from scipy.signal import find_peaks
import matplotlib.pyplot as plt
from collections import deque 
import os

hist = np.load(os.path.join("paupaul", "logs", "edge.npy"))

p, _ = find_peaks(-hist[:,2], prominence=0.05)

plt.plot(hist[:,2])
plt.scatter(p[0], hist[p[0],2], marker='x', color='g')

index = np.argmax(hist[0:p[0], 2])

pos = hist[index, 0:2]

print(hist)
print("Pos:", pos)
plt.scatter(index, hist[index,2], marker='x', color='r')
plt.show()