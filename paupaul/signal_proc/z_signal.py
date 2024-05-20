import numpy as np
from scipy.signal import find_peaks
import matplotlib.pyplot as plt
from collections import deque 
import os

z_list = np.load(os.path.join("paupaul", "logs", "z_list.npy"))
peaks = find_peaks(-z_list, prominence=0.08)

## simulate real condition
dyn_peaks = []
zmeas = deque(maxlen=150) ## can store arrays with position too

for i, z in enumerate(z_list):
    zmeas.append(z)
    
    zar = np.asarray(zmeas)
    
    p, props = find_peaks(-zar, prominence=0.05)
    if len(p) == 1:
        dyn_peaks.append(i-len(zar) + p[0] + 1)

        # print(max(zar[0:p[0]]))
        max_index = i-len(zar) + np.argmax(zar[0:p[0]]) + 1
        dyn_peaks.append(max_index)
        
        ## range of detection
        # dyn_peaks.append(i-len(zar)+1) ## first index
        # dyn_peaks.append(i) ## last index
        
        zmeas.clear()

print(peaks)
print(dyn_peaks)
plt.plot(z_list)
plt.scatter(dyn_peaks, z_list[dyn_peaks], marker='x', color='g')

plt.show()