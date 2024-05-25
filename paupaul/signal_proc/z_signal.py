import numpy as np
from scipy.signal import find_peaks
import matplotlib.pyplot as plt
from collections import deque 
import os

def compute_rms(ac_signal):
    mean_value = np.mean(ac_signal)  # Calculate the mean value of the signal
    ac_signal_centered = ac_signal - mean_value  # Subtract the mean from each sample
    squared_values = np.square(ac_signal_centered)  # Square each centered value
    mean_squared = np.mean(squared_values)  # Calculate the mean of squared centered values
    rms_value = np.sqrt(mean_squared)  # Take the square root of the mean
    return rms_value

z_list = np.load(os.path.join("paupaul", "logs", "z_list.npy"))
# peaks = find_peaks(-z_list, prominence=0.08)

## simulate real condition
dyn_peaks = []
zmeas = deque(maxlen=150) ## can store arrays with position too
dyn_rms = []
dyn_var = []

for i, z in enumerate(z_list):
    zmeas.append(z)
    
    zar = np.asarray(zmeas)
    
    p, info = find_peaks(-zar, prominence=0.03)
    if len(p) == 1:
        dyn_peaks.append(i-len(zar) + p[0] + 1)
        # print(max(zar[0:p[0]]))
        max_index = i-len(zar) + info['left_bases'][0] + 1
        dyn_peaks.append(max_index)
        
        max_index = i-len(zar) + info['right_bases'][0] + 1
        dyn_peaks.append(max_index)
        
        zmeas.clear()
    
    dyn_var.append(np.std(zmeas))
    dyn_rms.append(compute_rms(zmeas))

# print(peaks)
print(dyn_peaks)
# plt.plot(z_list)
# plt.plot(dyn_rms)
plt.plot(dyn_var)

# plt.scatter(dyn_peaks, z_list[dyn_peaks], marker='x', color='g')

plt.show()