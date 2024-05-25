import numpy as np
import matplotlib.pyplot as plt

def snake():
    
    layers_x = 4
    layers_y = 13
    dx = 0.2
    dy = 0.2
    start_p = np.array([3.2, 0.2])

    goal_list = []

    for i in range(layers_x):
        for j in range(layers_y):
            if i % 2 == 0:
                goal_list.append(start_p + np.array([i*dx, j*dy]))
            else:
                goal_list.append(start_p + np.array([i*dx, (layers_y-j-1)*dy]))

    return goal_list

goals = np.asarray(snake())

plt.scatter(goals)
plt.plot()