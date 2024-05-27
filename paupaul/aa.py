import numpy as np
import matplotlib.pyplot as plt


def snake():

    layers_x = 5
    layers_y = 8
    dx = 0.3
    dy = 0.3
    start_p = np.array([3.3, 0.3])

    goal_list = []

    for i in range(layers_x):
        for j in range(layers_y):
            if i % 2 == 0:
                goal_list.append(start_p + np.array([i*dx, j*dy]))
            else:
                goal_list.append(start_p + np.array([i*dx, (layers_y-j-1)*dy]))

    return goal_list


a = snake()

for i, val in enumerate(a):
    plt.scatter(val[0], val[1])
    plt.text(val[0], val[1], str(i), fontsize=9, ha='center', va='bottom')

plt.xlim(0, 5)
plt.ylim(0, 3)
plt.show()
