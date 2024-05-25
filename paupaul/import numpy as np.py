import numpy as np
import matplotlib.pyplot as plt

pos = np.array([0, 0])


def snake_return(pos):

    w = 10
    h = 3
    dx = 0.3
    dy = 0.3
    p = pos + np.array([h/2*dx, -w/2*dy])

    goal_list = []

    for i in range(h):
        for j in range(w):
            if i % 2 == 0:
                goal_list.append(p + np.array([i*dx, j*dy]))
            else:
                goal_list.append(p + np.array([i*dx, (w-j-1)*dy]))

    return goal_list


alist = snake_return(pos)
alist = np.array(alist)

plt.scatter(alist[:, 0], alist[:, 1])
plt.show()
