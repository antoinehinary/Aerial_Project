
import numpy as np
import matplotlib.pyplot as plt

edges = [np.array([0, 0]), np.array([0.5, 0.2]), np.array([1, 1])]

de = edges[1]-edges[0]
print(de)

de /= np.linalg.norm(de)


mean = np.mean(edges[0:2], axis=0)

goal = mean + np.array([de[1], -de[0]])

edges = np.asarray(edges)

plt.scatter(edges[:2,0], edges[:2,1])

plt.scatter(mean[0], mean[1])

plt.scatter(goal[0], goal[1])

plt.show()