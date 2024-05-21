
import numpy as np
edges = [np.array([0, 0]), np.array([0.5, 0.2]), np.array([1, 1])]

de = edges[1]-edges[0]
de /= np.linalg.norm(de)
goal = np.mean(np.asarray(edges)[0:2], axis=0)

print(goal)

goal += np.array([-de[1], de[0]])

print(goal)
