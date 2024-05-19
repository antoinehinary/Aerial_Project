import agent
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

# Assuming agent.find_landing_pos(edges) returns an optimization result object with .x attribute

edges_list = [
    np.array([[0.223458, 0.3],
              [0.3, 0.1],
              [0.3, 0.21454]]),
    np.array([[0, 0.2841],
              [0.3, 0.286140],
              [0.14989, 0.3]]),
    np.array([[0.3, 0.2861],
              [0.268658, 0.3],
              [0.2874, 0.3]])
]

# Initialize subplots
fig, axes = plt.subplots(1, 3, figsize=(15, 5))

# Define the rectangle parameters
rect_center = (0.15, 0.15)
rect_width = 0.3
rect_height = 0.3

# Calculate the bottom left corner of the rectangle
rect_bottom_left = (rect_center[0] - rect_width / 2, rect_center[1] - rect_height / 2)

for i, edges in enumerate(edges_list):
    ax = axes[i]

    # Get the center from the agent's find_landing_pos function
    center = agent.find_landing_pos(edges)
    print(f"Center for edges{i+1}: {center}")

    # Plot the points
    ax.scatter(edges[:, 0], edges[:, 1], color='blue', label='Points')

    # Plot the circle with radius 0.15
    circle = Circle(center, 0.15, color='red', fill=False, label='Circle of radius 0.15')
    ax.add_patch(circle)

    # Plot the center point
    ax.scatter(center[0], center[1], color='green', label='Center', zorder=5)

    # Plot the rectangle
    rectangle = Rectangle(rect_bottom_left, rect_width, rect_height, edgecolor='blue', fill=False, label='Rectangle')
    ax.add_patch(rectangle)

    # Setting labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_aspect('equal', 'box')
    ax.set_xlim(-0.1,0.4)
    ax.set_ylim(-0.1,0.4)
    # Show grid
    ax.grid(True)

# Display legend and show the plot
plt.show()
