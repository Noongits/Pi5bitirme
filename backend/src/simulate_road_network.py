import numpy as np
import variables

def create_road_network():
    grid = np.zeros((variables.roadnet_height, variables.roadnet_width), dtype=int)

    # Horizontal roads
    for y in range(0, variables.roadnet_height, 2):
        grid[y, :] = 1
    # Vertical roads
    for x in range(0, variables.roadnet_width, 2):
        grid[:, x] = 1

    return grid
