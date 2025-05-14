import numpy as np
from typing import List, Tuple
import math

class RoadNetwork:
    def __init__(self, width: int, height: int):
        # Initialize a grid where 1 represents roads and 0 represents non-road areas
        self.grid = np.zeros((height, width), dtype=int)
        self.width = width
        self.height = height
        
        # Create a simple road network (you can modify this pattern)
        # Horizontal roads
        for y in range(0, height, 2):
            self.grid[y, :] = 1
        # Vertical roads
        for x in range(0, width, 2):
            self.grid[:, x] = 1
        print(self.grid)

    def is_on_road(self, x: float, y: float) -> bool:
        """Check if a point is on a road (within a small tolerance)"""
        # For horizontal roads
        if abs(y - round(y)) < 0.1 and round(y) % 2 == 0 and 0 <= round(y) < self.height:
            return True
        # For vertical roads
        if abs(x - round(x)) < 0.1 and round(x) % 2 == 0 and 0 <= round(x) < self.width:
            return True
        return False

    def find_closest_road_point(self, x: float, y: float) -> Tuple[float, float]:
        """Find the closest point on the road network to the given coordinates"""
        first_option = (round(x), y)
        second_option = (x, round(y))

        if math.sqrt((first_option[0] - x)**2 + (first_option[1] - y)**2) < math.sqrt((second_option[0] - x)**2 + (second_option[1] - y)**2):
            return first_option
        elif math.sqrt((first_option[0] - x)**2 + (first_option[1] - y)**2) > math.sqrt((second_option[0] - x)**2 + (second_option[1] - y)**2):
            return second_option
        else:
            if first_option[0] + first_option[1] <= second_option[0] + second_option[1]:
                return first_option
            else:
                return second_option

    def find_path(self, start: Tuple[float, float], end: Tuple[float, float]) -> List[Tuple[float, float]]:
        """Find a path from start to end point along the roads"""
        path = [start]
        current = start
        
        while current != end:  # Stop when we reach the exact end point
            if end[0] == round(end[0]):
                # Move horizontally first, then vertically
                if current[0] != end[0]:
                    # Move towards end.x while staying on the current road
                    next_x = current[0] + (0.1 if end[0] > current[0] else -0.1)
                    next_y = current[1]
                    next_x = round(next_x, 1)
                    next_y = round(next_y, 1)
                else:
                    # Move towards end.y while staying on the current road
                    next_x = current[0]
                    next_y = current[1] + (0.1 if end[1] > current[1] else -0.1)
                    next_x = round(next_x, 1)
                    next_y = round(next_y, 1)
            else:
                if current[1] != end[1]:
                    next_x = current[0]
                    next_y = current[1] + (0.1 if end[1] > current[1] else -0.1)
                    next_x = round(next_x, 1)
                    next_y = round(next_y, 1)
                else:
                    next_x = current[0] + (0.1 if end[0] > current[0] else -0.1)
                    next_y = current[1]
                    next_x = round(next_x, 1)
                    next_y = round(next_y, 1)

            # Ensure the next point is on a road
            if self.is_on_road(next_x, next_y):
                current = (next_x, next_y)
                path.append(current)
            else:
                # If not on road, find the closest road point
                closest = self.find_closest_road_point(next_x, next_y)
                current = closest
                path.append(current)
        return path

def main():
    # Create a 20x20 road network
    network = RoadNetwork(20, 20)
    
    # Example destination point (can be modified)
    destination = (15.7, 12.3)
    
    # Find closest point on road to destination
    closest_point = network.find_closest_road_point(destination[0], destination[1])
    print(f"Destination: {destination}")
    print(f"Closest point on road: {closest_point}")
    
    # Find and print the path
    path = network.find_path((0.0, 0.0), closest_point)
    print("\nPath from (0,0) to destination:")
    for point in path:
        print(f"({point[0]:.3f}, {point[1]:.3f})")

if __name__ == "__main__":
    main()
