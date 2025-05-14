from typing import List, Tuple
import math
import variables

def is_on_road(x: float, y: float) -> bool:
    # For horizontal roads
    if abs(y - round(y)) < 0.1 and round(y) % 2 == 0 and 0 <= round(y) < variables.roadnet_height:
        return True
    # For vertical roads
    if abs(x - round(x)) < 0.1 and round(x) % 2 == 0 and 0 <= round(x) < variables.roadnet_width:
        return True
    return False

def find_closest_road_point(x: float, y: float) -> Tuple[float, float]:
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

def find_path(start: Tuple[float, float], end: Tuple[float, float]) -> List[Tuple[float, float]]:
    path = [start]
    current = start
    
    while current != end:  # Stop when we reach the exact end point
        if end[0] == round(end[0]):
            variables.start_on_x = True
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
        if is_on_road(next_x, next_y):
            current = (next_x, next_y)
            path.append(current)
        else:
            # If not on road, find the closest road point
            closest = find_closest_road_point(next_x, next_y)
            current = closest
            path.append(current)
    return path
