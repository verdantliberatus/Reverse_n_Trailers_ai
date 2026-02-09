import numpy as np
import math
from point import Point

def distance(p1, p2):
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

def circle_intersect(c1, c2, r1, r2):
    #c1, c2 are two Point objects describing the center of the circle
    #r1, r2 are the radii respectively
    # returns two intersection points of the circles
    # assumes they do intersect
    d = distance(c1, c2)
    if d > r1 + r2 or d < abs(r1 - r2) or d == 0:
        print("bad circles")
        print("c1:", c1)
        print("c2:", c2)
        print("r1:", r1)
        print("r2:", r2)
        print("end of bad circles")
        raise ValueError("The circles do not intersect or are invalid.")

    a = (r1**2 - r2**2 + d**2) / (2 * d)
    h = math.sqrt(r1**2 - a**2)

    x2_intermediate = c1.x + a * (c2.x - c1.x) / d
    y2_intermediate = c1.y + a * (c2.y - c1.y) / d

    offset_x = -h * (c2.y - c1.y) / d
    offset_y = h * (c2.x - c1.x) / d

    intersection1 = Point(x2_intermediate + offset_x, y2_intermediate + offset_y)
    intersection2 = Point(x2_intermediate - offset_x, y2_intermediate - offset_y)
    return intersection1, intersection2


if __name__ == "__main__":
    # Example usage:
    x1, y1, r1 = -1, -2, 5    # Circle 1: center (0, 0), radius 5
    x2, y2, r2 = 6, 2, 7    # Circle 2: center (6, 0), radius 5


    p1 = Point(x1, y1)
    p2 = Point(x2, y2)

    intersection_points = circle_intersect(p1, p2, r1, r2)
    print("Intersection Points:", intersection_points)