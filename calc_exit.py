from point import Point
from math import atan2, pi
from circle_intersect import circle_intersect
import numpy as np
import perpendicular_intersect_circle
from sympy import symbols, Eq, solve, nsolve
import sympy
def nearest_exit(exit1, exit2, p1, v1, c):
    # exit1, exit2: two possible exists on the circle
    # p1 + v1 * t: vector describing the vehicle
    # c: center Point of the circle
    # returns either exit1, or exit2, whichever requires less traveling on the circle.
    r = Point(p1.x-c.x, p1.y-c.y) #radial vector
    
    cross = r.x*v1.y - r.y*v1.x

    vehicle_theta = atan2(p1.y - c.y, p1.x - c.x)
    
    exit1_theta = atan2(exit1.y - c.y, exit1.x - c.x) 
    exit2_theta = atan2(exit2.y - c.y, exit2.x - c.x)

    if cross > 0: #counterclockwise
        if (exit1_theta - vehicle_theta)% 2*pi < (exit2_theta - vehicle_theta)% 2*pi:
            return exit1
        else:
            return exit2
    else: #clockwise
        if (vehicle_theta - exit1_theta)% 2*pi < (vehicle_theta - exit2_theta)% 2*pi:
            return exit1
        else:
            return exit2


def last_exit():
    pass
def construct_last_path(p1, v1, s):
    # p1 + v1*t position and orientation of last trailer
    # s Point of subgoal
    # Point of goal
    cx = symbols("cx", real=True)
    cy = symbols("cy", real=True)
    R = symbols("R", real=True)
    
    #eq0 = sympy.sqrt((cx-p1.x)**2 + (cy-p1.y)**2) - R
    #eq1 = sympy.sqrt((cx-s.x)**2 + (cy-s.y)**2) - R
    eq0= Eq((cx-p1.x)**2 + (cy-p1.y)**2, (cx-s.x)**2 + (cy-s.y)**2)

    eq1= sympy.sqrt((cx-p1.x)**2 + (cy-p1.y)**2) - R
    eq2 = v1.x * (cx-p1.x) + v1.y*(cy-p1.y)

    
    system = [eq0,eq1, eq2]

    result = solve(system, (cx, cy, R))[0]
    return (float(result[0]),float(result[1]),float(result[2]))
    
def construct_middle_circle(path, goal):
    p1 = np.array([path[0], path[1]])
    g = np.array([goal[0], goal[1]])
    v = g-p1

    c = 0.5*v + p1
    print(g, c)
    r = np.linalg.norm((g-c))
    return (float(c[0]), float(c[1]), float(r))



def calc_exit_point(unit, subgoal, goal, type):
    # unit: either trailer or vehicle object
    # subgoal: Point object on where to goal
    # type: type of unit. either "last", "middle", or vehicle
    p1 = unit.get_center()
    v1 = Point(unit.get_orientation()[0], unit.get_orientation()[1])

    if type == "last":
        path = construct_last_path(p1=p1, v1=v1, s=subgoal)
        mid = construct_middle_circle(path, goal)
        x1, y1, r1 = path
        x2, y2, r2 = mid
        exit1, exit2 = circle_intersect(Point(x1, y1), Point(x2, y2), r1, r2)
        nearest = nearest_exit(exit1=exit1, exit2=exit2, p1=p1, v1=v1, c=Point(x1, r1))
        icc = np.array([x1, y1])
        p = np.array(p1.x, p1.y)
        vec_to_icc = (icc - p)/np.linalg.norm(icc-p)
        point_to_icc = Point(vec_to_icc[0], vec_to_icc[1])
        return nearest, point_to_icc
    
    elif type == "middle" or type == "vehicle":
        back_hitch = unit.get_back_hitch()
        back_hitch = np.array([back_hitch.x, back_hitch.y])
        center = unit.get_center()
        center = np.array([center.x, center.y])
        hitch_len = np.linalg.norm(back_hitch-center)
        path_center, path_radius = perpendicular_intersect_circle.get_circle(p1, v1, subgoal, hitch_len)
        exit1, exit2 = circle_intersect(path_center, subgoal, r1 = path_radius, r2 = hitch_len)
        nearest = nearest_exit(exit1, exit2, p1 = unit.get_center(),v1=v1, c = path_center)

        icc = np.array([path_center[0], path_center[1]])
        p = np.array(p1.x, p1.y)
        vec_to_icc = (icc - p)/np.linalg.norm(icc-p)
        point_to_icc = Point(vec_to_icc[0], vec_to_icc[1])

        return nearest, point_to_icc, path_radius
    
    else:
        print("invalid type of vehicle unit")
        exit()







if __name__ == "__main__":
    '''
    a = nearest_exit(exit1=Point(7.5, 4), exit2 = Point(7.5,2), p1=Point(6.5, 3), v1=Point(0, -1), c = Point(7.5, 3))
    b = nearest_exit(exit1=Point(7.5, 4), exit2 = Point(7.5,2), p1=Point(8.5, 3), v1=Point(0, 1), c = Point(7.5, 3))
    print(a)
    print(b)

    import numpy as np



    '''
    a = np.array([1, 1])
    b = np.array([0,0])
    print(np.linalg.norm(a-b))
    construct_last_path(p1=Point(0,0), v1=Point(0, 1), s=Point(1, 0))
    #print(calc_exit_point(1, 1, Point(5,4), "last"))
