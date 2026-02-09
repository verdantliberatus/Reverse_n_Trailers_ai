from sympy import symbols, Eq, solve, nsolve
import sympy
from point import Point

def compute_circle(p1x, p1y, vx, vy, p2x, p2y, r):

    
    # Define symbols
    # cx center of cirle x-value
    # cy center of circle y-value
    # R radius of circle to solve 

    #returns x, y, r in 3x1 matrix
    cx = symbols("cx", real=True)
    cy = symbols("cy", real=True)
    R = symbols("R", real=True)
    
    eq0 = sympy.sqrt((cx-p1x)**2 + (cy-p1y)**2) - R 
    eq1 = vx*(cx - p1x) + vy*(cy-p1y)
    eq2 = (cx - p2x)**2 + (cy - p2y)**2 - R**2 - r**2

    system = [eq0, eq1, eq2]

    result = solve(system, (cx, cy, R))
    print(p1x, p1y, vx, vy, p2x, p2y, r)
    print("valid perp intersect circles", result)
    result = result[0]
    return (float(result[0]),float(result[1]),float(result[2]))



def get_circle(p1, v1, p2, r):
    '''
    Given a point p1 on vector v1, a circle of radius r at point p2.
    Find a circle that is tangent to the vector v1 and intersects p1 and perpendicularly intersects the other circle centered at p2.
    Perpendicular intersection means at the intersection point of the two circles, a tangent line drawn on the unknown circle will intersect point p2.
    '''

    #returns Point object denoting center of circle and returns r as the radius of the circle
    solution = compute_circle(p1.x, p1.y, v1.x, v1.y, p2.x, p2.y, r)
    center = Point(solution[0], solution[1])
    radius = solution[2]
    return center, radius

if __name__ == "__main__":

    p1 = Point(2, 5.25)
    v1 = Point(1, 1)
    p2 = Point(7.5, 3)
    r = 2
    print(compute_circle(p1x=2, p1y=5.25, vx=1, vy=1, p2x=7.5, p2y=3, r=2))
    #print(get_circle(p1,v1, p2, r))
