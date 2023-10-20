import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
from point import Point
import math

image = cv.imread('./map.png')
gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
gray[gray > 200] = 255
gray[gray != 255] = 0

Y, X = np.loadtxt('points.txt', unpack=True)

end = Point(int(X[1]), int(Y[1]), 0)
Point.goal = end
start = Point(int(X[0]), int(Y[0]))
image = np.array(gray)

def angle_between_vectors(a1, a2, b1, b2):
    ax = a2.x - a1.x
    ay = a2.y - a1.y 
    bx = b2.x - b1.x
    by = b2.y - b1.y
    ab = ax*bx + ay*by
    a = a1.dist_to(a2)
    b = b1.dist_to(b2)
    if a*b == 0:
        return 360
    else:
        cos = round(ab / (a * b), 5)
        return math.acos(cos)*180/math.pi

def line_of_sight(a, b):

    dx = b.x - a.x
    dy = b.y - a.y

    d = dy - (dx/2)  
    x = a.x 
    y = a.y  

    points = []

    points.append(Point(x, y))
    
    while (x < b.x): 
        x=x+1
        
        if(d < 0): 
            d = d + dy  

        else: 
            d = d + (dy - dx)  
            y=y+1

        points.append(Point(x, y))
    
    for i in points:
        if not i.is_empty(image):
            return False

    return True

def expand(a, delta, goal, a_m, bp, g, CLOSED):

    global image

    points = midpoint(a, delta)
    
    if a.dist_to(goal) < delta and goal not in points:
        points.append(goal)
    open = []
    
    for point in points:
        if not point.is_empty(image):
            continue
        if bp[(a.x, a.y)] != None and angle_between_vectors(bp[(a.x, a.y)], a, a, point) > a_m:
            continue
        for close_point in CLOSED:
            if point == close_point and bp[(point.x, point.y)] == close_point:
                continue
        if not line_of_sight(a, point):
            continue
        g[(point.x, point.y)] = g[(a.x, a.y)] + a.dist_to(point)
        open.append(point)
    return open

def midpoint(point: Point, r: int):

    points = []
    x_centre = point.x
    y_centre = point.y

    x = r
    y = 0
    points.append(Point(x + x_centre, y + y_centre))
    if r > 0:
        points.append(Point(x + x_centre, -y + y_centre))
        points.append(Point(y + x_centre, x + y_centre))
        points.append(Point(-y + x_centre, x + y_centre))
	
    P = 1 - r 

    while x > y:
        y += 1
        if P <= 0: 
            P = P + 2 * y + 1
			
        else:		 
            x -= 1
            P = P + 2 * y - 2 * x + 1
		
        if (x < y):
            break
		
        points.append(Point(x + x_centre, y + y_centre))
        points.append(Point(-x + x_centre, y + y_centre))
        points.append(Point(x + x_centre, -y + y_centre))
        points.append(Point(-x + x_centre, -y + y_centre))
		
        if x != y:
            points.append(Point(y + x_centre, x + y_centre))
            points.append(Point(-y + x_centre, x + y_centre))
            points.append(Point(y + x_centre, -x + y_centre))
            points.append(Point(-y + x_centre, -x + y_centre))

    return points

def LIAN(start, end, delta, a_m):

    global image

    OPEN = []
    CLOSE = []
    bp = {}
    bp[(start.x, start.y)] = None
    g = {}
    g[(start.x, start.y)] = 0

    current_point = None

    OPEN.append(start)

    while len(OPEN) > 0:
        a = sorted(OPEN, key=lambda x: x.priority)[0]
        OPEN.remove(a)
        if a == end:
            print("Path found!")
            bp[(a.x, a.y)] = current_point
            cur_point = end
            path = []
            while cur_point != start:
                path.append(cur_point)
                cur_point = bp[(cur_point.x, cur_point.y)]
            path.append(start)
            path.reverse()
            return bp, g

        CLOSE.append(a)
        if current_point != a:
            bp[(a.x, a.y)] = current_point
        for point in expand(a, delta, end, a_m, bp, g, CLOSE):
            if point not in OPEN:
                OPEN.append(point)
        current_point = a

    print("Path not found :(")


path, dist = LIAN(start, end, 20, 35)
print(len(path))
copy = image.copy()
for x, y in path:
    copy = cv.circle(copy, (x, y), 5, (127,127,127), -1)
plt.imshow(copy)
# plt.get_current_fig_manager().full_screen_toggle()
plt.show()
