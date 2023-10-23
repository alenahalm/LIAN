from point import Point
import math
import cv2 as cv
import matplotlib.pyplot as plt

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

def line_of_sight(image, a, b):

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

def expand(image, a, delta, goal, a_m, bp, g, CLOSED):

    points = midpoint(a, delta)
    
    if a.dist_to(goal) < delta and goal not in points:
        points.append(goal)
    open = []
    ret = True
    bp_changed = False    
    for point in points:
        k = (point.x, point.y)
        if not point.is_empty(image):
            CLOSED.append(a)
            ret = False
            continue
        if bp[(a.x, a.y)] != None and angle_between_vectors(bp[(a.x, a.y)], a, a, point) > a_m:
            continue
        for close_point in CLOSED:
            if point == close_point and bp[k] == close_point:
                continue
        if not line_of_sight(image, a, point):
            continue

        if not (k in bp and bp[k] is not None):
            g[k] = g[(a.x, a.y)] + a.dist_to(point)
            bp[k] = a
            bp_changed = True
            # if k == (89, 129):
            #     print('new 89, 129:', a)
            open.append(point)
        else:
            if g[k] > g[(a.x, a.y)] + a.dist_to(point):
                # if k == (89, 129):
                #     print('changed 89, 129:', a)
                #     print('prev', bp[k])
                g[k] = g[(a.x, a.y)] + a.dist_to(point)
                bp[k] = a
                bp_changed = True

                open.append(point)
    # if bp_changed:
    #     print('changed', bp[k])
    return ret, open

def midpoint(point: Point, r: int):
    points = []
    x_centre = point.x
    y_centre = point.y

    x = r
    y = 0
    points.append(Point(x + x_centre, y + y_centre))
    if r > 0:
        points.append(Point(-x + x_centre, -y + y_centre))
        points.append(Point(y + x_centre, x + y_centre))
        points.append(Point(-y + x_centre, -x + y_centre))
	
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

def clear_bp(bp):
    return 

def LIAN(start, end, delta, a_m, image):

    OPEN = []
    CLOSE = []
    bp = {}
    bp[(start.x, start.y)] = None
    g = {}
    g[(start.x, start.y)] = 0


    OPEN.append(start)
    i = 0
    while len(OPEN) > 0:
        a = sorted(OPEN, key=lambda x: x.priority + g[(x.x, x.y)])[0]
        OPEN.remove(a)
        print(f'iteration {i}')
        i+=1
        print('new a')
        if a == end:
            print("Path found!")
            path = []
            cur_point = end
            while cur_point != start:
                print(cur_point, '<', bp[(cur_point.x, cur_point.y)])
                path.append(cur_point)
                cur_point = bp[(cur_point.x, cur_point.y)]
            path.append(start)
            path.reverse()
            return path, bp, g
        
        CLOSE.append(a)



        ret, exp = expand(image, a, delta, end, a_m, bp, g, CLOSE)
        print('received from expand')

        # if not ret:
            # continue


        # if not ret:
            # current_point = a
            # keys = list(bp.keys())
            # for k in keys:
            #     if bp[k] is None:
            #         continue
                # else:
                #     copy = cv.circle(copy, (bp[k].x, bp[k].y), 1, (50, 50, 50), -1)
                # if (bp[k].x, bp[k].y) != (start.x, start.y) and k != (start.x, start.y):
                    # copy = cv.circle(image, k, 1, (50, 50, 50), -1)
                    # plt.imshow(copy)
                    # plt.show()
                    # CLOSE.append(bp[k])
                    # if bp[k] in OPEN:
                    #     OPEN.remove(bp[k])
                    # bp[k] = None
            # while current_point != start:
            #     current_point = bp[(current_point.x, current_point.y)]
            #     CLOSE.append(bp[(current_point.x, current_point.y)])
            #     if bp[(current_point.x, current_point.y)] in OPEN:
            #         OPEN.remove(bp[(current_point.x, current_point.y)])
            # continue
        # else:
        for point in exp:
            if point not in OPEN:
                OPEN.append(point)
        # copy = image.copy()
        # for p in OPEN:
        #     copy = cv.circle(copy, (p.x, p.y), 1, (50, 50, 50), -1)
        # for p in CLOSE:
        #     copy = cv.circle(copy, (p.x, p.y), 1, (150, 150, 150), -1)
        # copy = cv.circle(copy, (a.x, a.y), 3, (200, 200, 200), -1)
        
        # plt.imshow(copy)
        # plt.get_current_fig_manager().full_screen_toggle()
        # plt.show()

    print("Path not found :(")
    return [], bp, g
