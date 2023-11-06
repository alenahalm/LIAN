import cv2 as cv
import math
from point import Cell, Node

# midpoint
def circle_successors(a: Node, r: int):
    points = []
    x_centre = a.cell.x
    y_centre = a.cell.y

    x = r
    y = 0
    points.append(Cell(x + x_centre, y + y_centre))
    if r > 0:
        points.append(Cell(-x + x_centre, -y + y_centre))
        points.append(Cell(y + x_centre, x + y_centre))
        points.append(Cell(-y + x_centre, -x + y_centre))
	
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
		
        points.append(Cell(x + x_centre, y + y_centre))
        points.append(Cell(-x + x_centre, y + y_centre))
        points.append(Cell(x + x_centre, -y + y_centre))
        points.append(Cell(-x + x_centre, -y + y_centre))
		
        if x != y:
            points.append(Cell(y + x_centre, x + y_centre))
            points.append(Cell(-y + x_centre, x + y_centre))
            points.append(Cell(y + x_centre, -x + y_centre))
            points.append(Cell(-y + x_centre, -x + y_centre))

    for i in range(len(points)):
        points[i] = Node(points[i], a.g + a.cell.dist(points[i]), a)

    return points

# проверка угла
def angle_between_vectors(a1:Cell, a2:Cell, b1:Cell, b2:Cell) -> float:
    ax = a2.x - a1.x
    ay = a2.y - a1.y 
    bx = b2.x - b1.x
    by = b2.y - b1.y
    ab = ax * bx + ay * by
    a = a1.dist(a2)
    b = b1.dist(b2)
    if a * b == 0:
        return 360
    else:
        cos = round(ab / (a * b), 5)
        return math.acos(cos) * 180 / math.pi

# есть ли препятствие
def line_of_sight(image, a: Cell, b: Cell) -> bool:

    dx = b.x - a.x
    dy = b.y - a.y

    d = dy - (dx/2)  
    x = a.x 
    y = a.y

    line = Cell(x, y)

    if not line.is_traversable(image):
        return False
    
    while (x < b.x): 
        x = x + 1
        
        if(d < 0): 
            d = d + dy  

        else: 
            d = d + (dy - dx)  
            y = y + 1

        line.x = x
        line.y = y
        if not line.is_traversable(image):
            return False

    return True

def expand(image, a: Node, goal: Cell, delta: int, a_m: int, bp: dict, g: dict, CLOSED: list):

    # list с результатом 
    open = []

    # находим окружность
    SUCC = circle_successors(a, delta)

    # если цель ближе, чем delta, то добавляем в потенциальные
    if a.cell.dist(goal) < delta:
        SUCC.append(Node(goal, a.g + a.cell.dist(goal), a))
    
    for s in SUCC: 
        assert isinstance(s, Node) # все элементы SUCC должны быть Node

        # не рассматриваем точки с припятствием
        if not s.cell.is_traversable(image):
            continue
        
        # если у [a] в пути есть родитель, проверяем угол поворота.
        if bp[a.cell.index()] is not None and angle_between_vectors(bp[a.cell.index()].cell, a.cell, a.cell, s.cell) > a_m:
            continue

        # проходимся по рассмотренным точкам и сравниваем с потенциальной
        was_checked = False
        for cp in CLOSED:
            assert isinstance(cp, Node)
            # если их координаты совпадают
            # и координаты родителей тоже
            # отбрасываем точку
            if s.cell == cp.cell and s.parent.cell == cp.parent.cell:
                was_checked = True
                break
        if was_checked:
            continue

        # проверка на препятствие между точками
        if not line_of_sight(image, a.cell, s.cell):
            continue
        
        # если такой точки в словарях нет
        # или новое значение лучше существующего 
        # изменяем словари
        # и добавляем в OPEN
        if (s.cell.index() in g and g[s.cell.index()] > s.g) or s.cell.index() not in g:
            g[s.cell.index()] = s.g
            bp[s.cell.index()] = s.parent
            open.append(s)

    return open


def LIAN(image, start: Cell, goal: Cell, delta: int, a_m: int):

    # bp - это словарь. Ключ - координаты пикселя, значение - Node родителя
    bp = {}
    bp[start.index()] = None
    g = {}
    g[start.index()] = 0

    # стартовая Node.
    st = Node(start, g[start.index()], None)

    OPEN = [st]
    CLOSED = []
    c = 0
    while len(OPEN) > 0:
        # Сортируем OPEN по сумме расстояния до старта (g) и эвристического расстояния до цели
        # и берем нулевой элемент
        a = sorted(OPEN, key=lambda x: x.g + x.cell.dist(goal))[0]
        OPEN.remove(a)

        if a == goal:
            # восстанавливаем путь
            path = []
            cur_node = a.cell
            while cur_node != start:
                path.append(cur_node)
                cur_node = bp[cur_node.index()].cell
            path.reverse()
            print("Path found!")
            return path
        # добавляем a в рассмотренные
        if a not in CLOSED:
            CLOSED.append(a)

        # вызываем expand и получаем list потенциальных Node
        exp = expand(image, a, goal, delta, a_m, bp, g, CLOSED)

        # дополняем OPEN
        for n in exp:
            if n not in OPEN:
                OPEN.append(n)

        # Отображение процесса. Черный - текущий путь. Серый - все рассмотренные точки. Зеленый - рассматриваемые точки.
        copy = image.copy()
        copy = cv.cvtColor(copy, cv.COLOR_GRAY2BGR)
        copy = cv.putText(copy, f'{c}', (50, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv.LINE_AA)

        for p in CLOSED:
            copy = cv.circle(copy, p.cell.index(), 1, (200, 200, 200), -1)

        for p in OPEN:
            copy = cv.circle(copy, p.cell.index(), 1, (0, 125, 0), -1)

        path = []
        cur_node = CLOSED[-1].cell
        while cur_node != start:
            path.append(cur_node)
            cur_node = bp[cur_node.index()].cell
        path.reverse()
        for p in path:
            assert isinstance(p, Cell)
            copy = cv.circle(copy, p.index(), 1, (50, 50, 50), -1)
        cv.imshow("Window", copy)
        k = cv.waitKey(1)
        if k > 0:
            if chr(k) == 'd':
                break
        c += 1
    print("Path not found")
    return []
