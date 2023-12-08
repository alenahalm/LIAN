import cv2 as cv
import math
from time import time
import numpy as np
from queue import PriorityQueue
from point import Cell, Node


def angle_between_vectors(a1: Cell, a2: Cell, b1: Cell, b2: Cell) -> float:
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


def intermediates(image: np.ndarray, a: Cell, b: Cell):

    nb_points = int(a.dist(b))

    x_spacing = (b.x - a.x) / (nb_points + 1)
    y_spacing = (b.y - a.y) / (nb_points + 1)

    points = [[int(a.x + i * x_spacing), int(a.y + i * y_spacing)]
              for i in range(1, nb_points+1)]
    check_cell = Cell(0, 0)
    for p in points:
        check_cell.x = p[0]
        check_cell.y = p[1]
        if not check_cell.is_traversable(image):
            return False
    return True


def circle_successors(a: Node, r: int) -> list[Node]:
    x_centre = a.cell.x
    y_centre = a.cell.y

    x = r
    y = 0
    points = [Cell(x + x_centre, y + y_centre)]
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

    nodes = []
    for p in points:
        nodes.append(Node(p, a.g + p.dist(a.cell), a))

    return nodes


def expand(image: np.ndarray, a: Node, delta: int, a_m: int, CLOSE: list[Node], goal: Cell, nodes: dict) -> list[Node]:

    open = []

    SUCC = circle_successors(a, delta)
    if a.cell.dist(goal) < delta:
        SUCC.append(
            Node(goal, a.g + a.cell.dist(goal), a)
        )

    for child in SUCC:
        if not child.cell.is_traversable(image):
            continue

        if a.parent is not None and angle_between_vectors(a.parent.cell, a.cell, a.cell, child.cell) > a_m:
            continue

        if not intermediates(image, a.cell, child.cell):
            continue

        was_checked = False
        for cl in CLOSE:
            if child == cl and child.parent == cl.parent:
                was_checked = True
                break
        if was_checked:
            continue
        if child.index() in nodes:
            if child.g < nodes[child.index()].g:
                nodes[child.index()] = child
                open.append(child)
        else:
            nodes[child.index()] = child
            open.append(child)
    return open


def Lian(image: np.ndarray, start: Cell, goal: Cell, delta: int, a_m: int) -> list[Node]:
    start_node = Node(start, 0, None)
    OPEN = PriorityQueue()
    OPEN.put((0, start_node))
    CLOSE = [start_node]
    CLOSE.remove(start_node)

    nodes = {}

    t = time()

    c = 0
    while not OPEN.empty():
        a = OPEN.get()[1]
        assert (isinstance(a, Node))

        if a == goal:
            path = []
            cur_node = a
            while cur_node != start_node:
                path.append(cur_node)
                cur_node = cur_node.parent
            path.reverse()
            return path, a.g

        exp = expand(image, a, delta, a_m, CLOSE, goal, nodes)
        for e in exp:
            OPEN.put((e.g + e.cell.dist(goal), e))

        CLOSE.append(a)
        c += 1
        if time() - t > 0.2:
            copy = image.copy()
            copy = cv.cvtColor(copy, cv.COLOR_GRAY2BGR)
            copy = cv.putText(
                copy, f'{c}', (50, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv.LINE_AA)
            copy = cv.circle(copy, a.index(), 1, (0, 0, 255), -1)
            copy = cv.circle(copy, goal.index(), 1, (0, 0, 255), -1)
            for o in OPEN.queue:
                copy = cv.circle(copy, o[1].index(), 1, (0, 125, 0), -1)
            for cl in CLOSE:
                copy = cv.circle(copy, cl.index(), 1, (200, 200, 200), -1)
            cur_node = a
            while cur_node != start_node:
                copy = cv.circle(copy, cur_node.index(), 1, (0, 0, 0), -1)
                cur_node = cur_node.parent
            copy = cv.circle(copy, a.index(), 1, (0, 0, 255), -1)
            cv.imshow("Lian", copy)

            k = cv.waitKey(1)
            if k > 0 and chr(k) == 'd':
                break
            t = time()
    return [], 0
