import cv2 as cv
import numpy as np
import math
from point import Cell, Node


class Lian:

    def __init__(self, image: np.ndarray, start: Cell, goal: Cell, delta: int, a_m: int) -> None:
        self.image = image
        self.start = start
        self.goal = goal
        self.delta = delta
        self.a_m = a_m
        self.OPEN = [Node(start, 0, None)]
        self.CLOSED = []
        self.nodes = []

    # midpoint
    def circle_successors(self, a: Node):
        points = []
        x_centre = a.cell.x
        y_centre = a.cell.y

        x = self.delta
        y = 0
        points.append(Cell(x + x_centre, y + y_centre))
        if self.delta > 0:
            points.append(Cell(-x + x_centre, -y + y_centre))
            points.append(Cell(y + x_centre, x + y_centre))
            points.append(Cell(-y + x_centre, -x + y_centre))

        P = 1 - self.delta

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
            nodes.append(Node(p, a.g + a.cell.dist(p), a))

        return nodes

    # проверка угла
    def angle_between_vectors(self, a1: Cell, a2: Cell, b1: Cell, b2: Cell) -> float:
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

    # есть ли преграда между точками
    def intermediates(self, a: Cell, b: Cell):

        nb_points = int(a.dist(b))

        x_spacing = (b.x - a.x) / (nb_points + 1)
        y_spacing = (b.y - a.y) / (nb_points + 1)

        points = [[int(a.x + i * x_spacing), int(a.y + i * y_spacing)]
                  for i in range(1, nb_points+1)]
        check_cell = Cell(0, 0)
        for p in points:
            check_cell.x = p[0]
            check_cell.y = p[1]
            if not check_cell.is_traversable(self.image):
                return False
        return True

    def lian(self):

        c = 0
        while len(self.OPEN) > 0:
            # Сортируем OPEN по сумме расстояния до старта (g) и эвристического расстояния до цели
            # и берем нулевой элемент
            a = sorted(self.OPEN, key=lambda x: x.g +
                       x.cell.dist(self.goal))[0]
            self.OPEN.remove(a)

            if a == self.goal:
                # восстанавливаем путь
                path = []
                cur_node = a
                while cur_node != Node(self.start, 0, None):
                    path.append(cur_node)
                    cur_node = cur_node.parent
                path.reverse()
                print("Path found!")
                return path

            #  добавляем в рассмотренные
            if a not in self.CLOSED:
                self.CLOSED.append(a)

            # получаем новые точки OPEN
            self.expand(a)

        print("Path not found")
        return []

    def expand(self, a: Node):
        # находим окружность для точки a
        SUCC = self.circle_successors(a)

        # если цель ближе, чем delta, то добавляем в потенциальные
        if a.cell.dist(self.goal) < self.delta:
            SUCC.append(Node(self.goal, a.g + a.cell.dist(self.goal), a))

        for s in SUCC:
            assert (isinstance(s, Node))

            # не рассматриваем точки с припятствием
            if not s.cell.is_traversable(self.image):
                continue

            # если у [a] в пути есть родитель, проверяем угол поворота.
            if a.parent is not None and self.angle_between_vectors(a.parent.cell, a.cell, a.cell, s.cell) > self.a_m:
                continue

            # проходимся по рассмотренным точкам и сравниваем с потенциальной
            was_checked = False
            for cp in self.CLOSED:
                assert (isinstance(cp, Node))
                # если точка уже рассмотрена и у них один родитель
                # отбрасываем точку
                if s == cp and s.parent == cp.parent:
                    was_checked = True
                    break
            if was_checked:
                continue

            # проверка на препятствие между точками
            if not self.intermediates(a.cell, s.cell):
                continue

            # добавляем в OPEN
            if s.index() in self.nodes:
                if s.g < self.nodes[s.index()].g:
                    self.nodes[s.index()] = s
                    self.OPEN.append(s)
            else:
                self.nodes[s.index()] = s
                self.OPEN.append(s)
