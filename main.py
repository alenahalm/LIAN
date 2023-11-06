import cv2 as cv
import numpy as np
import time
import matplotlib.pyplot as plt
from lian import LIAN
from point import Cell, Node

image = cv.imread('map.png')
gr = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
gr[gr > 200] = 255
gr[gr != 255] = 0

Y, X = np.loadtxt('points.txt', unpack=True)

goal = Cell(int(X[1]), int(Y[1]))
start = Cell(int(X[0]), int(Y[0]))

image = np.array(gr)

t = time.time()

path = LIAN(image, start, goal, 5, 25)

for i in path:
    image = cv.circle(image, i.index(), 1, (50, 50, 50), -1)

plt.imshow(image)
plt.show()

print('----', time.time()-t, 'seconds ----')
