import cv2 as cv
import numpy as np
import time
import matplotlib.pyplot as plt
from lian import Lian
from point import Cell, Node


image = cv.imread('map1.png')
gr = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
gr[gr > 200] = 255
gr[gr != 255] = 0

Y, X = np.loadtxt('points.txt', unpack=True)

goal = Cell(int(X[1]), int(Y[1]))
goal = Cell(410, 420)
start = Cell(int(X[0]), int(Y[0]))

image = np.array(gr)

# print(image.shape)
# lian, dist = Lian(image, start, goal, 30, 25)


t = time.time()
lian, dist = Lian(image, start, goal, 30, 25)
print('----', time.time()-t, 'seconds ----')
for i in lian:
    print(i.cell)

for i in range(len(lian)-1):
    image = cv.circle(image, lian[i].index(), 1, (50, 50, 50), -1)

plt.imshow(image)
plt.show()
