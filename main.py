import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
from point import Point
import time
from LIAN import LIAN


image = cv.imread('./map.png')
gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
gray[gray > 200] = 255
gray[gray != 255] = 0

Y, X = np.loadtxt('points.txt', unpack=True)

end = Point(int(X[1]), int(Y[1]), 0)
Point.goal = end
start = Point(int(X[0]), int(Y[0]))
image = np.array(gray)

copy = image.copy()



t = time.time()

path, bp, dist = LIAN(start, end, 15, 13, image)

print('----', time.time()-t, 'seconds ----')

for x, y in bp:
    copy = cv.circle(copy, (x, y), 5, (150, 150, 150), -1)

for p in path:
    copy = cv.circle(copy, (p.x, p.y), 1, (50, 50, 50), -1)



plt.imshow(copy)
plt.show()
