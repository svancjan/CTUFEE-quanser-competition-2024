# import matplotlib.pyplot as plt
import cv2

import numpy as np

x_offset = 0.13
y_offset = 1.67

# load Qpath.npy
path = np.load('Qpath.npy')
# take just firts 2 columns of path
path = path[:, :2]

# load map image
map_img = cv2.imread('cityscape.png')



# show image
cv2.imshow('Map', map_img)
cv2.waitKey(0)



# get center of image
center = (map_img.shape[1]//2, map_img.shape[0]//2)

print("path before shift:", path)
# from all points in path shift by (-x_offset, -y_offset)
path = path - np.array([y_offset, x_offset])
print("path after shift",path)

path = path * 100

# shift by center
path = path + center

# make all points integers
path = path.astype(int)


# draw points from path into the image

for i in range(1, len(path)):
    cv2.line(map_img, tuple(path[i-1]), tuple(path[i]), (0, 255, 0), 2)

cv2.imshow('Map', map_img)
cv2.waitKey(0)

