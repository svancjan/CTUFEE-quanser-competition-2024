from pal.products.qcar import QCarRealSense
from pal.utilities.vision import Camera3D
from PIL import Image

from matplotlib import pyplot as plt

import time
import struct
import numpy as np
import cv2
import os

#init realsense camera
RealSense = QCarRealSense()


if RealSense.streamOpened:
    print("RealSense stream opened successfully")

rgb_read = False
max_attempts = 10
attempts = 0
while not rgb_read:
    timestamp = RealSense.read_RGB()
    if timestamp != -1:
        image_rgb = RealSense.imageBufferRGB
    if np.sum(image_rgb) != 0:
        rgb_read = True
    if attempts > max_attempts:
        break
    attempts += 1

depth_read = False
attempts = 0
while not depth_read:
    attempts += 1
    timestamp = RealSense.read_depth(dataMode='PX')
    if timestamp != -1:
        image_depth = RealSense.imageBufferDepthPX
    else:
        print("Error reading depth image")
        continue
    if np.sum(image_depth) != 0:
        depth_read = True
    if attempts > max_attempts:
        break


plt.subplot(1,2,1)
plt.imshow(image_depth)
plt.colorbar()
plt.title("Depth Image")


plt.subplot(1,2,2)
image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2RGB)
plt.imshow(image_bgr)
plt.title("RGB Image")
    


plt.show()


# directory to save images
# img_dir = ".\\images"

# if not os.path.exists(img_dir):
#     os.mkdir(img_dir)
    
# os.chdir(img_dir) 
# cv2.imwrite("stop_close2.png", image_rgb)







