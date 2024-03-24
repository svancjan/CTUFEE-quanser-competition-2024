from pal.products.qcar import QCarCameras
import time
import struct
import numpy as np
import cv2
import os

# initiate csi cameras
cameras = QCarCameras(
    enableBack=True,
    enableFront=True,
    enableLeft=True,
    enableRight=True,
)

# directory to save images
img_dir = ".\images\csi"


def read_cameras():
    cameras.readAll()
    back = cameras.csiRight.imageData
    front = cameras.csiFront.imageData
    left = cameras.csiLeft.imageData
    right = cameras.csiRight.imageData
    return back, front, left, right


if __name__ == "__main__":
    # wait 1/20 second for cameras to start bc default 30 fps framerate
    time.sleep(1/20)

    back, front, left, right = read_cameras()

    # show images
    cv2.imshow("back", back)
    cv2.imshow("front", front)
    cv2.imshow("left", left)
    cv2.imshow("right", right)
    cv2.waitKey(0)



    # save images
    if not os.path.exists(img_dir):
        os.mkdir(img_dir)
        
    os.chdir(img_dir) 
    cv2.imwrite("back.png", back)
    cv2.imwrite("front.png", front)
    cv2.imwrite("left.png", left)
    cv2.imwrite("right.png", right)



