from PIL import Image
from matplotlib import pyplot as plt
import os
import cv2
import numpy as np
from scipy import stats
from copy import deepcopy
import time


def make_cc(img, treshold):
    '''
    Tresholds certain color in a image and returns connected components of that color
    inputs:
        img: numpy array <-- rgb image
        treshold: list of 3 ints <-- treshold values for hsv
    outputs:
        connected_comp: tuple of numpy arrays <-- cv2 connected components
    '''
    range = 15
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img_h = img[:, :, 0]
    img_s = img[:, :, 1]
    img_v = img[:, :, 2]
    if treshold[0] - range < 0:
        tresh_h1 = (img_h > treshold[0] -
                    range) & (img_h < treshold[0] + range)
        tresh_h2 = (img_h > 180 - (range - treshold[0]))
        tresh_h = tresh_h1 | tresh_h2
    elif treshold[0] + range > 180:
        tresh_h1 = (img_h > treshold[0] -
                    range) & (img_h < treshold[0] + range)
        tresh_h2 = (img_h < (range - (180 - treshold[0])))
        tresh_h = tresh_h1 | tresh_h2
    else:
        tresh_h = (img_h > treshold[0] - range) & (img_h < treshold[0] + range)

    tresh_s = img_s > treshold[1]
    tresh_v = img_v > treshold[2]

    tresh = tresh_h & tresh_s & tresh_v
    tresh_output = tresh * 255
    connected_comp = cv2.connectedComponentsWithStats(
        tresh_output.astype(np.uint8), connectivity=4)

    return connected_comp
    
def detect_stop_sign(img):
    '''
    Detects the stop sign in the image.
    inputs:
        image: numpy array <-- rgb image
    outputs:
        bb: numpy array <-- 4x1 array, where first two elements are x,y position of the top left corner and the last two are width and height of the bounding box
        '''
    image = deepcopy(img)

    # make upper third of image black
    image[:image.shape[0]//3, :] = 0


    # connected_components = make_cc(image, [3,80,80])
    connected_components = make_cc(image, [3,80,80])
    sign_img = np.zeros_like(connected_components[1], dtype=np.uint8)
    map = np.where(connected_components[1] != 0)
    sign_img[map] = 1
    kernel = np.ones((10, 10), np.uint8)
    closed_img = cv2.morphologyEx(sign_img, cv2.MORPH_CLOSE, kernel)

    # get connected componenents again
    connected_components = cv2.connectedComponentsWithStats(
        closed_img, connectivity=8)
    # get bounding boxes of all componennts
    bounding_boxes = connected_components[2]

    # find bounding box with almost square ratio in bounding boxes
    found_box  = False
    best_index = None
    for i in range(len(bounding_boxes)):
        x, y, w, h, area = bounding_boxes[i]
        if area > 50 and area/w/h > 0.7:
            ratio = w/h
            if 0.8 < ratio < 1.2:
                found_box = True
                best_index = i

                break
    if not found_box:
        return None
    else:
        return bounding_boxes[best_index]
    
def detect_traffic_lights(image):
    '''
    Detects traffic lights in the image.
    inputs:
        image: numpy array <-- rgb image
    outputs:
        (bb ,state): bb :numpy array <-- 4x1 array, where first two elements are x,y position of bottom left corner and the last two are width and height of the bounding box
                     state: int <-- state of the traffic light, 0 for red, 1 for green, 2 for yellow/unknown
                     if no traffic light is detected, returns None
    
    '''
    # connected_components = make_cc(image, [40,50,50])
    connected_components = make_cc(image, [35,50,50])
    sign_img = np.zeros_like(connected_components[1], dtype=np.uint8)
    map = np.where(connected_components[1] != 0)
    sign_img[map] = 1
    kernel = np.ones((10, 10), np.uint8)
    closed_img = cv2.morphologyEx(sign_img, cv2.MORPH_CLOSE, kernel)
    # get connected componenents again
    connected_components = cv2.connectedComponentsWithStats(
        closed_img, connectivity=8)
    # get bounding boxes of all componennts
    bounding_boxes = connected_components[2]

    # find bounding box with almost square ratio in bounding boxes
    found_box  = False
    best_index = None
    for i in range(len(bounding_boxes)):
        x, y, w, h, area = bounding_boxes[i]
        if area > 200 and area/w/h > 0.7:
            ratio = w/h
            if 0.4 < ratio < 0.6:
                found_box = True
                best_index = i

                break
    if not found_box:
        bb = None
    else:
        bb = bounding_boxes[best_index]

    if bb is not None:
        x,y,w,h, area = bb
        cv2.rectangle(image, (x, y), (x+w, y+h), (10, 10, 255), 1)

        left_offset = w*0.34
        light_size = int(w*0.3)
        # upper light rectagle
        upper_light_offset = h*0.14
        ul_x = int(x + left_offset)
        ul_y = int(y+upper_light_offset)
        upper_img = image[ul_y:ul_y+light_size, ul_x:ul_x+light_size]

        # lower light rectagle
        lower_light_offset = h*0.65
        ll_x = int(x + left_offset)
        ll_y = int(y+lower_light_offset)
        lower_img = image[ll_y:ll_y+light_size, ll_x:ll_x+light_size]

        ul_vector = np.median(upper_img, axis=(0,1))
        ll_vector = np.median(lower_img, axis=(0,1))

        # magic numbers for red and green
        red = np.array([ 31.5, 83, 255, 17.5, 171, 80.5])
        green = np.array([ 19, 41.5, 213.5, 71,  241.5, 162])

        vector = np.concatenate((ul_vector,ll_vector))
        red_distance = np.linalg.norm(vector - red)
        green_distance = np.linalg.norm(vector - green)

        if red_distance < green_distance:
            state = 0
        else:
            state = 1

        if abs(red_distance - green_distance) < 10:
            state = 2

        output = (bb, state)
    else:
        output = None

    return output


def estimate_distance_y1(xmin, ymin, xmax, ymax, image):
    # Camera intrinsic matrix
    focal_length_x = 318.86 #CSI front 
    principal_point_x = 401.34 #CSI front 
    focal_length_y = 312.14 #CSI front 
    principal_point_y = 201.50 #CSI front

    focal_length_x = 455.20 
    principal_point_x = 308.53
    focal_length_y = 459.49
    principal_point_y = 213.55
    K = np.array([[focal_length_x, 0, principal_point_x],
                  [0, focal_length_y, principal_point_y],
                  [0, 0, 1]])

    # Distortion coefficients
    dist_coeffs = np.array([-0.9033, 1.5314, -0.0173, 0.0080, -1.1659])  #CSI front 
    dist_coeffs = np.array([-0.05114, 5.4549 , -0.0226 , -0.0062, -20.190])

    # Pixel coordinates of bounding box
    bbox_pixel_coords = [xmin, ymin, xmax, ymax]

    # Undistort image
    undistorted_image = cv2.undistort(image, K, dist_coeffs)

    # Calculate pixel coordinates of bounding box
    xmin_ud, ymin_ud, xmax_ud, ymax_ud = bbox_pixel_coords

    # Convert to normalized image coordinates
    bbox_normalized_coords = [(xmin_ud / undistorted_image.shape[1], ymin_ud / undistorted_image.shape[0]),
                              (xmax_ud / undistorted_image.shape[1], ymax_ud / undistorted_image.shape[0])]

    # Backproject to ray
    ray_start = np.dot(np.linalg.inv(K), np.array([bbox_normalized_coords[0][0], bbox_normalized_coords[0][1], 1]))
    ray_end = np.dot(np.linalg.inv(K), np.array([bbox_normalized_coords[1][0], bbox_normalized_coords[1][1], 1]))

    # Normalize rays
    ray_start /= np.linalg.norm(ray_start)
    ray_end /= np.linalg.norm(ray_end)

    # Estimate distance along y-axis
    distance_to_camera_y = (ray_start[1] - ray_end[1]) / (ray_end[2] - ray_start[2])

    return distance_to_camera_y

def estimate_distance_y2(xmin, ymin, xmax, ymax, image,flag):
    '''
    # Camera intrinsic matrix
    focal_length_x = 318.86 #CSI front 
    principal_point_x = 401.34 #CSI front 
    focal_length_y = 312.14 #CSI front 
    principal_point_y = 201.50 #CSI front
    '''
    focal_length_x = 455.20 
    principal_point_x = 308.53
    focal_length_y = 459.49
    principal_point_y = 213.55
    '''
    focal_length_x = 481.06 
    principal_point_x = 320.36
    focal_length_y = 479.76
    principal_point_y = 235.64
    '''
    K = np.array([[focal_length_x, 0, principal_point_x],
                  [0, focal_length_y, principal_point_y],
                  [0, 0, 1]])

    # Distortion coefficients
    #dist_coeffs = np.array([-0.9033, 1.5314, -0.0173, 0.0080, -1.1659])  #CSI front 
    dist_coeffs = np.array([-0.05114, 5.4549 , -0.0226 , -0.0062, -20.190])

    # Pixel coordinates of bounding box
    bbox_pixel_coords = [xmin, ymin, xmax, ymax]

    # Undistort image
    undistorted_image = cv2.undistort(image, K, dist_coeffs)

    # Calculate pixel coordinates of bounding box
    xmin_ud, ymin_ud, xmax_ud, ymax_ud = bbox_pixel_coords
    
    # Convert to normalized image coordinates
    bbox_normalized_coords = [(xmin_ud / undistorted_image.shape[1], ymin_ud / undistorted_image.shape[0]),
                              (xmax_ud / undistorted_image.shape[1], ymax_ud / undistorted_image.shape[0])]
    if flag == 0: # traffic light
        #real_width = 0.075 # 7.5cm ve zmensenem svete 10*, tedy 75cm ve svete 1:1
        real_width = 0.0775 # 7.75dm
        real_height = 0.161 # 0.16183m in real world 16,083 dm
    elif flag == 1: # stop sign
        real_width = 0.065 # 
        real_height = 0.065 #
    else:
        print("Detected object cannot be recognised")
    distance_to_camera_x = (real_width * focal_length_x)/ (abs(xmax_ud-xmin_ud))
    distance_to_camera_y = (real_height * focal_length_y)/ (abs(ymax_ud-ymin_ud))
    print("Computed distance in x direction:",distance_to_camera_x)
    print("Computed distance in y direction:",distance_to_camera_y)
    print("\n")
    return distance_to_camera_x,distance_to_camera_y

# ___________________________________________________________________________
if __name__ == "__main__":
    # Load the image
    path = "C:\\Users\\krava\\OneDrive\\Plocha\\Quanser\\Research_Resources\\Research_Resources\\src\\libraries\\tutorials\\auto_traffic24.jpg"
    image = cv2.imread(path)

    # Detect stop sign
    bb_stop = detect_stop_sign(image)
    
    # Detect traffic light
    bb_semafor = detect_traffic_lights(image)

    # Display the image with bounding boxes and state information
    #plt.figure()
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Draw stop sign bounding box if detected
    if bb_stop is not None:
        x, y, w, h, area = bb_stop
        print("Pw",w)
        cv2.rectangle(image, (x, y), (x+w, y+h), (10, 10, 255), 2)
        print("Stop Sign Bounding Box:", x, y, w, h)
        distance = estimate_distance_y2(x, y, x+w, y+h, image,1)
        print("The estimated distance to stop sign is: ",distance)
    # Draw traffic light bounding box and state if detected
    if bb_semafor is not None:
        bb, state = bb_semafor
        x, y, w, h, area = bb
        cv2.rectangle(image, (x, y), (x+w, y+h), (10, 10, 255), 2)
        if state == 0:
            print("Traffic Light: Red")
            cv2.putText(image, "Red", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (10, 10, 255), 2)
        elif state == 1:
            print("Traffic Light: Green")
            cv2.putText(image, "Green", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (10, 10, 255), 2)
        else:
            print("Traffic Light: Unknown")
            cv2.putText(image, "Unknown", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (10, 10, 255), 2)
        distance = estimate_distance_y2(x, y, x+w, y+h, image,0)
        print("The estimated distance to traffic light is: ",distance)
    cv2.imshow("Image demage",image)
    #plt.show()
    cv2.waitKey(0)
# _________________________________________________________________


