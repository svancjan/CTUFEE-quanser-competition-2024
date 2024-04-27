from PIL import Image
from matplotlib import pyplot as plt
import os
import cv2
import numpy as np
import json

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
    range = 20
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

def h2e(array):
    array = np.array(array)
    output_array = []
    for i in range(array.shape[0]):
        output_array.append(array[i][0:-1]/array[i][-1])
    return np.array(output_array)

def e2h(array):
    array = np.array(array)
    output_array = []
    for i in range(array.shape[0]):
        output_array.append(np.append(array[i],1))
    return np.array(output_array)

def make_car_coords(img_pts, H):
    # convert image points to homogeneous coordinates
    img_pts_h = e2h(img_pts)
    # convert image points to car coordinates
    car_pts_h = np.dot(H, img_pts_h.T).T
    # convert car coordinates to euclidean coordinates
    car_pts = h2e(car_pts_h)
    return car_pts

def make_treshold(image, treshold):
    '''
    inputs:
        image: numpy array <-- rgb image
        treshold: int <-- treshold value
    outputs:
        black_map: numpy array <-- binary image, where 1 represents black pixels
    '''
    black_map = np.zeros_like(image[:, :, 0])
    red_treshold = image[:, :, 0] < treshold
    green_treshold = image[:, :, 1] < treshold
    blue_treshold = image[:, :, 2] < treshold
    black_map[red_treshold & green_treshold & blue_treshold] = 1
    return black_map

def detect_road(img, M, distance_treshold = 0.5):
    '''
    Finds a line between the road and the white background in the image.
    inputs:
        img: numpy array <-- rgb image
        M: numpy array <-- 3x3 array, homography matrix
        distance_treshold: float <-- treshold for the distance from the top of the image that I dont use for detection
    outputs:
        pts_in_car_coords: numpy array <-- nx2 array. Points of the road line in car coordinates.
                         : None <-- if no road line is detected
        '''
    image = deepcopy(img)
    img_black_tresh = make_treshold(image, 100)

    kernel = np.ones((20, 20), np.uint8)
    closed_img = cv2.morphologyEx(img_black_tresh, cv2.MORPH_CLOSE, kernel)

    connected_components = cv2.connectedComponentsWithStats(
        closed_img.astype(np.uint8), connectivity=4)

    # get the biggest connected component
    max_area = 0
    max_area_index = 0
    for i in range(1, connected_components[0]):
        if connected_components[2][i][4] > max_area:
            max_area = connected_components[2][i][4]
            max_area_index = i

    if max_area < 200:
        return None
            

    sign_img = np.zeros_like(connected_components[1], dtype=np.uint8)
    map = np.where(connected_components[1] == max_area_index)
    sign_img[map] = 1


    contours, _ = cv2.findContours(sign_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        return None
    convex_hull = cv2.convexHull(contours[0])
    cv2.drawContours(sign_img, [convex_hull], -1, 1, 2)
    
    if len(contours) == 0:
        return None
    
    if len(contours[0]) < 20:
        return None
    
    krajnice = np.zeros_like(img_black_tresh)
    cv2.drawContours(krajnice, [convex_hull], 0, 1, 1)
    # make all border px 0
    krajnice[0, :] = 0
    krajnice[-1, :] = 0
    krajnice[:, 0] = 0
    krajnice[:, -1] = 0

    # all pixels that has x value lower than half of the image width are 0
    krajnice[:int(np.size(krajnice, 1) * distance_treshold),:] = 0

    num_labels, labeled_image = cv2.connectedComponents(krajnice)

    # Initialize variables to keep track of the rightmost component
    max_rightmost_point = 0
    rightmost_component_label = -1

    # Iterate through each connected component
    for label in range(1, num_labels):  # Skip background label 0
        # Find all pixels belonging to the current component
        component_pixels = np.where(labeled_image == label)
        
        # Find the rightmost point of the component
        rightmost_point = np.max(component_pixels[1])
        
        # If this component's rightmost point is greater than the current max, update
        if rightmost_point > max_rightmost_point:
            max_rightmost_point = rightmost_point
            rightmost_component_label = label

    points = np.array(np.where(labeled_image == rightmost_component_label)).T

    points = points[:, ::-1]
    if len(points) < 5:
        return None

    pts_in_car_coords = make_car_coords(points, M)

    return pts_in_car_coords

def detect_yellow_line(img,M, distance_treshold = 0.6):
    '''
    Finds a line best fitting the yellow line in the image.
    inputs:
        image: numpy array <-- rgb image
        M: numpy array <-- 3x3 array, homography matrix
    outputs:
        best_line: numpy array <-- 4x1 array, where first two elements are the direction of the line and the last two elements are a point on the line
    '''
    image = deepcopy(img)
    # make upper half of the image black
    image[:image.shape[0]//2, :] = 0

    connected_components = make_cc(image, [35,50,50])

    # get the biggest connected component
    max_area = 0
    max_area_index = 0
    for i in range(1, connected_components[0]):
        if connected_components[2][i][4] > max_area:
            max_area = connected_components[2][i][4]
            max_area_index = i
    if max_area < 150:
        return None
    
    sign_img = np.zeros_like(connected_components[1], dtype=np.uint8)
    map = np.where(connected_components[1] == max_area_index)
    sign_img[map] = 1

    # all pixels that has x value lower than the magic distnace treshold of the image width are 0
    sign_img[:int(np.size(sign_img, 1) * distance_treshold),:] = 0

    # fix irregulaities in the shape of the line
    contours = cv2.findContours(sign_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    if len(contours) == 0:
        return None
    convex_hull = cv2.convexHull(contours[0])
    cv2.drawContours(sign_img, [convex_hull], -1, 1, 2)


    kernel = np.ones((5,5),np.uint8)
    yellow_mask = cv2.morphologyEx(sign_img, cv2.MORPH_CLOSE, kernel)

    # Compute the gradient along the horizontal axis
    gradient = cv2.Sobel(yellow_mask, cv2.CV_64F, 1, 0, ksize=7)
    # Normalize the gradient to 0-255
    gradient = cv2.normalize(gradient, None, 0, 100, cv2.NORM_MINMAX)

    yellow_line_mask = np.where(gradient < 40)
    yellow_line = np.zeros_like(gradient, dtype=np.uint8)
    yellow_line[yellow_line_mask] = 1

    num_labels, labeled_image = cv2.connectedComponents(yellow_line)


    # Initialize variables to keep track of the rightmost component
    max_rightmost_point = 0
    rightmost_component_label = -1

    # Iterate through each connected component
    for label in range(1, num_labels):  # Skip background label 0
        # Find all pixels belonging to the current component
        component_pixels = np.where(labeled_image == label)
        
        # Find the rightmost point of the component
        rightmost_point = np.max(component_pixels[1])
        
        # If this component's rightmost point is greater than the current max, update
        if rightmost_point > max_rightmost_point:
            max_rightmost_point = rightmost_point
            rightmost_component_label = label

    points = np.array(np.where(labeled_image == rightmost_component_label)).T

    points = points[:, ::-1]
    if len(points) < 5:
        return None

    pts_in_car_coords = make_car_coords(points, M)

    return pts_in_car_coords


def detect_stop_sign(img):
    '''
    Detects the stop sign in the image.
    inputs:
        image: numpy array <-- rgb image
    outputs:
        bb: numpy array <-- 4x1 array, where first two elements are x,y position of the top left corner and the last two are width and height of the bounding box
        '''
    image = deepcopy(img)

    connected_components = make_cc(image, [3,80,80])
    sign_img = np.zeros_like(connected_components[1], dtype=np.uint8)
    map = np.where(connected_components[1] != 0)

    sign_img[map] = 1
    kernel = np.ones((10, 10), np.uint8)
    closed_img = cv2.morphologyEx(sign_img, cv2.MORPH_CLOSE, kernel)

    # get connected componenents again
    connected_components = cv2.connectedComponentsWithStats(
        closed_img, connectivity=8)

    # get bounding boxes of all componennts but the first one is the background
    bounding_boxes = connected_components[2][1:]

    # find bounding box with almost square ratio in bounding boxes
    found_box  = False
    best_index = None
    for i in range(len(bounding_boxes)):
        x, y, w, h, area = bounding_boxes[i]
        if area > 200 and w*h*0.7 < area:
            #print("area", area)
            #print("w*h", w*h)
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
    connected_components = make_cc(image, [40,50,50])
    sign_img = np.zeros_like(connected_components[1], dtype=np.uint8)
    map = np.where(connected_components[1] != 0)
    sign_img[map] = 1
    kernel = np.ones((10, 10), np.uint8)
    closed_img = cv2.morphologyEx(sign_img, cv2.MORPH_CLOSE, kernel)
    # get connected componenents again
    connected_components = cv2.connectedComponentsWithStats(
        closed_img, connectivity=8)
    # get bounding boxes of all componennts
    bounding_boxes = connected_components[2][1:]

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

def estimate_distance_y2(xmin, ymin, xmax, ymax, image,flag):
    
    # Camera intrinsic matrix
    # focal_length_x = 318.86 #CSI front 
    # principal_point_x = 401.34 #CSI front 
    # focal_length_y = 312.14 #CSI front 
    # principal_point_y = 201.50 #CSI front
    
    focal_length_x = 455.20 
    principal_point_x = 308.53
    focal_length_y = 459.49
    principal_point_y = 213.55
    
    # focal_length_x = 481.06 
    # principal_point_x = 320.36
    # focal_length_y = 479.76
    # principal_point_y = 235.64
    
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
    #print("Computed distance in x direction:",distance_to_camera_x)
    #print("Computed distance in y direction:",distance_to_camera_y)
    #print("\n")
    return distance_to_camera_x,distance_to_camera_y

if __name__ == "__main__":
    path = ".\\images\\edge1.png"
    with open('RS_road_h.json', 'r') as f:
        M = np.array(json.load(f))
    image = cv2.imread(path)
    start_time = time.time()
    road_line = detect_road(image, M, 0.45)
    road_time = time.time() - start_time
    yellow_line = detect_yellow_line(image, M, 0.45)
    yellow_time = time.time() - start_time - road_time
    bb_stop = detect_stop_sign(image)
    stop_time = time.time() - start_time - road_time - yellow_time
    bb_semafor = detect_traffic_lights(image)
    semafor_time = time.time() - start_time - road_time - yellow_time - stop_time
    print("Time: ", time.time() - start_time)
    print("Road line time:", road_time)
    print("Yellow line time:", yellow_time)
    print("Stop sign time:", stop_time)
    print("Traffic light time:", semafor_time)



    # draw stop sign bounding box
    if bb_stop is not None:
        x,y,w,h, area = bb_stop
        cv2.rectangle(image, (x, y), (x+w, y+h), (10, 10, 255), 2)
        stop_distance = estimate_distance_y2(x, y, x+w, y+h, image,1)
        print("The estimated distance to stop sign is: ",stop_distance)
    else:
        print("No stop sign detected")

    # draw traffic light bounding box
    if bb_semafor is not None:
        bb, state = bb_semafor
        x,y,w,h,area = bb
        cv2.rectangle(image, (x, y), (x+w, y+h), (10, 10, 255), 2)
        if state == 0:
            print("Red")
            cv2.putText(image, "Red", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (10, 10, 255), 2)
        elif state == 1:
            print("Green")
            cv2.putText(image, "Green", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (10, 10, 255), 2)
        else:
            print("Unknown")
            cv2.putText(image, "Unknown", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (10, 10, 255), 2)
        traffic_distance = estimate_distance_y2(x, y, x+w, y+h, image,0)
        print("The estimated distance to traffic light is: ",traffic_distance)
    else:
        print("No traffic light detected")

    plt.subplot(1,2,1)
    plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    plt.title("Original Image")


    rows, cols = image.shape[:2]
    if road_line is not None or yellow_line is not None:
        plt.subplot(1,2,2)

    if road_line is not None:
        print("road line shape", road_line.shape)
        # draw road line
        plt.scatter(road_line[:, 0], road_line[:, 1], color="black")
    else:
        print("No road line detected")

    if yellow_line is not None:
        # draw yellow line
        plt.scatter(yellow_line[:, 0], yellow_line[:, 1], color="yellow")
    else:
        print("No yellow line detected")
    
    if bb_stop is not None:
        # draw large red dot
        plt.scatter(0, np.mean(stop_distance), marker="h", color="red",s=300)
    if bb_semafor is not None:
        # draw large yellow dot
        plt.scatter(0, np.mean(traffic_distance), marker="s", color="green",s=300)
    
    plt.legend(["Road line", "Yellow line", "Stop sign", "Traffic light"])

    
    plt.axis('equal')
    plt.title("car coordinates")
    plt.show()
        
    
    
    print("Done")
