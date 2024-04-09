from PIL import Image
from matplotlib import pyplot as plt
import os
import cv2
import numpy as np
from copy import deepcopy
import time

def fit_line_ransac(contours, max_iterations=70, distance_threshold=1, right_constant =0.2):
    '''
    This function fits a line to the given contours using RANSAC algorithm. 
    And it tries to choose the line that is the most right in the image.
    The smaller the right_constant, the more the line will be on the right side of the image and not the biggest line.
    inputs:
        contours: list of numpy arrays <-- list of contours
        max_iterations: int <-- maximum number of iterations
        distance_threshold: int <-- distance threshold for inliers
        right_constant: float <-- constant for the right side of the image
    outputs:
        best_line: numpy array <-- 4x1 array, where first two elements are the direction of the line and the last two elements are a point on the line
    '''
    best_line = np.zeros((4, 1))
    max_inliers = 0
    best_average = np.zeros((2, 1))
    for i in range(max_iterations):
        # get two random points from the contours
        points = np.random.choice(np.arange(len(contours)), 2, replace=False)
        point1 = contours[points[0]]
        point2 = contours[points[1]]
        # get the direction of the line
        direction = point2 - point1
        # get the normal of the direction
        normal = np.array([direction[1], -direction[0]])
        # normalize the normal
        normal = normal / np.linalg.norm(normal)
        # get the point on the line
        point = (point1 + point2) / 2
        # get the distances of the points from the line
        distances = np.abs(np.dot(contours - point, normal))
        # get the inliers
        inliers = np.where(distances < distance_threshold)[0]
        # if the number of inliers is greater than the max_inliers
        if len(inliers) > max_inliers:
            # get average psition of inliners
            average = np.mean(contours[inliers], axis=0)
            if len(inliers) > max_inliers * right_constant:
                if average[0] > best_average[0]:
                    best_average = average
                    # update the max_inliers
                    max_inliers = len(inliers)
                    # fit the line with the inliers
                    best_line = cv2.fitLine(contours[inliers], cv2.DIST_L2, 0, 0.01, 0.01)

    return best_line



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

def detect_road(image):
    '''
    Finds a line between the road and the white background in the image.
    inputs:
        image: numpy array <-- rgb image
    outputs:
        best_line: numpy array <-- 4x1 array, where first two elements are the direction of the line and the last two elements are a point on the line
    '''
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
    
    if len(contours[0]) < 20:
        return None

    # all points of the coutour[0] on the left side of the image are deleted
    points = np.array([])
    for point in contours[0]:
        if point[0][0] > np.size(image, 1) / 2:
            points = np.append(points, point)

    if len(points) < 20:
        line = fit_line_ransac(contours[0].reshape(-1, 2))
    else:
        line = fit_line_ransac(points.reshape(-1, 2))

    return line

def detect_yellow_line(img):
    '''
    Finds a line best fitting the yellow line in the image.
    inputs:
        image: numpy array <-- rgb image
    outputs:
        best_line: numpy array <-- 4x1 array, where first two elements are the direction of the line and the last two elements are a point on the line
    '''
    image = deepcopy(img)
    # make upper half of the image black
    image[:image.shape[0]//2, :] = 0

    connected_components = make_cc(image, [30,50,50])

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


    contours, _ = cv2.findContours(sign_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


    line = fit_line_ransac(contours[0].reshape(-1, 2))
    return line


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

if __name__ == "__main__":
    path = ".\\images\\auto_traffic24.jpg"
    image = cv2.imread(path)
    start_time = time.time()
    road_line = detect_road(image)
    road_time = time.time() - start_time
    yellow_line = detect_yellow_line(image)
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


    plt.figure()

    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # draw stop sign bounding box
    if bb_stop is not None:
        x,y,w,h, area = bb_stop
        cv2.rectangle(image, (x, y), (x+w, y+h), (10, 10, 255), 2)
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
    else:
        print("No traffic light detected")

    rows, cols = image.shape[:2]
    if road_line is not None:
        # draw road line
        lefty = int((-road_line[2] * road_line[1] / road_line[0]) + road_line[3])
        righty = int(((cols - road_line[2]) * road_line[1] / road_line[0]) + road_line[3])
        image = cv2.line(image, (cols, righty), (0, lefty), (255, 10, 10), 2)
    else:
        print("No road line detected")

    if yellow_line is not None:
        # draw yellow line
        lefty = int((-yellow_line[2] * yellow_line[1] / yellow_line[0]) + yellow_line[3])
        righty = int(((cols - yellow_line[2]) * yellow_line[1] / yellow_line[0]) + yellow_line[3])
        image = cv2.line(image, (cols, righty), (0, lefty), (10, 255, 10), 2)
    else:
        print("No yellow line detected")
        
    plt.imshow(image)
    plt.show()
    
    print("Done")
