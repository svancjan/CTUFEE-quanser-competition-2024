import numpy as np
import matplotlib.pyplot as plt

CAR_WIDTH = 0.21

def get_center_line(lines:tuple[np.ndarray | None, np.ndarray | None]) -> np.ndarray | None:
    index = None
    two_lines = True
    for idx, item in enumerate(lines):
        if item is not None:
            index = idx
        else:
            two_lines = False
    
    if two_lines:
        # compute the centerline as average of the two lines
        right_line, left_line = lines
        center_line = (left_line + right_line)/2
    elif index:
        offset = 1.15 * CAR_WIDTH/2
        line = lines[index]
        if index == 0:
            center_line = offset_curve(line, offset)
        else:
            center_line = offset_curve(line, -offset)
    else:
        center_line = None

    return center_line

def offset_curve(points:np.ndarray, distance):
    """
    Offset a curve defined by a series of points by a specified distance.
    
    Args:
    - points (numpy.ndarray): Array of shape (N, 2) representing N points (x, y) on the curve.
    - distance (float): Distance by which to offset the curve.
    
    Returns:
    - numpy.ndarray: Array of shape (N, 2) representing N points (x, y) on the offset curve.
    """
    # Calculate unit direction vectors along the curve
    diffs = np.diff(points, axis=0)
    directions = np.arctan2(diffs[:, 1], diffs[:, 0])
    
    # Calculate normal vectors of the curve
    normals_x = np.cos(directions + np.pi/2)
    normals_y = np.sin(directions + np.pi/2)
    
    # Scale normals to desired distance
    offset_normals_x = normals_x * distance
    offset_normals_y = normals_y * distance
    
    # Adjust the shape of the offset normals array
    offset_normals = np.vstack((offset_normals_x, offset_normals_y))
    first_normal = offset_normals[:, 0].reshape((2, 1))
    offset_normals = np.hstack((first_normal, offset_normals))

    # Offset the points
    offset_points = points + offset_normals.T
    
    return offset_points


if __name__ == "__main__":
    # Define points on the curve
    x = np.arange(0,10,0.1) 
    y1 = np.cos(x)*np.sin(x)
    y2 = np.cos(x)*np.sin(x) + 0.6

    right_line = np.column_stack((x, y1))
    left_line = np.column_stack((x, y2))



    # Offset the curve by 0.5 units
    offset_distance = 0.21/2

    # possible scenarios
    
    # center_line = get_center_line((right_line, left_line))
    center_line = get_center_line((None, left_line))
    # center_line = get_center_line((right_line, None))
    # center_line = get_center_line((None, None))

    # Visualize the curves
    plt.figure()
    plt.plot(right_line[:, 0], right_line[:, 1], label='right', color='black', linewidth=1)
    plt.plot(left_line[:, 0], left_line[:, 1], label='left', color='gold', linewidth=1)
    if center_line is not None:
        plt.plot(center_line[:, 0], center_line[:, 1], label='center', color='blue', linewidth=1)
    plt.xlim((0,10))
    plt.ylim((-4,4))
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Offset Curve Visualization')
    plt.gca().set_aspect('equal', adjustable='box')
    plt.legend()
    plt.grid(True)
    plt.show()