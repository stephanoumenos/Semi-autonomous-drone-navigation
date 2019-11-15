import cv2
import numpy as np
import math
import sys
import random

def draw_lines(img, lines, color = [255,0,0], thickness = 3):
    # Draws the lines described into lines over the given image
    # img = image data, lines = list of lines
    # Note that individual lines should follow (x1, y1, x2, y2)
   
    img = np.copy(img)

    # Creates an "mask" of lines
    line_img = np.zeros(
        (
            img.shape[0],
            img.shape[1],
            3
        ),
        dtype=np.uint8,
    )    

    # Draws the lines on the "mask"
    for line in lines:
        x1, y1, x2, y2 = map(int, line[:4])
        cv2.line(line_img, (x1,y1), (x2, y2), color, thickness)

    # Integrates it with original image
    img_with_lines = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)

    return img_with_lines

def draw_points(img, points, color = [255,0,0], thickness = 3):
    # Takes an image and coordinates of desired lines and returns an image with the drawn points
    
    #making a copy of the original image
    img = np.copy(img)

    point_img = np.zeros(
        (
            img.shape[0],
            img.shape[1],
            3
        ),
        dtype=np.uint8,
    )

    # Loop over all the lines and draw them on the blank image.
    for point in points:
        x1 = int(math.floor(point[0]))
        y1 = int(math.floor(point[1]))
        cv2.circle(point_img, (x1,y1) ,10 ,color, thickness)
        

    # Merge the image with the lines onto the original
    img = cv2.addWeighted(img,0.8,point_img, 1.0, 0.0)

    return img

def lsd_detector(image):
    # Returns the detected lines for a given image
    lsd_detector          = cv2.createLineSegmentDetector()
    lines, _, _, _ = lsd_detector.detect(image)
    
    return np.squeeze(lines).tolist()

def append_canonical(lines):
    canonical_lines = []
    for x1,y1,x2,y2 in lines:
        a = y1-y2
        b = x2-x1
        c = x1*(y2-y1)+ y1*(x1-x2)
        n = math.sqrt((y2-y1)**2 + (x2-x1)**2)
        canonical_lines.append([x1, y1, x2, y2, a, b, c, n])
    
    return canonical_lines

def intersect(lines):
    intersections = []
    for i, si in enumerate(lines):
        si = np.array(si)
        for sj in lines[i+1:]:
            sj = np.array(sj)
            cross_product = np.cross(si[4:6], sj[4:6]) # [a1,b1] ^ [a2, b2]
            if cross_product != 0:
                coeff = 1.0 / cross_product

                intersections.append([coeff * np.cross(si[5:7]   , sj[5:7]), # [b1, c1] ^ [b2, c2]
                                      coeff * np.cross(sj[[4, 6]], si[[4, 6]])]) # -[a1, c1] ^ [a2, c2]
    return np.array(intersections)

def intersect_lines(lines):

    # Appends canonical representation and length to list
    canonical_lines = append_canonical(lines)

    # Find the intersections between the lines
    intersection_points = intersect(canonical_lines)
    return intersection_points

# Filters intersection to inside screen
def filter_intersections(image, intersections):
    filtered_points = []
    for point in intersections:
        if image.shape[1] > point[0] >= 0 and image.shape[0] > point[1] >= 0:
            filtered_points.append(point)
    return filtered_points

def sci(values, confidence):
    nb        = values.shape[0]
    values    = np.sort(values)
    size      = (int)(nb*confidence+.5)
    nb_iter   = nb - size + 1
    sci       = None
    sci_width = sys.float_info.max
    inf       = 0
    sup       = size
    if sup == 0:
        return -1
    for i in range(nb_iter) :
        sciw = values[sup-1] - values[inf]
        if sciw < sci_width :
            sci       = values[inf:sup]
            sci_width = sciw
        inf += 1
        sup += 1
    return sci


def compute_centroid(points):
    x_points = np.array([point[0] for point in points])
    y_points = np.array([point[1] for point in points])

    x_confidence_interval = sci(x_points, 0.3)
    y_confidence_interval = sci(y_points, 0.3)

    centroid = [np.median(x_confidence_interval), np.median(y_confidence_interval)]
    
    return centroid


def compute_line_angle(line):
    x1, y1, x2, y2 = line
    delta_x = x2 - x1
    delta_y = y2 - y1
    return np.rad2deg(abs(np.arctan2(delta_y, delta_x)))


def filter_lines(lines, min_angle, max_angle, min_length, absolute=True):
    """
    Filters the lines according to the parameters. If absolute is true,
    it will convert angles higher than 90 to between 0 and 90
    """
    non_vertical_lines = []

    if lines is None or lines == []:
        return None
    elif isinstance(lines[0], float):
        lines = [lines]

    for line in lines:
        line_angle = compute_line_angle(line)
        if line_angle > 90 and absolute:
            line_angle = 180 - line_angle   

        if min_angle < line_angle < max_angle and line_length(line) > min_length:
            non_vertical_lines.append(line)

    
    return non_vertical_lines


def line_length(line):
    x1, y1, x2, y2 = line
    return math.sqrt((x2-x1)**2 + (y2-y1)**2)


def remove_lines_with_vp(lines, vanishing_point, cutoff_dist):
    p_point = np.asarray(vanishing_point)

    lines_after_vp = []
    for line in lines:
        x1, y1, x2, y2 = line
        p1_line = np.array([x1,y1])
        p2_line = np.array([x2,y2])

        dist = np.linalg.norm(np.cross(p2_line-p1_line, p1_line-p_point))/np.linalg.norm(p2_line-p1_line)
        if dist < cutoff_dist:
            lines_after_vp.append(line)
    
    return lines_after_vp
        

def draw(image, now):
    # Takes a image and returns it overlayed with detected lines and vanishing point
    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    # Detect lines in the image
    detected_lines = lsd_detector(gray_image)
    # If no lines are found, we can return the empty image
        # Impossible to find vanishing point
    filtered_lines = filter_lines(detected_lines, 10, 80, 50)
    if filtered_lines is None:
        return image
    
    intersection_points = intersect_lines(filtered_lines) 
    filtered_points = filter_intersections(image, intersection_points)
    
    #Finds centroid using a confidence interval method
    vanishing_point = compute_centroid(filtered_points)

    lines_after_vp = remove_lines_with_vp(filtered_lines, vanishing_point, 10)

    left_lines = filter_lines(lines_after_vp, 0, 90, 0, absolute=False)
    if left_lines:
        left_angles = map(compute_line_angle, left_lines)
        random_angle_left = sum(left_angles)/len(left_angles)
    else:
        random_angle_left = None

    right_lines = filter_lines(lines_after_vp, 90, 180, 0, absolute=False)
    if right_lines:
        right_angles = map(compute_line_angle, right_lines)
        random_angle_right = sum(right_angles)/len(right_angles)
    else:
        random_angle_right = None

    #intersections = intersect_lines()
    image_with_lines = draw_lines(image, lines_after_vp)
    image_with_points = draw_points(image_with_lines, [vanishing_point])
    
    return image_with_points, vanishing_point, random_angle_left, random_angle_right
