import cv2
import numpy as np
import math

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
        print(point)
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

def draw(image, now):
    # Takes a image and returns it overlayed with detected lines and vanishing point
    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    # Detect lines in the image
    detected_lines = lsd_detector(gray_image)
    # If no lines are found, we can return the empty image
        # Impossible to find vanishing point
    if detected_lines is None:
        return image
    
    intersection_points = intersect_lines(detected_lines) 
    filtered_points = filter_intersections(image, intersection_points)

    
    #intersections = intersect_lines()
    image_with_lines = draw_lines(image, detected_lines)
    image_with_points = draw_points(image_with_lines, filtered_points)
    
    return image_with_points
