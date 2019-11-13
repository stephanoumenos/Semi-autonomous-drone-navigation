import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import math
import sys
from scipy import stats

VERTICAL_ANGLE_LIMIT = 75
#draw_lines algorithm

def draw_lines(img, lines, color = [255,0,0], thickness = 3):
    # Takes an image and coordinates of desired lines and returns an image with the drawn lines
    if len(lines)==0:
        return
    
    #making a copy of the original image
    img = np.copy(img)

    line_img = np.zeros(
        (
            img.shape[0],
            img.shape[1],
            3
        ),
        dtype=np.uint8,
    )

    # Loop over all the lines and draw them on the blank image.
    
    for line in lines:
        print(line)
        x1, y1, x2, y2 = line[:4]
        cv2.line(line_img, (x1,y1),(x2,y2),color, thickness)

    # Merge the image with the lines onto the original
    img = cv2.addWeighted(img,0.8,line_img, 1.0, 0.0)

    return img

def draw_points(img, points, color = [255,0,0], thickness = 3):
    # Takes an image and coordinates of desired lines and returns an image with the drawn points
    if len(points)==0:
        return
    
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
        x1 = math.floor(point[0])
        y1 = math.floor(point[1])
        cv2.circle(point_img, (x1,y1) ,10 ,color, thickness)
        #print(x1,y1)

    # Merge the image with the lines onto the original
    img = cv2.addWeighted(img,0.8,point_img, 1.0, 0.0)

    return img

#line intersection algorithm
def intersect(lines):
    #print(lines)
    intersections = []
    for i, si in enumerate(lines):
        for sj in lines[i+1:]:
            cross_product = np.cross(si[4:6], sj[4:6]) # [a1,b1] ^ [a2, b2]
            if cross_product != 0:
                coeff = 1.0 / cross_product

                intersections.append([coeff * np.cross(si[5:7]   , sj[5:7]), # [b1, c1] ^ [b2, c2]
                                      coeff * np.cross(sj[[4, 6]], si[[4, 6]])]) # -[a1, c1] ^ [a2, c2]
    return np.array(intersections)

# Removing outliers
def sci(values, confidence) :
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

def draw(image, now):

    #reading image
    #image = cv2.imread(frame, cv2.IMREAD_COLOR)

    #plt.figure()
    #plt.imshow(image)

    # convert to grayscale
    gray_image = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)

    #Line Segment detector algorithm
    cannyed_image = cv2.Canny(gray_image,100,200)

    lines = cv2.HoughLinesP(
        cannyed_image,
        rho=6,
        theta = np.pi/60,
        threshold=160,
        lines=np.array([]),
        minLineLength=90,
        maxLineGap=25
    ) 

    if lines is None:
        return image


    clean_list= []
    # cleaning vertical lines
    angle = np.deg2rad(VERTICAL_ANGLE_LIMIT)
    for line in lines:
        for x1, y1, x2, y2 in line:
            delta_x = x2 - x1
            delta_y = y2 - y1
            if delta_x == 0:
                proportion = np.inf
            else:
                proportion = delta_y/delta_x

            
            if abs(proportion) <= np.tan(angle):
                clean_list.append([x1, y1, x2, y2])
    
    if len(clean_list) == 0:    
        return image

    new_list = []
    for x1,y1,x2,y2 in clean_list:
        a = y1-y2
        b = x2-x1
        c = x1*(y2-y1)+ y1*(x1-x2)
        n = math.sqrt((y2-y1)**2 + (x2-x1)**2)
        new_list.append([x1, y1, x2, y2, a, b, c, n])

    image_with_lines = draw_lines(image, new_list)
    #return image_with_lines


    # # #finding intersection points
    # intersection_points = intersect(new_list)
    # if len(intersection_points) == 0:
    #     return image_with_lines

    # # # Removing intersection points outside image
    # new_int_points=[]
    # for point in intersection_points:
    #     if point[0] > 0 and point[1] > 0:
    #         new_int_points.append(point)
    # if len(new_int_points) == 0:
    #     return image_with_lines
        

    # x_int_points = np.array([a[0] for a in new_int_points])
    # y_int_points = np.array([a[1] for a in new_int_points])

    # x_confidence_interval = sci(x_int_points, 0.3)
    # y_confidence_interval = sci(y_int_points, 0.3)
    # if x_confidence_interval.all() == -1 or y_confidence_interval.all() == -1:
    #     return image

    # valid_points =[]
    # for element in new_int_points:
    #     if element[0] in x_confidence_interval and element[1] in y_confidence_interval:
    #     # if x_confidence_interval[0] <= element[0] <= x_confidence_interval[1]:
    #     #     if y_confidence_interval[0] <= element[1] <= y_confidence_interval[1]:
    #         valid_points.append(element)
    # if len(valid_points) == 0:
    #     return valid_points

    # #finding the centroid
    # centroid= np.mean(valid_points,axis=0)
    # centroid = map(int, centroid)
    # #print(centroid)

    # #OUTPUT

    # #print(intersection_points)
    # # if centroid is tuple:
    # new_image = draw_points(image,[centroid])
    # # else:
    # #new_image = image
    # #line_image = draw_lines(image,clean_list)
    # return new_image
    # #plt.figure()
    # #plt.imshow(new_image)

    # #plt.show()