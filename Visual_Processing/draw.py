import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import math
import sys
from scipy import stats

#manipulating region of interest of photo

def region_of_interest(img, vertices):
    # Define a blank matrix that matches the image height/width.
    mask = np.zeros_like(img)    # Retrieve the number of color channels of the image.

    match_mask_color = 255
      
    # Fill inside the polygon
    cv2.fillPoly(mask, vertices, match_mask_color)
    
    # Returning the image only where mask pixels match
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

#draw_lines algorithm

def draw_lines(img, lines, color = [255,0,0], thickness = 3):
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
        for x1, y1, x2, y2 in line:
            cv2.line(line_img, (x1,y1),(x2,y2),color, thickness)

    # Merge the image with the lines onto the original
    img = cv2.addWeighted(img,0.8,line_img, 1.0, 0.0)

    return img

def draw_points(img, points, color = [255,0,0], thickness = 3):
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

#defining confidence interval

def sci(values, confidence) :
    nb        = values.shape[0]
    values    = np.sort(values)
    size      = (int)(nb*confidence+.5)
    nb_iter   = nb - size + 1
    sci       = None
    sci_width = sys.float_info.max
    inf       = 0
    sup       = size
    for i in range(nb_iter) :
        sciw = values[sup-1] - values[inf]
        if sciw < sci_width :
            sci       = values[inf:sup]
            sci_width = sciw
        inf += 1
        sup += 1
    return sci

#defining region of interest
region_of_interest_vertices = [(0, 50),(350,50),(350, 350),(0,350)]

#reading image
image = mpimg.imread('Visual_Processing/corridor1.jpg')

#plt.figure()
#plt.imshow(image)

# convert to grayscale
gray_image = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)

#Line Segment detector algorithm
cannyed_image = cv2.Canny(gray_image,100,200)

#cropping _ line segment image
cropped_image = region_of_interest(cannyed_image, np.array([region_of_interest_vertices], np.int32),)

lines = cv2.HoughLinesP(
    cropped_image,
    rho=6,
    theta = np.pi/60,
    threshold=160,
    lines=np.array([]),
    minLineLength=150,
    maxLineGap=25
) 

angle = np.deg2rad(75)

clean_list= []

# cleaning vertical lines
for line in lines:
    for x1, y1, x2, y2 in line:
        delta_x = x2 - x1
        delta_y = y2 - y1
        prop = delta_y/delta_x
        if abs(prop) < np.tan(angle):
            clean_list.append(line)

clean_list = np.array(clean_list)

new_list = []

for line in clean_list:
    for x1,y1,x2,y2 in line:
        a = y1-y2
        b = x2-x1
        c = x1*(y2-y1)+ y1*(x1-x2)
        n = math.sqrt((y2-y1)**2 + (x2-x1)**2)
        new_list.append(np.concatenate((np.array(line),np.array([a,b,c,n])),axis=None))

#finding intersection points
intersection_points = intersect(new_list)

# Removing intersection points outside image
new_int_points=[]
for point in intersection_points:
    if point[0] > 0 and point[1] > 0:
        new_int_points.append(point)


x_int_points = np.array([a[0] for a in new_int_points])
y_int_points = np.array([a[1] for a in new_int_points])

x_confidence_interval = sci(x_int_points,0.30)
y_confidence_interval = sci(y_int_points, 0.3)

valid_points =[]
for element in new_int_points:
    if x_confidence_interval[0] <= element[0] <= x_confidence_interval[1] and y_confidence_interval[0] <= element[1] <= y_confidence_interval[1]:
        valid_points.append(element)

#finding the centroid
centroid= np.mean(valid_points,axis=0)
print(centroid)
    
#OUTPUT

#print(intersection_points)
new_image = draw_points(image,[centroid])
#line_image = draw_lines(image,clean_list)

plt.figure()
plt.imshow(new_image)

plt.show()