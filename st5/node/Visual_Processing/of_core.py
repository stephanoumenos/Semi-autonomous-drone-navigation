import cv2
import numpy as np
import scipy.signal 

def coordinate_transform(y, hmin, hmax, ymin, ymax):
    deltah = hmax - hmin
    deltay = ymax - ymin

    y_transformed = []
    for y_point in y:
        y_transformed_point = (deltah * y_point / deltay) + hmin - ymin
        y_transformed.append(y_transformed_point)

    return y_transformed

def draw_function(image, y, hmin, hmax, ymin, ymax, color =[255,0,0], thickness=3):
    y_transformed = coordinate_transform(y, hmin, hmax, ymin, ymax)

    img = np.copy(image)

    # Creates an "mask" of lines
    line_img = np.zeros(
        (
            img.shape[0],
            img.shape[1],
            3
        ),
        dtype=np.uint8,
    )    

    # Draws the function over the picture
    previous_point = (0, y_transformed[0])
    for i, y_point in enumerate(y_transformed[1:]):
        cv2.line(line_img, previous_point, (i, y_point), color, thickness)
        previous_point = (i, y_point)

    # Integrates it with original image
    img_with_lines = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)

    return img_with_lines

def median_filter(values, medfilt_size):
    filtered = scipy.signal.medfilt(values, medfilt_size) # median filter
    filtered = map(int, filtered)

    return filtered

def shift(curY, prevY):
    print(np.convolve(curY, prevY))
def draw(image, previous_y, now):
    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    y_values = gray_image[127]
    
    y_values = median_filter(y_values, 11)

    current_overlay = draw_function(image, y_values, 150, 300, 0, 255)
    if len(previous_y) == 0:
        return current_overlay, y_values
    
    shift(y_values, previous_y)

    overlayed_image = draw_function(current_overlay, previous_y, 150, 300, 0, 255, color=[0,255,0])
    return overlayed_image, y_values 