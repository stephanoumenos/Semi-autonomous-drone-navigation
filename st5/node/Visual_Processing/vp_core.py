import cv2
import numpy as np

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
        x1, y1, x2, y2 = line[:4]
        cv2.line(line_img, (x1,y1), (x2, y2), color, thickness)

    # Integrates it with original image
    img_with_lines = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)

    return img_with_lines

def lsd_detector(image):
    # Returns the detected lines for a given image
    lsd_detector          = cv2.createLineSegmentDetector()
    lines, _, _, _ = lsd_detector.detect(image)

    # Avoids the writing of each line as a 2D table
    # TODO: Possible solution without copying list
    flattened_lines = []
    for line in lines:
        flattened_lines.append(np.ravel(line))
    #for i, line in enumerate(lines):
        # print("ORIGINAL LINE: ")
        # print(line)
        # print("RAVEL: ")
        # print(np.ravel(line))
        # lines[i] = np.ravel(line)
        # print("FINAL RESULT")
        # print(lines[i])

    return flattened_lines

def draw(image, now):
    # Takes a image and returns it overlayed with detected lines and vanishing point
    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    detected_lines = lsd_detector(gray_image)
    if detected_lines is None:
        # If no lines are found, we can return the empty image
        # Impossible to find vanishing point
        return image
    
    image_with_lines = draw_lines(image, detected_lines)
    return image_with_lines
