import cv2
import numpy as np

def detect_buoy(img):
    # convert img to HSV
    img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    
    # inrange (will be in greyscale)
    img = cv2.inRange(img, np.array([60, 20, 0], np.uint8), np.array([191, 255, 255], np.uint8))
    
    # erosion
    kernel = np.ones((10, 10), np.uint8)
    img = cv2.erode(img, kernel)

    # find contours
    contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    # sort contours by area, get the largest one
    r = sorted(contours, key=cv2.contourArea)[-1]
    
    # make a bounding rectangle
    x, y, w, h = cv2.boundingRect(r)
    cv2.rectangle(img, (x, y), (x + w, y + h), (255), 2)

    # return three variables: buoy in image, (x, y, z), image for debugging
    
    # only consider it when the contour is large enough
    if w * h > 20000:
        x = x + w/2
        y = y + h/2
        z = (100000 - w*h) / 100000
        return True, (x, y, z), img

    # otherwise return empty
    return True, (-1, -1, -1), img    
