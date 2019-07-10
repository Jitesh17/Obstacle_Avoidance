import cv2 as cv
import cv2
# 3s20mwhn
import numpy as np
import matplotlib.pyplot as plt


def do_canny(f):
    gray = cv.cvtColor(f, cv.COLOR_RGB2GRAY)
    blur = cv.GaussianBlur(gray, (5, 5), 0)
    canny = cv.Canny(blur, 50, 150)
    return canny


def do_segment(f):
    height = f.shape[0]
    polygons = np.array([[(0, height), (800, height), (380, 290)]])
    mask = np.zeros_like(f)
    cv.fillPoly(mask, polygons, 255)
    segment = cv.bitwise_and(f, mask)
    return segment


def calculate_lines(f, lines):
    left = []
    right = []
    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        parameters = np.polyfit((x1, x2),(y1, y2), 1)
        slope = parameters[0]
        y_intercept = parameters[1]
        if slope < 0:
            left.append((slope, y_intercept))
        else:
            right.append((slope, y_intercept))
    left_avg = np.average(left, axis=0)
    right_avg = np.average(left, axis=0)
    left_line = calculate_coordinates(f, left_avg)
    right_line = calculate_coordinates(f, right_avg)
    return np.array([left_line, right_line])


def calculate_coordinates(f, parameters):
    slope, intercept = parameters
    y1 = f.shape[0]
    y2 = int(y1 - 150)
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    return np.array([x1, y1, x2, y2])


def visualize_lines(f, lines_):
    lines_visualize_ = np.zeros_like(f)
    if lines_ is not None:
        for x1, y1, x2, y2 in lines_:
            cv.line(lines_visualize_, (x1, y1), (x2, y2), (0, 255, 0), 5)
    return lines_visualize_


    ######################

# capture frames from a camera 
# cap = cv2.VideoCapture(0)

# loop runs if capturing has been initialized 
# reads frames from a camera
# ret, frame = cap.read()
frame = cv2.imread("../Images/a.png")
"""
    #cv2.imshow('Edges',frame) 
    lower = np.array([0,120,70])
    upper = np.array([10,255,255]) 
    mask1 = cv2.inRange(frame, lower, upper)
    lower = np.array([170,120,70])
    upper = np.array([180,255,255]) 
    mask2 = cv2.inRange(frame, lower, upper)
    mask = mask1+mask2
    """
# white
# low = 200
# h = 255
# lower = np.array([low / 2, low, low])
# # upper = np.array([255, 150, 90])
# # upper = np.array([105, 105, 255])
# upper = np.array([255, 255, 255])
# mask_white = cv2.inRange(frame, lower, upper)
# output = cv2.bitwise_and(frame, frame, mask=mask_white)
# cv2.imshow("color", np.hstack([frame, output]))
#########################################


canny = do_canny(frame)
segment1 = do_segment(canny)
hough = cv2.HoughLinesP(segment1, 2, np.pi / 180, 100, np.array([]), 100, 50)
lines = calculate_lines(frame, hough)
lines_visualize = visualize_lines(frame, lines)
output = cv.addWeighted(frame, 0.9, lines_visualize, 1, 1)
cv.imshow("output", output)
"""
frame1 = frame
gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray, 150, 550)
lines = cv2.HoughLinesP(gray, 1, np.pi / 180, 50, 50, 10)
if lines is None:  # sum(gray)>100:
    pass
else:

    for x1, y1, x2, y2 in lines[0]:
        cv2.line(frame1, (x1, y1), (x2, y2), (255, 0, 0), 2, 3)
    cv2.imshow("color", np.hstack([frame1, output]))
    cv2.imshow("edges", edges)
# Wait for Esc key to stop
"""

# Close the window 
# cap.release()
# cv2.imshow("edges", output)
cv2.waitKey(0)
# De-allocate any associated memory usage 
cv2.destroyAllWindows()
