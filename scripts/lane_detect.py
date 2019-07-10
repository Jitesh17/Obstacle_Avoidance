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


# capture frames from a camera 
cap = cv2.VideoCapture(0)

# loop runs if capturing has been initialized 
# reads frames from a camera
ret, frame = cap.read()
# frame = cv2.imread("../Images/30cm.png")
while 1:
    canny = do_canny(frame)
    segment1 = do_segment(canny)
    hough = cv2.HoughLinesP(segment1, 2, np.pi / 180, 100, np.array([]), 100, 50)
    lines = calculate_lines(frame, hough)
    lines_visualize = visualize_lines(frame, lines)
    output = cv.addWeighted(frame, 0.9, lines_visualize, 1, 1)
    cv.imshow("output", output)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

# Close the window 
cap.release()
# cv2.imshow("edges", output)
# cv2.waitKey(0)
# De-allocate any associated memory usage 
cv2.destroyAllWindows()
