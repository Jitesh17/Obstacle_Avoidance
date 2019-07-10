import cv2 as cv
import cv2
# 3s20mwhn
import numpy as np
import matplotlib.pyplot as plt


def do_canny(f):
    gray = cv.cvtColor(f, cv.COLOR_RGB2GRAY)
    blur = cv.GaussianBlur(gray, (5, 5), 0)
    canny = cv.Canny(blur, 100, 550)
    return canny


def do_segment(f):
    y_dir = frame.shape[0]
    x_dir = frame.shape[1]
    polygons = np.array([[(0, y_dir-150), (0, int(y_dir*6/16)), (int(x_dir/4), int(y_dir/4)), (int(x_dir*3/4), int(y_dir/4)), (x_dir, int(y_dir*6/16)), (x_dir, y_dir-150)]])
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


frame = cv2.imread("../Images/2564.jpg")
# frame = cv2.imread("../Images/a.png")
# frame = cv2.imread("../Images/lane.jpg")
# print(frame[0,0])
frame1 = frame
print(np.average(frame[0]), np.average(frame[1]), np.average(frame[2]))
lower = np.array([0, 100, 0])
upper = np.array([160, 255, 150])
mask_green = cv2.inRange(frame, lower, upper)
mask_green = 255 - mask_green
output1 = cv2.bitwise_and(frame, frame, mask=mask_green)
cv.imshow("frame", output1)
# frame1 = output1[:, :]
canny = do_canny(frame1)
segment1 = do_segment(canny)

cv.imshow("frame11", canny)
#
# segment1 = canny
haha = cv2.cvtColor(segment1,cv2.COLOR_GRAY2RGB)
cv2.imshow("edge", np.hstack([frame, haha]))
# cv.imshow("segment1", segment1)
# hough = cv2.HoughLinesP(segment1, 2, np.pi / 180, 100, np.array([]), 100, 50)
# if hough is not None:
#     lines = calculate_lines(frame1, hough)
#     lines_visualize = visualize_lines(frame1, lines)
#     output = cv.addWeighted(frame, 0.9, lines_visualize, 1, 1)
#     # cv.imshow("segment1", segment1)
#     # cv.imshow("frame", frame)
#     # cv.imshow("output", output)
#     cv2.imshow("Lane", np.hstack([output, output1]))
#     cv2.imshow("Lane Detect", np.hstack([frame, segment1]))
cv2.waitKey(0)
# De-allocate any associated memory usage 
cv2.destroyAllWindows()
