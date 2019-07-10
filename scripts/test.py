import cv2 as cv
import cv2
# 3s20mwhn
import numpy as np

frame = cv2.imread("../Images/lightsoff.png")

y_dir = frame.shape[0]
x_dir = frame.shape[1]
polygons = np.array([[(0, y_dir-150), (0, int(y_dir*6/16)), (int(x_dir/4), int(y_dir/4)), (int(x_dir*3/4), int(y_dir/4)), (x_dir, int(y_dir*6/16)), (x_dir, y_dir-150)]])
mask = np.zeros_like(frame)
cv.fillPoly(mask, polygons, 255)
segment = cv.bitwise_and(frame, mask)

cv2.imshow("seg", np.hstack([frame, segment]))

cv2.waitKey(0)
# De-allocate any associated memory usage
cv2.destroyAllWindows()