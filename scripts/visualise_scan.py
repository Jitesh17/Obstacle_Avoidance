#!/usr/bin/env python
import rospy

import cv2
import numpy as np
import math

from sensor_msgs.msg import LaserScan


def distanceAt(angle, dr):
    n = (angle-90)/180*len(dr)
    d = dr[int(n)]
    return d


def callback(data):
    p = 255
    frame_size = 1000
    mid_frame = int(frame_size/2)
    ymid_frame = mid_frame + 200
    frame = np.zeros((frame_size, frame_size+150, 3), np.uint8)
    angle = data.angle_min
    pi = data.angle_max - data.angle_min
    np.warnings.filterwarnings('ignore')
    # print(distanceAt(20, data.ranges))
    for r in data.ranges:
        assert isinstance(r, object)

        if r == np.inf:
            r = 10.
        l = 8.
        if r > l:
            r = l
        if np.isnan(r):
            r = 0.
        if angle == 15*pi/180:
            print(r)
        size = frame_size/10
        x = math.trunc((r * 2 * size) * math.cos(angle + (-90 * pi / 180)))
        y = math.trunc((r * 2 * size) * math.sin(angle + (-90 * pi / 180)))
        # x = -x
        way_m = 4
        way_n = 20
        # print (x)
        # cv2.line(frame, (x + mid_frame - 1, y + ymid_frame - 2), (x + mid_frame, y + ymid_frame), ((r)*255/5, (r)*255/5, (1-r/5)*255), 4)
        cv2.line(frame, (mid_frame, ymid_frame), (x + mid_frame, y + ymid_frame), (55, 55, 55), 1)
        # cv2.line(frame, (mid_frame, ymid_frame), (x + mid_frame, y + ymid_frame), (55, 55, 55), 1)
        if -way_m*pi/180 < angle < way_m*pi/180:
            cv2.line(frame, (x + mid_frame - 1, y + ymid_frame-2), (x + mid_frame, y + ymid_frame), (20, 20, 255), 4)
            cv2.line(frame, (mid_frame, ymid_frame), (x + mid_frame, y + ymid_frame), (44, 44, 77), 1)
        elif -way_n*pi/180 < angle < way_n*pi/180:
            cv2.line(frame, (x + mid_frame - 1, y + ymid_frame-2), (x + mid_frame, y + ymid_frame), (20, 255, 255), 4)
            cv2.line(frame, (mid_frame, ymid_frame), (x + mid_frame, y + ymid_frame), (40, 70, 70), 1)
        else:
            cv2.line(frame, (x + mid_frame - 1, y + ymid_frame-2), (x + mid_frame, y + ymid_frame), (255, 20, 20), 4)
            cv2.line(frame, (mid_frame, ymid_frame), (x + mid_frame, y + ymid_frame), (88, 55, 44), 1)
        ######################################################################
        cir = 0.8
        x1 = math.trunc((cir * 2 * size) * math.cos(angle + (-90 * pi / 180)))
        y1 = math.trunc((cir * 2 * size) * math.sin(angle + (-90 * pi / 180)))
        cv2.line(frame, (x1 + mid_frame - 1, y1 + ymid_frame-2), (x1 + mid_frame, y1 + ymid_frame), (55, 55, 255), 1)
        ######################################################################
        ######################################################################
        cir = 0.3
        x1 = math.trunc((cir * 2 * size) * math.cos(angle + (-90 * pi / 180)))
        y1 = math.trunc((cir * 2 * size) * math.sin(angle + (-90 * pi / 180)))
        cv2.line(frame, (x1 + mid_frame - 1, y1 + ymid_frame-2), (x1 + mid_frame, y1 + ymid_frame), (55, 55, 255), 1)
        ######################################################################

        angle = angle + data.angle_increment
    d = int(0.3*2*size)
    h = int(0.8*2*size)
    cv2.line(frame, (mid_frame + d, ymid_frame), (mid_frame + d, ymid_frame - h), (p, p, p), 1)
    cv2.line(frame, (mid_frame - d, ymid_frame), (mid_frame - d, ymid_frame - h), (p, p, p), 1)
    cv2.line(frame, (mid_frame - d, ymid_frame - h), (mid_frame + d, ymid_frame - h), (p, p, p), 1)
    cv2.line(frame, (mid_frame, ymid_frame), (mid_frame, ymid_frame - h), (p, p, p), 1)
    cv2.circle(frame, (mid_frame, ymid_frame), 2, (255, 255, 0))
    cv2.circle(frame, (mid_frame, ymid_frame-300), 2, (0, 0, 255))
    cv2.imshow('frame', frame)
    cv2.waitKey(1)


def laser_listener():
    rospy.init_node('laser_listener', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()


if __name__ == '__main__':
    laser_listener()
