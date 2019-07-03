#!/usr/bin/env python
import rospy

import cv2
import numpy as np
import math

from sensor_msgs.msg import LaserScan


def callback(data):
    p = 255
    p1 = 200
    frame_size = 1000
    mid_frame = int(frame_size/2)
    ymid_frame = mid_frame + 400
    frame = np.zeros((frame_size, frame_size, 3), np.uint8)
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
        size = frame_size/5
        x = math.trunc((r * 2 * size) * math.cos(angle + (-90 * pi / 180)))
        y = math.trunc((r * 2 * size) * math.sin(angle + (-90 * pi / 180)))
        # x = -x
        way_m = 9
        way_n = 25
        thres_m = 0.8
        thres_mx = math.trunc((thres_m * 2 * size) * math.cos(angle + (-90 * pi / 180)))
        thres_my = math.trunc((thres_m * 2 * size) * math.sin(angle + (-90 * pi / 180)))
        thres_n = 0.3
        thres_nx = math.trunc((thres_n * 2 * size) * math.cos(angle + (-90 * pi / 180)))
        thres_ny = math.trunc((thres_n * 2 * size) * math.sin(angle + (-90 * pi / 180)))
        # print (x)
        # cv2.line(frame, (x + mid_frame - 1, y + ymid_frame - 2), (x + mid_frame, y + ymid_frame), ((r)*255/5, (r)*255/5, (1-r/5)*255), 4)
        # cv2.line(frame, (mid_frame, ymid_frame), (x + mid_frame, y + ymid_frame), (55, 55, 55), 1)
        # cv2.line(frame, (mid_frame, ymid_frame), (x + mid_frame, y + ymid_frame), (55, 55, 55), 1)
        if -way_m*pi/180 < angle < way_m*pi/180:
            cv2.line(frame, (x + mid_frame - 1, y + ymid_frame-2), (x + mid_frame, y + ymid_frame), (20, 255, 255), 4)
            cv2.line(frame, (mid_frame, ymid_frame), (mid_frame + thres_nx, ymid_frame + thres_ny), (44, 44, 77), 1)
            cv2.line(frame, (mid_frame + thres_nx, ymid_frame + thres_ny), (mid_frame + thres_mx, ymid_frame + thres_my), (40, 70, 70), 1)
            cv2.line(frame, (mid_frame + thres_mx, ymid_frame + thres_my), (x + mid_frame, y + ymid_frame), (88, 55, 44), 1)
        elif -way_n*pi/180 < angle < way_n*pi/180:
            cv2.line(frame, (x + mid_frame - 1, y + ymid_frame-2), (x + mid_frame, y + ymid_frame), (20, 20, 255), 4)
            cv2.line(frame, (mid_frame, ymid_frame), (mid_frame + thres_nx, ymid_frame + thres_ny), (44, 44, 77), 1)
            cv2.line(frame, (mid_frame + thres_nx, ymid_frame + thres_ny), (x + mid_frame, y + ymid_frame), (88, 55, 44), 1)
        else:

            cv2.line(frame, (x + mid_frame - 1, y + ymid_frame-2), (x + mid_frame, y + ymid_frame), (255, 20, 20), 4)
            cv2.line(frame, (mid_frame, ymid_frame), (x + mid_frame, y + ymid_frame), (88, 55, 44), 1)
        ######################################################################
        cir = thres_m
        x1 = math.trunc((cir * 2 * size) * math.cos(angle + (-90 * pi / 180)))
        y1 = math.trunc((cir * 2 * size) * math.sin(angle + (-90 * pi / 180)))
        cv2.line(frame, (x1 + mid_frame - 1, y1 + ymid_frame-1), (x1 + mid_frame, y1 + ymid_frame), (55, 55, p1), 1)
        ######################################################################
        ######################################################################
        cir = thres_n
        x1 = math.trunc((cir * 2 * size) * math.cos(angle + (-90 * pi / 180)))
        y1 = math.trunc((cir * 2 * size) * math.sin(angle + (-90 * pi / 180)))
        cv2.line(frame, (x1 + mid_frame - 1, y1 + ymid_frame-1), (x1 + mid_frame, y1 + ymid_frame), (55, 55, p1), 1)
        ######################################################################

        angle = angle + data.angle_increment
    d = int(0.2*2*size)
    h = int(0.8*2*size)
    cv2.line(frame, (mid_frame + d, ymid_frame), (mid_frame + d, ymid_frame - h), (p1, p1, p1), 1)
    cv2.line(frame, (mid_frame - d, ymid_frame), (mid_frame - d, ymid_frame - h), (p1, p1, p1), 1)
    cv2.line(frame, (mid_frame - d, ymid_frame - h), (mid_frame + d, ymid_frame - h), (p1, p1, p1), 1)
    cv2.line(frame, (mid_frame, ymid_frame), (mid_frame, ymid_frame - h), (p/2, p/2, p/2), 1)
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
