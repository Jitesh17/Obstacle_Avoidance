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
    frame_size = 800
    mid_frame = int(frame_size/2)
    ymid_frame = mid_frame + 200
    frame = np.zeros((frame_size, frame_size, 3), np.uint8)
    angle = data.angle_min
    pi = data.angle_max - data.angle_min
    np.warnings.filterwarnings('ignore')
    print(distanceAt(20, data.ranges))
    for r in data.ranges:
        assert isinstance(r, object)

        if r == np.inf:
            r = 10.
        if np.isnan(r):
            r = 0.
        if angle == 15*pi/180:
            print(r)
        x = math.trunc((r * 40) * math.cos(angle + (-90 * pi / 180)))
        y = math.trunc((r * 40) * math.sin(angle + (-90 * pi / 180)))
        x = -x
        way_m = 2
        way_n = 20
        if -way_m*pi/180 < angle < way_m*pi/180:
            cv2.line(frame, (mid_frame, ymid_frame), (x + mid_frame, y + ymid_frame), (0, 0, 255), 4)
        elif -way_n*pi/180 < angle < way_n*pi/180:
            cv2.line(frame, (mid_frame, ymid_frame), (x + mid_frame, y + ymid_frame), (0, 255, 255), 4)
        else:
            cv2.line(frame, (mid_frame, ymid_frame), (x + mid_frame, y + ymid_frame), (255, 0, 0), 2)

        angle = angle + data.angle_increment

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
