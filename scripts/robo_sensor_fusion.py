#!/usr/bin/env python
import math
import cv2
import numpy as np
import rospy
import sensor_msgs.msg
from std_msgs.msg import Float32
from avoid_obstacles import BoundingBoxes.msg

# Parameters
PI = math.pi


def LaserScanProcess(data):
    a = 0


def main():
    rospy.Subscriber("scan", sensor_msgs.msg.LaserScan, LaserScanProcess)

    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
