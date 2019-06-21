#!/usr/bin/env python
import math
import numpy as np
import rospy
import sensor_msgs.msg
from std_msgs.msg import Float32

speed: float = 0.0  # Always forward linear velocity.
PI = math.pi
Kp = 1
angle = 0
max_angle = 30
max_speed = 80


def sigmoid(x):
    y = 1 / (1 + np.exp(-x))
    return y


def distanceAt(range_angle, dr):
    n = (range_angle - 90) / 180 * len(dr)
    d = dr[int(n)]
    return d


def changeAngle(current_angle, destination_angle, change_rate):
    global new_angle
    if current_angle < destination_angle + (change_rate + 2) and angle > destination_angle - (change_rate + 2):
        new_angle = current_angle
    elif current_angle < destination_angle:
        new_angle = current_angle + change_rate
    elif current_angle > destination_angle:
        new_angle = current_angle - change_rate
    if new_angle < -max_angle:
        new_angle = -max_angle
    if new_angle > max_angle:
        new_angle = max_angle
    return new_angle


def changeSpeed(current_speed, destination_speed, change_rate):
    global new_speed
    if current_speed < destination_speed + (change_rate + 2) and angle > destination_speed - (change_rate + 2):
        new_speed = current_speed
    elif current_speed < destination_speed:
        new_speed = current_speed + change_rate
    elif current_speed > destination_speed:
        new_speed = current_speed - change_rate
    if new_speed < -max_speed:
        new_speed = -max_speed
    if new_speed > max_speed:
        new_speed = max_speed
    return new_speed


def LaserScanProcess(data):
    ranges = np.array(data.ranges)
    ranges[np.isnan(ranges)] = 0.
    ranges[np.isinf(ranges)] = 10.
    np.warnings.filterwarnings('ignore')
    ########################


def main():
    rospy.init_node('listener', anonymous=True)

    robo_angle_pub = rospy.Publisher('robo_angle', Float32, queue_size=1000)
    robo_speed_pub = rospy.Publisher('robo_speed', Float32, queue_size=1000)
    rospy.Subscriber("scan", sensor_msgs.msg.LaserScan, LaserScanProcess)

    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        msg_angle = angle
        msg_speed = speed
        robo_angle_pub.publish(msg_angle)
        robo_speed_pub.publish(msg_speed)
        rate.sleep()


if __name__ == '__main__':
    main()
