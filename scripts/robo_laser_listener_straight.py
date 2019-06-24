#!/usr/bin/env python
import math
import numpy as np
import rospy
import sensor_msgs.msg
from std_msgs.msg import Float32

PI = math.pi
Kp = 1
max_angle = 30
max_speed = 200
near_arc_angle = 18
mid_arc_angle = 4
far_arc_angle = 2
near_threshold = 0.3
mid_threshold = 0.8
far_threshold = 5
start = rospy.Time.now()
direction = 0
carry_dir = 0
# near_threshold = 2  # 0.3
# mid_threshold = 4
# far_threshold = 5
direct = 1


def sigmoid(x):
    y = 1 / (1 + np.exp(-x))
    return y


def distanceAt(range_angle, dr):
    n = (range_angle - 90) / 180 * len(dr)
    d = dr[int(n)]
    return d


def changeAngle(current_angle, destination_angle, change_rate):
    global new_angle
    if current_angle - destination_angle > 15:
        change_rate = change_rate*3  #
    if destination_angle - (change_rate + 2) < current_angle < destination_angle + (change_rate + 2):
        new_angle = current_angle
        # if destination_angle == 0:
        #     new_angle = 0
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
    if current_speed - destination_speed > 60:
        change_rate = 10  #
    elif current_speed - destination_speed > 30:
        change_rate = 5  #
    if destination_speed - (change_rate + 2) < current_speed < destination_speed + (change_rate + 3):
        new_speed = current_speed
        if destination_speed == 0:
            new_speed = 0
    elif current_speed < destination_speed:
        new_speed = current_speed + change_rate
    elif current_speed > destination_speed:
        new_speed = current_speed - change_rate*2
    if current_speed < destination_speed + (change_rate + 2) and angle > destination_speed - (change_rate + 2):
        new_speed = current_speed
    if new_speed < 6:
        new_speed = 0
    if new_speed > max_speed:
        new_speed = max_speed
    return new_speed


def LaserScanProcess(data):
    global angle
    global speed
    global direct
    near = False
    mid = False
    free = False
    stop = False
    ranges = np.array(data.ranges)
    ranges[np.isnan(ranges)] = 0.
    ranges[np.isinf(ranges)] = 10.
    np.warnings.filterwarnings('ignore')
    ########################
    print(distanceAt(0, ranges))
    for line_n in range(-near_arc_angle, near_arc_angle):
        if distanceAt(line_n, ranges) < near_threshold:
            near = True
            break
    if near:
        angle = changeAngle(angle, 0, 1)
        speed = changeSpeed(speed, 0, 1)
        # carry_dir = angle * speed
        print("I'm blocked")
    else:
        for line_m in range(-mid_arc_angle, mid_arc_angle):
            if distanceAt(line_m, ranges) < mid_threshold:
                mid = True
                break
    if mid:
        angle = changeAngle(angle, 30, 3)
        speed = changeSpeed(speed, direct * 50, 5)
        # carry_dir = angle * speed
        print("Obstacle ahead")
    elif not near:
        free = True
    if free:
        angle = changeAngle(angle, 0, 1)
        speed = changeSpeed(speed, max_speed, 10)
        print("It's a free world")
    if stop:
        angle = changeAngle(angle, 0, 1)
        speed = changeSpeed(speed, 0, 5)
        print("Stop")


def main():
    global angle
    global speed
    angle = 0.0
    speed = 0.0
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
"""
        elif False:　#　carry_dir !=0:
            angle = changeAngle(angle, -direct*30, 1)
            speed = changeSpeed(speed, -direct*50, 5)
        """
