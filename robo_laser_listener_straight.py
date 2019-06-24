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
start = 0  # rospy.Time.now()
duration = 0.
direction = 0
carry_dir = 0.
# near_threshold = 2  # 0.3
# mid_threshold = 4
# far_threshold = 5
direct = 1


def sigmoid(x):
    y = 1 / (1 + np.exp(-x))
    return y


def distanceAt(range_angle, dr):
    # n = (range_angle - 90) / 180 * len(dr)
    # d = dr[int(n)]
    d = dr[range_angle]
    return d


def changeAngle(current_angle, destination_angle, change_rate):
    global new_angle
    if current_angle - destination_angle > 15:
        change_rate = change_rate * 3  #
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
        new_speed = current_speed - change_rate * 2
    if current_speed < destination_speed + (change_rate + 2) and angle > destination_speed - (change_rate + 2):
        new_speed = current_speed
    if new_speed < 6:
        new_speed = 0
    if new_speed > max_speed:
        new_speed = max_speed
    return new_speed


def LaserScanProcess(data):
    global start
    global duration
    global carry_dir
    global angle
    global speed
    global direct
    near_check = 0
    mid_check = 0
    # near = False
    # mid = False
    free = False
    stop = False
    mid_line = int(len(data.ranges)/2)
    # increaseBy = data.angle_increment*180/PI
    ranges = np.array(data.ranges)
    ranges[np.isnan(ranges)] = 0.
    ranges[np.isinf(ranges)] = 10.
    np.warnings.filterwarnings('ignore')
    # #########~~~~~~~Time~~~~~~~##############
    if start != 0:
        duration = (start - rospy.Time.now()).to_sec()
    carry_dir = carry_dir + (-angle * speed * duration)
    if carry_dir > 0:
        direct = 1
    elif carry_dir < 0:
        direct = -1
    start = rospy.Time.now()
    print(data.ranges[mid_line])
    print("carry_dir", carry_dir)
    ##########################################
    # scan_lines = np.arange(len(data.ranges))
    # print("mid_line",mid_line)
    near_arc_line = int(len(data.ranges) * near_arc_angle / 180)
    mid_arc_line = int(len(data.ranges) * mid_arc_angle / 180)
    for line_n in range(mid_line - near_arc_line, mid_line + near_arc_line):
        # print("near = ", line_n)
        if data.ranges[line_n] < near_threshold:
            near_check = near_check+1
            # near = True
            # break
    if near_check > 2 * near_arc_line * 0.2:
        angle = changeAngle(angle, 0, 1)
        speed = changeSpeed(speed, 0, 1)

        print("I'm blocked")
    else:
        for line_m in range(mid_line - mid_arc_line, mid_line + mid_arc_line):
            # print("mid  = ", line_m)
            if data.ranges[line_m] < mid_threshold:
                mid_check = mid_check + 1
                # mid = True
                # break
        if mid_check > 2 * mid_arc_line * 0.2:
            angle = changeAngle(angle, direct * 30, 3)
            speed = changeSpeed(speed, max_speed/2, 5)
            # carry_dir = angle * speed
            print("Obstacle ahead")
        else:
            if -50 < carry_dir < 0:
                angle = changeAngle(angle, 0, 1)
            else:
                angle = changeAngle(angle, -direct * 30, 1)
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
