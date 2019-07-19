#!/usr/bin/env python3
import math
import numpy as np
import rospy
import sensor_msgs.msg
from std_msgs.msg import Float32
from message_filters import ApproximateTimeSynchronizer, Subscriber

# Parameters
PI = math.pi
Kp = 1
max_angle = 30
max_speed = 300
near_arc_angle = 25
mid_arc_angle = 20  # 15  # 9
far_arc_angle = 2
near_threshold = 0.25
mid_threshold = 0.8
far_threshold = 5
start = 0
duration = 0.
direction = 0
carry_dir = 0.
carry_speed = 0
carry_angle = 0
# near_threshold = 2  # 0.3
# mid_threshold = 4
# far_threshold = 5
direct = 1
angle = 0.0
anglep = 0.0
speed = 0.0


def sigmoid(x):
    y = 1 / (1 + np.exp(-x))
    return y


def changeAngle(current_angle, destination_angle, change_rate):
    global new_angle
    if destination_angle - (change_rate + 2) < current_angle < destination_angle + (change_rate + 2):
        new_angle = current_angle
    if current_angle - destination_angle > 15:
        change_rate = change_rate * 2  #
        # if destination_angle == 0:
        #     new_angle = 0
    if current_angle < destination_angle:
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
    global carry_speed
    # if destination_speed - (change_rate + 2) < current_speed < destination_speed + (change_rate + 3):
    #     new_speed = 0.8*current_speed +0.2*destination_speed
    #     print("00")
    #     if destination_speed == 0:
    #         new_speed = 0

    if current_speed - destination_speed > 60:
        change_rate = 10  #
    elif current_speed - destination_speed > 30:
        change_rate = 5  #
    if current_speed < destination_speed:
        new_speed = current_speed + change_rate
    elif current_speed > destination_speed:
        new_speed = current_speed - change_rate * 2
    if current_speed < destination_speed + (change_rate + 2) and angle > destination_speed - (change_rate + 2):
        new_speed = current_speed
    if new_speed < 6:
        if new_speed > 0 and carry_speed < 100:
            carry_speed = carry_speed + new_speed
        else:
            carry_speed = 0
            new_speed = 0

    if new_speed > max_speed:
        new_speed = max_speed
    return new_speed


def pilot(steer):
    global anglep
    anglep = steer.data


def avoid(data):
    global start
    global duration
    global carry_dir
    global angle
    global anglep
    global speed
    global direct
    near_check = 0
    mid_check = 0
    mid_left_check = 0
    mid_right_check = 0
    # near = False
    # mid = False
    # free = False
    # stop = False
    mid_line = int(len(data.ranges) / 2)
    # increaseBy = data.angle_increment*180/PI
    ranges = np.array(data.ranges)
    ranges[np.isnan(ranges)] = 0.
    ranges[np.isinf(ranges)] = 10.
    np.warnings.filterwarnings('ignore')
    # #########~~~~~~~Time~~~~~~~##############----------------------------------
    """
    if start != 0:
        duration = (start - rospy.Time.now()).to_sec()
    carry_dir = carry_dir + (angle * speed * duration)
    if carry_dir > 0:
        direct = 1
    elif carry_dir < 0:
        direct = -1
    start = rospy.Time.now()
    print("mid line", data.ranges[mid_line])
    print("carry_dir", carry_dir)
    """
    ##########################################
    # scan_lines = np.arange(len(data.ranges))
    # print("mid_line",mid_line)
    near_arc_line = int(len(ranges) * near_arc_angle / 180)
    mid_arc_line = int(len(ranges) * mid_arc_angle / 180)
    left_sum = np.sum(ranges[0:int(len(ranges) / 2)])
    right_sum = np.sum(ranges[int(len(ranges) / 2):len(ranges)])
    if left_sum < right_sum:
        direct = 1
    else:
        direct = -1
    for line_n in range(mid_line - near_arc_line, mid_line + near_arc_line):
        # print("near = ", line_n)
        if ranges[line_n] < near_threshold:
            near_check = near_check + 1
            # near = True
            # break
    if near_check > 2 * near_arc_line * 0.1:  # if near arc line is blocked by more than 10%
        angle = changeAngle(angle, 0, 1)
        speed = changeSpeed(speed, 0, 1)

        print("I'm blocked")
        ##########################################################################################
    else:
        for line_m in range(mid_line - mid_arc_line, mid_line + mid_arc_line):
            # print("mid  = ", line_m)
            if ranges[line_m] < mid_threshold:  # if mid arc line is blocked by more than 10%
                mid_check = mid_check + 1
                if line_m < mid_line:
                    mid_left_check = mid_left_check + 1
                elif line_m > mid_line:  #
                    mid_right_check = mid_right_check + 1
                # mid = True
                # break
        if mid_check > 2 * mid_arc_line * 0.2:
            if mid_left_check < mid_right_check - 5:
                direct = -1
                print("      -1       ", mid_left_check, mid_right_check)
            elif mid_left_check > mid_right_check + 5:
                direct = 1
                print("      1       ", mid_left_check, mid_right_check)
            else:
                print("                                sum                                ", left_sum, right_sum)
                if left_sum < right_sum:
                    direct = 1
                else:
                    direct = -1
            angle =  changeAngle(angle, direct * 30, 3)  #direct * 30
            # angle = changeAngleSmooth(angle, -direct * 30, 0.1)
            speed = changeSpeed(speed, max_speed / 2, 5)
            # carry_dir = angle * speed
            print("Obstacle ahead")
            ##########################################################################################
        else:
            rospy.Subscriber("/Steering_Angel", Float32, pilot)
            angle = anglep  # changeAngle(steer, 0, 0.4)
            speed = changeSpeed(speed, max_speed/2, 10)
            print("It's a free world")
        ##########################################################################################
    # if stop:
    #     angle = changeAngle(angle, 0, 1)
    #     speed = changeSpeed(speed, 0, 5)
    #     print("Stop")


def main():
    global angle
    global speed

    rospy.init_node('listener', anonymous=True)

    robo_angle_pub = rospy.Publisher('robo_angle', Float32, queue_size=10)
    robo_speed_pub = rospy.Publisher('robo_speed', Float32, queue_size=10)
    rospy.Subscriber("scan", sensor_msgs.msg.LaserScan, avoid)

    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        msg_angle = angle
        msg_speed = speed
        print("                                s =", speed, "a =", angle)

        robo_angle_pub.publish(msg_angle)
        robo_speed_pub.publish(msg_speed)
        rate.sleep()


if __name__ == '__main__':
    main()
