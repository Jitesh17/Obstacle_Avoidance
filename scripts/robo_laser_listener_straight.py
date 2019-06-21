#!/usr/bin/env python
import math
import numpy as np
import rospy
import sensor_msgs.msg
from std_msgs.msg import Float32

speed = 0.0  # Always forward linear velocity.
THRESHOLD = 1.5  # THRESHOLD value for laser scan.
PI = math.pi
Kp = 1
angle = 0
max_angle = 30
max_speed = 80


def sigmoid(x):
    y = 1 / (1 + np.exp(-x))
    return y


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
    global speed
    global angle
    range_angels = np.arange(len(data.ranges))  # 0-511
    range_values = np.array(data.ranges)
    # print(ranges)
    # ranges[ranges == "inf"] = 100
    # print(ranges)
    np.warnings.filterwarnings('ignore')

    range_mask = (range_values > THRESHOLD)
    # ranges = list(range_angels[range_mask])

    range_values = range_values
    # max_gap = 20  # 40
    # print(range_angels)
    # print(range_mask)
    # print(ranges)
    """
    gap_list = []
    for k, g in groupby(enumerate(ranges), lambda (i,x):i-x):  #continuos no.
        gap_list.append(map(itemgetter(1), g))
    #gap_list.sort(key=len)
    #largest_gap = gap_list[-1]
    for path in range(len(gap_list)):
        mid_gap = gap_list[int(math.floor(len(gap_list)/2))]
        largest_gap = mid_gap
        #print(math.floor(len(gap_list)/2))
        #print(largest_gap)
        min_angle, max_angle = largest_gap[0]*((data.angle_increment)*180/PI), \
        largest_gap[-1]*((data.angle_increment)*180/PI)
        #print((data.angle_increment)*180/PI,largest_gap[0],largest_gap[-1])
        average_gap = (max_angle - min_angle)/2
        turn_angle = min_angle + average_gap
        if (average_gap < max_gap): #this path is small
            del gap_list[int(math.floor(len(gap_list)/2))]
        else:
            break
    """
    # print 'min angle=' ,min_angle, 'max angle=', max_angle
    # print max_gap,'gap/2=',average_gap,'turn angle= ',90.00020822390628-turn_angle

    #######################
    # print data.intensities
    # print data.angle_min
    # print data.angle_max
    # print data.angle_increment
    # print data.time_increment
    # print data.scan_time
    # print data.range_min
    # print data.range_max
    ########################
    line_no = []
    objects = []
    for i in range(0, len(data.ranges) - 1):
        if data.ranges[i] < 10:
            range_values[i] = data.ranges[i]
            line_no.append(i)
            # print i, data.ranges[i]
            # detect = 1
        else:
            # detect = 0
            range_values[i] = 100
            if len(line_no) > 1:
                objects.append(line_no)
            line_no = []
        # print(range_values[i])
    if len(objects) > 0:
        print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
        for j in range(len(objects)):
            object_line_no = (objects[j][0] + objects[j][-1]) / 2
            print(object_line_no * data.angle_increment * 180 / PI - 90, data.ranges[object_line_no])
    else:
        print('no object')

    #######################
    # print (average_gap)
    """
    if average_gap < max_gap:   # path block
        #angz = -max_angle #*random.choice([-1, 1])
        angle = changeAngle(angle, -max_angle, 1)
        #y = data.ranges[len(data.ranges)/2]
        y = min(data.ranges)
        speed = 0.125 * max_speed * sigmoid(y)

        print(angle, "********")
    else: 
        
        angle = changeAngle(angle, Kp * (90 - turn_angle), 1)
        y = data.ranges[len(data.ranges)/2]
        speed = speed + max_speed * sigmoid(y)
    if speed > max_speed:
        speed = max_speed
    if speed < 20:
        speed = 0
        
        angle = changeAngle(angle, 0, 1)
    """
    ########################
    """
    field = len(data.ranges)*3/8 #100
    for laser in range(len(data.ranges)/2-field, len(data.ranges)/2+field):
        if (data.ranges[laser]<2):
            LINX = LINX*0.1
            angz = -0.5"""
    speed = speed
    ########################

    ########################


def main():
    rospy.init_node('listener', anonymous=True)

    robo_angle_pub = rospy.Publisher('robo_angle', Float32, queue_size=1000)
    robo_speed_pub = rospy.Publisher('robo_speed', Float32, queue_size=1000)
    rospy.Subscriber("scan", sensor_msgs.msg.LaserScan, LaserScanProcess)

    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        # command = Twist()
        # command.linear.x = speed
        # command.angular.z = angle
        # pub.publish(command)
        msg_angle = angle
        msg_speed = speed
        # print msg_speed
        robo_angle_pub.publish(msg_angle)
        robo_speed_pub.publish(msg_speed)
        rate.sleep()


if __name__ == '__main__':
    main()
