#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sensor_msgs.msg
import random
import numpy as np
import math
import random
from geometry_msgs.msg import Twist
from itertools import *
from operator import itemgetter

LINX = 0.0 #Always forward linear velocity.
THRESHOLD = 1.5 #THRESHOLD value for laser scan.
PI = math.pi
Kp = 0.05
angz = 0

def LaserScanProcess(data):
    range_angels = np.arange(len(data.ranges)) #0-719
    ranges = np.array(data.ranges)
    range_mask = (ranges > THRESHOLD)
    ranges = list(range_angels[range_mask])
    max_gap = 40
    # print(ranges)
    print(len(data.ranges))
    gap_list = []
    for k, g in groupby(enumerate(ranges), lambda (i,x):i-x):
        gap_list.append(map(itemgetter(1), g))
    gap_list.sort(key=len)
    largest_gap = gap_list[-1]
    #print(gap_list)
    #print(largest_gap)
    min_angle, max_angle = largest_gap[0]*((data.angle_increment)*180/PI), largest_gap[-1]*((data.angle_increment)*180/PI)
    #print((data.angle_increment)*180/PI,largest_gap[0],largest_gap[-1])
    average_gap = (max_angle - min_angle)/2

    turn_angle = min_angle + average_gap

    #print 'min angle=' ,min_angle, 'max angle=', max_angle
    #print max_gap,'gap/2=',average_gap,'turn angle= ',90.00020822390628-turn_angle
    
    #######################
    #print data.intensities
    #print data.angle_min
    #print data.angle_max
    #print data.angle_increment
    #print data.time_increment
    #print data.scan_time
    #print data.range_min
    #print data.range_max
    line_no = []
    objects = []
    for i in range(0,len(data.ranges)-1):
        if data.ranges[i]<10:
            line_no.append(i)
            #print i, data.ranges[i]
            detect = 1
        else:
            detect = 0
            if len(line_no)>1:
                objects.append(line_no)
            line_no = []
        
    if len(objects)>0:
        print '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
        for j in range(len(objects)):
            object_line_no = (objects[j][0] + objects[j][-1])/2
            print object_line_no*data.angle_increment*180/PI-90, data.ranges[object_line_no]
    else:
        print 'no object'    

    #######################

    global LINX
    global angz
    if average_gap < max_gap:
        angz = -0.5*random.randint({-1},{1})
        print(angz,"********")
    else:
        LINX = 0.5
        angz = Kp*(-1)*(90 - turn_angle)

def main():
    rospy.init_node('listener', anonymous=True)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("scan", sensor_msgs.msg.LaserScan , LaserScanProcess)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        command = Twist()
        command.linear.x = LINX
        command.angular.z = angz
        pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    main()
