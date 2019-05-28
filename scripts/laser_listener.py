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
THRESHOLD = 4 #THRESHOLD value for laser scan.
PI = math.pi
Kp = 0.05
angz = 0

def sigmoid(x):
    y = 1/(1 + np.exp(-x))
    return y

def LaserScanProcess(data):
    range_angels = np.arange(len(data.ranges)) #0-511
    range_values = np.array(data.ranges)
    #print(ranges)
    #ranges[ranges == "inf"] = 100
    #print(ranges)
    np.warnings.filterwarnings('ignore')

    range_mask = (range_values > THRESHOLD)
    ranges = list(range_angels[range_mask])
    
    range_values = range_values
    max_gap = 40#40
    #print(range_angels)
    #print(range_mask)
    #print(ranges)
    
    gap_list = []
    for k, g in groupby(enumerate(ranges), lambda (i,x):i-x):  #continuos no.
        gap_list.append(map(itemgetter(1), g))
    #gap_list.sort(key=len)
    #largest_gap = gap_list[-1]
    mid_gap = gap_list[int(math.floor(len(gap_list)/2))]
    largest_gap = mid_gap
    print(math.floor(len(gap_list)/2))
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
    ######################## 
    line_no = []
    objects = []
    for i in range(0,len(data.ranges)-1):
        if data.ranges[i]<10:
            range_values[i] = data.ranges[i]
            line_no.append(i)
            #print i, data.ranges[i]
            detect = 1
        else:
            detect = 0
            range_values[i] = 100
            if len(line_no)>1:
                objects.append(line_no)
            line_no = []
        #print(range_values[i])
    if len(objects)>0:
        print '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
        for j in range(len(objects)):
            object_line_no = (objects[j][0] + objects[j][-1])/2
            #print object_line_no*data.angle_increment*180/PI-90, data.ranges[object_line_no]
    else:
        print 'no object'    
    
    #######################
    #print (average_gap)
    global LINX
    global angz
    if average_gap < max_gap:   # path block
        angz = -0.5 #*random.choice([-1, 1])
        #y = data.ranges[len(data.ranges)/2]
        y = min(data.ranges)
        LINX = 0.5*sigmoid(y)
        
        print(angz,"********")
    else:
        angz = Kp*(1)*(90 - turn_angle)
        y = data.ranges[len(data.ranges)/2]
        LINX = 2*0.5*sigmoid(y)
    ########################
    """
    field = len(data.ranges)*3/8 #100
    for laser in range(len(data.ranges)/2-field, len(data.ranges)/2+field):
        if (data.ranges[laser]<2):
            LINX = LINX*0.1
            angz = -0.5"""
    ########################
    print 'angz=',angz,'\nLINX=',LINX,'\nranges=',data.ranges[len(data.ranges)/2] 
    ########################
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
