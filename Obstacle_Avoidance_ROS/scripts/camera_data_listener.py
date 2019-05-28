#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sensor_msgs.msg
import random
import numpy as np
import math
from geometry_msgs.msg import Twist
from itertools import *
from operator import itemgetter

LINX = 0.0 #Always forward linear velocity.
THRESHOLD = 1.5 #THRESHOLD value for laser scan.
PI = 3.14
Kp = 0.05
angz = 0

def CamProcess(data):
    
    if (data.roi.x_offset!=0)or(data.roi.height!=0):
        print 'height=',data.height
        print 'width=',data.width
        print 'distortion_model=',data.distortion_model
        print 'D=', data.D#float64[] D
        print 'K=',data.K #float64[9] K
        print 'R=',data.R #float64[9] R
        print 'P=',data.P #float64[12] P
        print 'binning_x=',data.binning_x #binning_x
        print 'binning_y=',data.binning_y
        print 'x offset=',data.roi.x_offset
        print 'height=',data.roi.height
        print 'width=',data.roi.width
    '''
    range_angels = np.arange(len(data.ranges))
    ranges = np.array(data.ranges)
    range_mask = (ranges > THRESHOLD)
    ranges = list(range_angels[range_mask])
    max_gap = 40
    # print(ranges)
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
    
    ################# Detect Obstacle range and angle ################
    line_no = []
    objects = []
    for i in range(0,719):
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
        print '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
        for j in range(len(objects)):
            object_line_no = (objects[j][0] + objects[j][-1])/2
            ############# Angle and Distance from scanner ################################
            #print object_line_no*data.angle_increment*180/PI-90, data.ranges[object_line_no]
            #######################################################################################
            r_square = data.ranges[object_line_no]*data.ranges[object_line_no]
            d_lid_cam_square= 0.7*0.7 + 0.3*0.3 #from my_robot_tutorial.urdf
            d_camera = math.sqrt(r_square + d_lid_cam_square - 1.4*data.ranges[object_line_no]*math.cos(object_line_no*data.angle_increment - PI/2))
            angle_cam_o = math.acos((r_square - d_camera*d_camera - 0.49)/(1.4*d_camera))*180/PI
            ############# Angle and Distance from camera ################################
            print 'A=',angle_cam_o,' D=', d_camera
    else: 
        print 'No obstacle found'    

    #################################################################
    global LINX
    global angz
    if average_gap < max_gap:
        angz = -0.5
    else:
        LINX = 1.5
        angz = Kp*(-1)*(90 - turn_angle)
'''

def main():
    rospy.init_node('listener', anonymous=True)

    #pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.Subscriber("/camera_info", sensor_msgs.msg.CameraInfo , CamProcess)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        #command = Twist()
        #command.linear.x = LINX
        #command.angular.z = angz
        #pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    main()
