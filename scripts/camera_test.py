#!/usr/bin/env python
import math
import numpy as np
import rospy
import sensor_msgs.msg
from std_msgs.msg import Float32
from avoid_obstacles.msg import BoundingBoxes
stop = False


def camera(data):
    global stop, prob, y_max
    stop = False
    num = len(data.bounding_boxes)
    real_y_max = 0
    for i in range(0, num):
        c = "stop sign"
        m = data.bounding_boxes[i].class_name
        if  m == c:  # data.bounding_boxes[i].ymax < 400 and
            Class_name = data.bounding_boxes[i].class_name
            prob = data.bounding_boxes[i].probability
            x_min = data.bounding_boxes[i].xmin
            x_max = data.bounding_boxes[i].xmax
            y_min = data.bounding_boxes[i].ymin
            y_max = data.bounding_boxes[i].ymax
            if real_y_max < y_max and prob > 0.4:  # and Class_name != "chair"
                real_y_max = y_max
    if real_y_max >200:
        stop = True
    # print(stop)


def main():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, camera)

    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        print(stop)
    #     msg_angle = angle
    #     msg_speed = speed
    #     print("                                s =", speed, "a =", angle)
    #     robo_angle_pub.publish(msg_angle)
    #     robo_speed_pub.publish(msg_speed)
        rate.sleep()


if __name__ == '__main__':
    main()