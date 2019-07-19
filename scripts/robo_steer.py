#!/usr/bin/env python
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
mid_arc_angle = 15  # 9
far_arc_angle = 2
near_threshold = 0.25
mid_threshold = 0.8
far_threshold = 5
start = 0  # rospy.Time.now()
duration = 0.
direction = 0
carry_dir = 0.
carry_speed = 0
carry_angle = 0
# near_threshold = 2  # 0.3
# mid_threshold = 4
# far_threshold = 5
direct = 1


def avoid(steer):
    global angle
    global speed

    angle = steer.data  #changeAngle(, 0, 0.4)
    speed = 80  #changeSpeed(speed, max_speed, 10)
    print("It's a free world")
    # rospy.Publisher('robo_angle', Float32, queue_size=10)
    # rospy.Publisher('robo_speed', Float32, queue_size=10)


def main():
    global angle
    global speed
    angle = 0.0
    speed = 0.0
    rospy.init_node('listener', anonymous=True)

    robo_angle_pub = rospy.Publisher('robo_angle', Float32, queue_size=10)
    robo_speed_pub = rospy.Publisher('robo_speed', Float32, queue_size=10)
    # rospy.Subscriber("scan", sensor_msgs.msg.LaserScan, LaserScanProcess)
    # laser_sub = Subscriber("scan", sensor_msgs.msg.LaserScan, avoid)
    rospy.Subscriber("Steering_Angel", Float32, avoid)
    # ats = ApproximateTimeSynchronizer([laser_sub, pilot_sub], queue_size=5, slop=0.1, allow_headerless=True)
    # ats.registerCallback(avoid)
    # rospy.spin()

    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        # speed = 80.0
        msg_angle = angle
        msg_speed = speed
        print("                                s =", speed, "a =", angle)
        robo_angle_pub.publish(msg_angle)
        robo_speed_pub.publish(msg_speed)
        rate.sleep()


if __name__ == '__main__':
    main()

