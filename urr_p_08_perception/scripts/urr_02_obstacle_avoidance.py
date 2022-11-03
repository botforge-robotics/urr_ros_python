#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

move = Twist()


def laser_callback(data):

    # if the distance to an obstacle in front of robot is bigger than 1 mtr move forward
    if (data.ranges[180] > 1.0):
        move.linear.x = 0.1
        move.angular.z = 0.0

    # if the distance to an obstacle in front of robot is smaller than 0.3mtr turn left
    if (data.ranges[180] < 0.3):
        move.linear.x = 0.0
        move.angular.z = 0.2

    # if the distance to an obstacle at left of robot smaller than 0.3 turn right
    if (data.ranges[270] < 0.3):
        move.linear.x = 0.0
        move.angular.z = -0.2

    # if the distance to an obstacle at right of robot smaller than 0.3 turn left
    if (data.ranges[90] < 0.3):
        move.linear.x = 0.0
        move.angular.z = 0.2

    vel_pub.publish(move)


rospy.init_node('Laser_listener', anonymous=False)
laser_sub = rospy.Subscriber("/ninjabot/scan", LaserScan, laser_callback)
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

rospy.spin()
