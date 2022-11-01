#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


def callback(msg):
    # print(msg) # this will print the whole odometry message

    # print(msg.header) #This will print the header section of odometry message

    # print(msg.pose) #This will print the pose section of odometry message

    # convert quaternion to euler
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y,
                        orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    print("x:{}, y:{}, Yaw:{}".format(x, y, yaw))


rospy.init_node("odom_sub_node")
sub = rospy.Subscriber('/odom', Odometry, callback)
rospy.spin()
