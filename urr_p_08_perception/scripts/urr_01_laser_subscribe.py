#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan


def laser_callback(data):
    # print(data.ranges)
    # print(len(data.ranges))
    print(data.ranges[180])  # print front laser beam range
    print(data.ranges[270])  # print left laser beam range
    print(data.ranges[90])  # print right laser beam range


def main():
    rospy.init_node('Laser_listener', anonymous=False)
    rospy.Subscriber("/ninjabot/scan", LaserScan, laser_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    main()
