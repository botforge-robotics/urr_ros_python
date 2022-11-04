#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

bridge = CvBridge()


def get_contour_center(contour):
    M = cv2.moments(contour)
    cx = -1
    cy = -1
    if (M['m00'] != 0):
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    return cx, cy


def image_callback(ros_image):
    global bridge
    # convert ros_image into an opencv-compatible image
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    # from now on, you can work exactly like with opencv

    # convert the image into the HSV color space
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv image", hsv)

    # find the upper and lower bounds of the red color (cricket ball)
    redLower = (0, 221, 56)
    redUpper = (0, 255, 255)
    # define a mask using the lower and upper bounds of the red color
    mask = cv2.inRange(hsv, redLower, redUpper)
    cv2.imshow("mask image", mask)

    # contours detection
    contours, hierarchy = cv2.findContours(
        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    cv2.drawContours(cv_image, contours, -1, (255, 0, 0), 2)
    cv2.imshow("Image window", cv_image)

    # min and max area of ball 200 and 12000
    min_area_ball = 200
    max_area_ball = 11000
    ball_area = 0
    ball_center = (0, 0)
    for c in contours:
        area = cv2.contourArea(c)
        cx, cy = get_contour_center(c)
        # print("Area: {}".format(area))
        if (area > min_area_ball):
            ball_center = (cx, cy)
            ball_area = area
    # print("number of contours: {}".format(len(contours)))

    # if ball detected
    if (ball_area > min_area_ball):
        print("ball center x:{}, y:{}, and area is {}.".format(
            ball_center[0], ball_center[1], ball_area))
    cv2.waitKey(3)


def main(args):
    rospy.init_node('image_converter', anonymous=True)
    image_sub = rospy.Subscriber(
        "/ninjabot/camera/image_raw", Image, image_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
