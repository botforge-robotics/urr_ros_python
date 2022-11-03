#!/usr/bin/env python3

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

bridge = CvBridge()


def image_callback(ros_image):
    print('got an image')
    global bridge
    # convert ros_image into an opencv-compatible image
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    # from now on, you can work exactly like with opencv
    cv2.imshow("Image window", cv_image)

    # convert the image into the HSV color space
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv image", hsv)

    # find the upper and lower bounds of the red color (cricket ball)
    redLower = (0, 221, 56)
    redUpper = (0, 255, 255)
    # define a mask using the lower and upper bounds of the red color
    mask = cv2.inRange(hsv, redLower, redUpper)
    cv2.imshow("mask image", mask)

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
