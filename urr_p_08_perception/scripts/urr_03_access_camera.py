#!/usr/bin/env python3

import rospy
import cv2
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
    cv2.putText(cv_image, 'Ninjabot Cam Streaming..',
                (10, 350), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imshow("Image window", cv_image)
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
