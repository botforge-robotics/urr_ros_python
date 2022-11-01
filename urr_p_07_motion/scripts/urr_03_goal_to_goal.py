#! /usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from urr_p_07_motion.srv import Goal, GoalResponse
from geometry_msgs.msg import Twist

x = 0
y = 0
yaw = 0

# odom topic subscibe callback


def odomCallback(msg):
    global x, y, yaw
    # convert quaternion to euler
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y,
                        orientation_q.z, orientation_q.w]
    (_roll, _pitch, _yaw) = euler_from_quaternion(orientation_list)

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    yaw = _yaw

# go to goal service request handler


def handle_goal_request(req):
    global x, y, yaw, vel_pub

    velocity_message = Twist()

    while (True):
        # calculate linear distance and velocity
        K_linear = 0.25
        distance = abs(math.sqrt(((req.x - x) ** 2) + ((req.y - y) ** 2)))
        linear_speed = distance * K_linear

        # force reset max velocity to 0.35m/s
        if (linear_speed > 0.35):
            linear_speed = 0.35

        # calculate angular distance and velocity
        K_angular = 0.3
        desired_angle_goal = math.atan2(req.y - y, req.x - x)
        angular_speed = (desired_angle_goal - yaw) * K_angular

        # force reset angular velocity to 0.5rad/sec
        if (angular_speed > 0.5):
            angular_speed = 0.5

        # publish velocity
        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        vel_pub.publish(velocity_message)

        if (distance < 0.05):
            velocity_message.linear.x = 0.0
            velocity_message.angular.z = 0.0
            vel_pub.publish(velocity_message)
            break

    return GoalResponse(success=True)


rospy.init_node("Go_to_Goal_service_node")
odom_sub = rospy.Subscriber('/odom', Odometry, odomCallback)
goal_service = rospy.Service('go_to_goal', Goal, handle_goal_request)
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rospy.loginfo("Go_to_Goal_service_node_running...")
rospy.spin()
