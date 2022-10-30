#! /usr/bin/env python3

import rospy
import actionlib
import sys

from urr_p_05_actions.msg import FibonacciAction
from urr_p_05_actions.msg import FibonacciGoal


def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('fibonacci', FibonacciAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = FibonacciGoal(order=20)

    # Sends the goal to the action server.
    client.send_goal(goal)
    
    # Goal State
    # PENDING = 0
    # ACTIVE = 1
    # DONE = 2
    # WARN = 3
    # ERROR = 4

    result_state = client.get_state()

    rate = rospy.Rate(1)
    rospy.loginfo("result_state: "  + str(result_state))

    counter = 0
    while result_state < 2:
        rospy.loginfo("doing some other stuff while waiting for result...")
        counter += 1
        rate.sleep()
        result_state = client.get_state()
        rospy.loginfo("result_state: "  + str(result_state))

        
    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        result = fibonacci_client()
        print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)