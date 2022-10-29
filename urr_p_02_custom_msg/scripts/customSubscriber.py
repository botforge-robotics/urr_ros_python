#!/usr/bin/env python3

import rospy
from urr_p_02_custom_msg.msg import Person

def callback(data):
    name = data.first_name + " "+ data.last_name
    age = data.age
    rospy.loginfo("Hi! I am {}, age {}.".format(name,age))
    
def subscriber():
    rospy.init_node('customSubscriber', anonymous=False)    #initialize node
    rospy.Subscriber("person_details", Person, callback)    #create subscriber to topic person_details
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    subscriber()