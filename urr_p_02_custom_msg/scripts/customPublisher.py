#!/usr/bin/env python3

import rospy
from urr_p_02_custom_msg.msg import Person

def publish():
    rospy.init_node('customPublisher', anonymous=False)  # initialize node
    pub = rospy.Publisher('person_details', Person, queue_size=10)  #create publisher of topic person_details
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        instructor = Person() #instance of person message type
        instructor.first_name = "chaitanya"
        instructor.last_name = "mandala"
        instructor.age = 27
        rospy.loginfo("published.. %s" % rospy.get_time())
        pub.publish(instructor) #publish
        rate.sleep()

if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass