#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerRequest

def client():
    rospy.wait_for_service('dht')
    try:
        dht_client = rospy.ServiceProxy('dht', Trigger)
        resp = dht_client(TriggerRequest())
        if(resp.success):
            print(resp.message)
        else:
            print("failed to get data")
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    client()