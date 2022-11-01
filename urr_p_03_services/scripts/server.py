#!/usr/bin/env python3

from std_srvs.srv import Trigger, TriggerResponse
import rospy


def handle_request(req):
    resp = TriggerResponse()  # variable to store responce data
    # do some sensor data collection
    temperature = 26
    humdidity = 69
    # end data collection
    resp.success = True
    resp.message = "The temp is {}c and humidity is {}%.".format(
        temperature, humdidity)
    return resp


def dht_server():
    rospy.init_node('dht_server')
    s = rospy.Service('dht', Trigger, handle_request)
    print("Ready to collect temp. humid. data!")
    rospy.spin()


if __name__ == "__main__":
    dht_server()
