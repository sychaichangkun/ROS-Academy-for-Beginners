#!/usr/bin/env python
# coding:utf-8

import rospy
from service_client_demo.srv import *

def client_srv():
    rospy.wait_for_service("greetings")
    try:
        greetings_client = rospy.ServiceProxy("greetings",Service_demo)
        resp = greetings_client("HAN",20)
        print "Message From server:%s"%resp.response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__=="__main__":
    client_srv()
