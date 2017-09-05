#!/usr/bin/env python
# coding:utf-8

from sevice_client_demo.srv import *
import rospy

def client_srv():
    rospy.wait_for_service("greetings")
    try:
        greetings_client = rospy.ServiceProxy("greetings",SERVICE_DEMO)
        resp = greetings_client("HAN",20)
        return resp.response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__=="__main__":
    client_srv()
