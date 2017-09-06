#!/usr/bin/env python
# coding:utf-8

# Import the modules that we need
import rospy
from service_client_demo.srv import *

def client_srv():

    # Wait the service named "greetings" to be available
    rospy.wait_for_service("greetings")
    try:
        # Define the service client with service name "greetings" and service type Service_demo
        greetings_client = rospy.ServiceProxy("greetings",Service_demo)

        # Request values name "HAN" and age 20 
        resp = greetings_client("HAN",20)
        print "Message From server:%s"%resp.response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# if run this file alone, call the function client_srv()
if __name__=="__main__":
    client_srv()
