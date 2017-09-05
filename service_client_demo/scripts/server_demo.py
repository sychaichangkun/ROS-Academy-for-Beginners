#!/usr/bin/env python
# coding:utf-8

import rospy
from service_client_demo.srv import SERVICE_DEMO

def server_srv():
    rospy.init_node("greetings_server")
    s = rospy.Service("greetings", SERVICE_DEMO, handle_function)
    print "Ready to handle the request:"
    rospy.spin()

def handle_function(req):
    print "Hi Server, my name is %s and I'm %s years old"%(req.name,req.age)
    return SERVICE_DEMOResponse("Hi %s"%req.name)

if __name__=="__main__":
    server_srv()
