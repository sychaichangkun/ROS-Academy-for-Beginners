#!/usr/bin/env python
# coding:utf-8

# Import necessary modules. Attention that how we import the service files.
# the srv is a folder under the service_client_demo package. 
import rospy
from service_client_demo.srv import *

def server_srv():
    # Init the node with name "greetings_server"
    rospy.init_node("greetings_server")
    # Define the service server with name "greetings" and service type Service_demo
    # and request message will be handled by handle_function.
    s = rospy.Service("greetings", Service_demo, handle_function)
    print "Ready to handle the request:"
    # avoid the server from quiting until the node shutdown
    rospy.spin()

# Define the handle function to handle the request inputs
def handle_function(req):
    # Please look out how we use the request information, the service is like a class 
    # If we want to use the information in it, for example, in our example, the Service_demo.srv
    # file contains two parts, the first one is request part, and the second one is response part
    # if we want to use the request information(name with type string and age with type int64), 
    # we can call them as req.name and req.age
    print "Hi Server, my name is %s and I'm %s years old"%(req.name,req.age)

    # The Service_demoResponse class is instanced automatically by ROS once the service is defined.`
    return Service_demoResponse("Hi %s"%req.name)

if __name__=="__main__":
    server_srv()
