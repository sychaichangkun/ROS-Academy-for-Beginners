#!/usr/bin/env python
# coding:utf-8

import rospy

def param_demo():
    rospy.init_node("param_demo")
    while(not rospy.is_shutdown()):
        parameter = rospy.get_param("/Param_demo")
        if not(parameter==init_parameter):
            print "You've changed parameter '/Param_demo' from %s to %s"%(init_parameter, parameter)
            global init_parameter
            init_parameter = parameter


        print "Param_demo=%s"%parameter
        rospy.sleep(1)

if __name__=="__main__":
    init_parameter = rospy.get_param("/Param_demo")
    param_demo()
