#!/usr/bin/env python
# coding:utf-8

import rospy

def param_set():
    rospy.init_node("param_set")
    rospy.set_param("/Param_demo",20)

if __name__=="__main__":
    param_set()
