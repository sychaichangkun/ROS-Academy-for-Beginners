#!/usr/bin/env python
# coding:utf-8

import rospy

def param_demo():
    rospy.init_node("param_demo")
    rate = rospy.Rate(1)
    while(not rospy.is_shutdown()):
        #get param
        parameter1 = rospy.get_param("/param1")
        parameter2 = rospy.get_param("/param2", default=222)
        rospy.loginfo('Get param1 = %d', parameter1)
        rospy.loginfo('Get param2 = %d', parameter2)

        #delete param
        rospy.delete_param('/param2')

        #set param
        rospy.set_param('/param2',2)
        
        #check param
        ifparam3 = rospy.has_param('/param3')
        if(ifparam3):
            rospy.loginfo('/param3 exists')
        else:
            rospy.loginfo('/param3 does not exist')

        #get all param names
        params = rospy.get_param_names()
        rospy.loginfo('param list: %s', params)

        rate.sleep()

if __name__=="__main__":
    param_demo()
