#!/usr/bin/env python  
# -*- coding:utf-8 -*-  

import rospy  
import math  
import tf   
  
if __name__ == '__main__':  
    rospy.init_node('py_tf_broadcaster')
    print '讲解tf.transformBroadcaster类'
    print '第1种发布方式：sendTransform(translation,rotation,time,child,parent)'
#第一部分，发布sendTransform(translation,rotation,time,child,parent)
    br = tf.TransformBroadcaster()
#输入相对原点的值和欧拉角
    x=1.0 
    y=2.0
    z=3.0  
    roll=0 
    pitch=0
    yaw=1.57 
    rate = rospy.Rate(1)
    while not rospy.is_shutdown(): 
        yaw=yaw+0.1   
        br.sendTransform((x,y,z),  
                     tf.transformations.quaternion_from_euler(roll,pitch,yaw),  
                     rospy.Time.now(),  
                     "base_link",  
                     "link1")  #发布base_link到link1的平移和翻转   
        rate.sleep()  
