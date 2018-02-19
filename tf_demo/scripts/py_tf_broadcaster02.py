#!/usr/bin/env python  
# -*- coding:utf-8 -*-  

import rospy 
import geometry_msgs.msg
import tf2_ros.transform_broadcaster
import math  
import tf   
  
if __name__ == '__main__':  
    rospy.init_node('py_tf_broadcaster')
    print '讲解tf.transformBroadcaster类'
    print '第2种发布方式：sendTransformMessage(transform)'
#第二部分，发布sendTransformMessage(transform)
    m=tf.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = 'base_link'
    t.header.stamp = rospy.Time(0)
    t.child_frame_id = 'link1'
    t.transform.translation.x = 1
    t.transform.translation.y = 2
    t.transform.translation.z = 3
    t.transform.rotation.w=1
    t.transform.rotation.x=0
    t.transform.rotation.y=0
    t.transform.rotation.z=0
#输入相对原点的值和欧拉角
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        m.sendTransformMessage(t)
        rate.sleep()  
