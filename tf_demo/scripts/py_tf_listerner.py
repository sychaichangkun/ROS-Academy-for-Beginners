#!/usr/bin/env python  
# -*- coding:utf-8 -*-  

import rospy  
import math  
import tf   
  
if __name__ == '__main__':  
    rospy.init_node('py_tf_turtle')
    listener = tf.TransformListener() #TransformListener创建后就开始接受tf广播信息，最多可以缓存10s  目前存在的问题，是四个数值的顺序我还有点问题
    rate = rospy.Rate(1.0)  
    #1. 阻塞直到frame相通
    print '1. 阻塞直到frame相通'  
    listener.waitForTransform("/base_link", "/link1", rospy.Time(), rospy.Duration(4.0))
    while not rospy.is_shutdown():  
        try:  
       #2. 监听对应的tf,返回平移和旋转
            print '2. 监听对应的tf,返回平移和旋转'  
            (trans,rot) = listener.lookupTransform('/base_link', '/link1', rospy.Time(0)) #rospy.Time(0)不表示0时刻的tf，而是指最近一帧tf 
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):  
            continue  
        
        rospy.loginfo('距离原点的位置: x=%f ,y= %f，z=%f \n 旋转四元数: w=%f ,x= %f，y=%f z=%f ',trans[0],trans[1],trans[2],rot[0],rot[1],rot[2],rot[3])
        #3. 判断两个frame是否相通
        print '3. 判断两个frame是否相通'
        if listener.canTransform('/link1','/base_link',rospy.Time(0)) :
            print 'true'
        else :
            print 'false'
        rate.sleep()  
