#!/usr/bin/env python  
# -*- coding:utf-8 -*-  
import roslib  
roslib.load_manifest('learning_tf')  
import rospy  
import math  
import tf   
  
if __name__ == '__main__':  
    rospy.init_node('py_tf_turtle')  
  
    listener = tf.TransformListener() #TransformListener创建后就开始接受tf广播信息，最多可以缓存10s  目前存在的问题，是四个数值的顺序我还有点问题
    rate = rospy.Rate(10.0)  
    while not rospy.is_shutdown():  
        try:  
            (trans,rot) = listener.lookupTransform('/base_link', '/link1', rospy.Time(0))  
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):  
            continue  
        
        rospy.loginfo('距离原点的位置: x=%f ,y= %f，z=%f \n 旋转四元数: w=%f ,x= %f，y=%f z=%f ',trans[0],trans[1],trans[2],rot[0],rot[1],rot[2],rot[3])
  
        rate.sleep()  
