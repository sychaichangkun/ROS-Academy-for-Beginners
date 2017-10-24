#!/usr/bin/env python  
# -*- coding:utf-8 -*-  
import roslib  
roslib.load_manifest('learning_tf')  
import rospy  
import math  
import tf   
  
if __name__ == '__main__':  
    rospy.init_node('py_tf_broadcaster')  
    br = tf.TransformBroadcaster()
    #输入相对原点的值和欧拉角
    x=input("输入原点的x数值：\n") 
    y=input("输入原点的y数值：\n")
    z=input("输入原点的z数值：\n")  
    roll=input("输入原点的roll数值：\n") 
    pitch=input("输入原点的pitch数值：\n")
    yaw=input("输入原点的yaw数值：\n") 
    br.sendTransform((x,y,z),  
                     tf.transformations.quaternion_from_euler(roll,pitch,yaw),  
                     rospy.Time.now(),  
                     "base_link",  
                     "link1")  #发布乌龟的平移和翻转   
