#!/usr/bin/env python
#coding=utf-8
import rospy
#导入mgs到pkg中
from publish_subscribe_demo.msg import add
#回调函数输入的应该是msg
def callback(add):
     rospy.loginfo('Listener GPS: x=%f ,y= %f,distance=%f',add.x,add.y,add.distance)
def listener():
    rospy.init_node('pylistener', anonymous=True)
#Subscriber函数第一个参数是topic的名称，第二个参数是接受的数据类型 第三个参数是回调函数的名称
    rospy.Subscriber('chatter', add, callback)
    rospy.spin()
if __name__ == '__main__':
    listener()

