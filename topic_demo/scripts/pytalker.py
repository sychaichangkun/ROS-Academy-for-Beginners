#!/usr/bin/env python
#coding=utf-8
import rospy
#倒入自定义的数据类型
from topic_demo.msg import gps

def talker():
    #Publisher 函数第一个参数是话题名称，第二个参数 数据类型，现在就是我们定义的msg 最后一个是缓冲区的大小
    #queue_size: None（不建议）  #这将设置为阻塞式同步收发模式！
    #queue_size: 0（不建议）#这将设置为无限缓冲区模式，很危险！
    #queue_size: 10 or more  #一般情况下，设为10 。queue_size太大了会导致数据延迟不同步。
    pub = rospy.Publisher('gps_info', gps , queue_size=10)
    rospy.init_node('pytalker', anonymous=True)
    #更新频率是1hz
    rate = rospy.Rate(1) 
    x=1.0
    y=2.0
    state='working'
    while not rospy.is_shutdown():
        #计算距离
        rospy.loginfo('Talker: GPS: x=%f ,y= %f',x,y)
        pub.publish(gps(state,x,y))
        x=1.03*x
        y=1.01*y
        rate.sleep()

if __name__ == '__main__':
    talker()
 
