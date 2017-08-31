#!/usr/bin/env python
# coding: utf-8

# 整体思路：首先，本例为service示例，从client端接收到request，判断是否是拍照命令
# 'c'，如果是，则从/rgb_camera/image_raw这个相机图片topic中接收一张图片，然后显示
# 出来
import os
from datetime import datetime
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
from simple_arm.srv import *

def handle_function(req):

    # 提取request消息(service的request)
    srv_command = req.command

    # 若request为'c', 则接收并显示图像，实现拍照功能
    if srv_command == "c":
        # 定义图片topic，并注册到其上，接收到消息后调用callback_topic
        img_topic = "/rgb_camera/image_raw"
	img_msg = rospy.wait_for_message(img_topic, Image)
        try:
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(img_msg,'bgr8')
            image_name = datetime.now().strftime('%Y-%m-%d_%H:%M:%S')+".png"
            cv2.imwrite(image_name,cv_image)
            cv2.waitKey(0)

        except CvBridgeError as e:
            print(e)

        return TakePhotoCommandResponse("Photo taken configuration!")
    else:
        return TakePhotoCommandResponse("Please input 'c'")

def take_photo_server():
    rospy.init_node("take_photo_server")
    s = rospy.Service('take_photo', TakePhotoCommand, handle_function)
    rospy.spin()

if __name__ == "__main__":
    take_photo_server()
