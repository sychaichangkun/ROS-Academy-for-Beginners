#!/usr/bin/env python
# coding: utf-8

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import rospy
from simple_arm.srv import TakePhotoCommand

def img_subscriber(srv_command):
    if srv_command == "c":
        img_topic = "/rgb_camera/image_raw"
        img_sub = rospy.Subscriber(img_topic, Image, callback_topic)
    return img_sub

def callback_topic(img_msg):
    try:
        plt.imshow(img_msg.data)
        #cv2.imshow(image_msg.data)
       # bridge = CvBridge()
       # cv_image = bridge.imgmsg_to_cv2(img_msg,'bgr8')
       # cv2.imshow('photo_taken',cv_image)
    except CvBridgeError as e:
        print(e)

    return cv_image

def handle_function(req):
    srv_command = req.command
    if srv_command == "c":
        img_capture = img_subscriber(srv_command)
        #plt.imshow(img_capture)
    return TakePhotoCommandResponse("Photo taken configuration!")

def take_photo_server():
    rospy.init_node("take_photo_server")
    s = rospy.Service('take_photo', TakePhotoCommand, handle_function)
    rospy.spin()

if __name__ == "__main__":
    take_photo_server()
