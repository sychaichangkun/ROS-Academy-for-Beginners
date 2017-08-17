#!/usr/bin/env python
# coding: utf-8
# filename: keyboard_photo_client2.py

import sys
import rospy
from simple_arm.srv import TakePhotoCommand

def keyboard_photo_client2(command):
    rospy.wait_for_service('take_photo')
    try:
        take_photo_client = rospy.ServiceProxy('take_photo',TakePhotoCommand)
        resp = take_photo_client(command)
        print resp.feedback

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    command = raw_input('Please input command:')
    keyboard_photo_client2(command)
