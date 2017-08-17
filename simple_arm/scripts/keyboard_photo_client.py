#!/usr/bin/env python
# coding: utf-8

import rospy
from simple_arm.srv import TakePhotoCommand
from evdev import InputDevice
from select import select

def detectInputKey():
    dev = InputDevice('/dev/input/event4')
    rospy.wait_for_service('take_photo')
    command = ''
    rospy.init_node('keyboard_photo_client')
    #while True:
    try:
        select([dev],[],[])
        for event in dev.read():
            if(event.code ==46 and event.value == 1):
                command = "OK"
                take_photo_client = rospy.ServiceProxy('take_photo',TakePhotoCommand)
                resp = take_photo_client(command)
                print resp.feedback
            else:
                command = 'NO'

    except rospy.ServiceException, e:
        print "Service call failed:%s"%e

    rospy.spin()

if __name__ == "__main__":
    detectInputKey()



