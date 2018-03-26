#!/usr/bin/env python
import sys
import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
        rospy.init_node('py_tf_follower')

        listener = tf.TransformListener()
        
        turtle_vel = rospy.Publisher('/mybot_cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    
        rate = rospy.Rate(10.0)
        ctrl_c = False
        
        follower_model_frame = "/mybot_link"
        model_to_be_followed_frame = "/base_footprint"
        
        def shutdownhook():
            # works better than the rospy.is_shut_down()
            global ctrl_c
            print "shutdown time! Stop the robot"
            cmd = geometry_msgs.msg.Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            turtle_vel.publish(cmd)
            ctrl_c = True

        rospy.on_shutdown(shutdownhook)
        
        while not ctrl_c:
            try:
                (trans,rot) = listener.lookupTransform(follower_model_frame, model_to_be_followed_frame, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            angular = 4 * math.atan2(trans[1], trans[0])
            linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            cmd = geometry_msgs.msg.Twist()
            if 2*linear>1:            
            	cmd.linear.x = linear*0.2
            	cmd.angular.z = -angular*0.1
            else:
            	cmd.linear.x =0
            	cmd.angular.z =0
            turtle_vel.publish(cmd)
            rate.sleep()
