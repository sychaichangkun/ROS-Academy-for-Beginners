  #! /usr/bin/env python

  import roslib
  roslib.load_manifest('my_pkg_name')
  import rospy
  import actionlib

  from action_demo.msg import DoDishesAction, DoDishesGoal

  if __name__ == '__main__':
      rospy.init_node('dishes_Client')
      client = actionlib.SimpleActionClient('dishes', DoDishesAction)
      client.wait_for_server()

      goal = DoDishesGoal()
      # Fill in the goal here
     client.send_goal(goal)
     client.wait_for_result(rospy.Duration.from_sec(5.0))
