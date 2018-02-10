  #! /usr/bin/env python

  import roslib
  roslib.load_manifest('my_pkg_name')
  import rospy
  import actionlib

  from action_demo.msg import DoDishesAction

  class DoDishesServer:
    def __init__(self):
      self.server = actionlib.SimpleActionServer('dishes', DoDishesAction, self.execute, False)
      self.server.start()

    def execute(self, goal):
      # Do lots of awesome groundbreaking robot stuff here
      self.server.set_succeeded()


  if __name__ == '__main__':
    rospy.init_node('dishes_server')
   server = DoDishesServer()
   rospy.spin()
