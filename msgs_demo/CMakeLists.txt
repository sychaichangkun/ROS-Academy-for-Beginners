cmake_minimum_required(VERSION 2.8.3)
project(msgs_demo)
find_package(catkin REQUIRED COMPONENTS std_msgs nav_msgs sensor_msgs std_srvs actionlib_msgs message_generation)

add_message_files(DIRECTORY msg
                      FILES
                      Accel.msg
                      Echos.msg
                      Imu.msg
                      LaserScan.msg
                      Odometry.msg
                      Point.msg
                      Pose.msg
                      PoseStamped.msg
                      PoseWithCovariance.msg
                      Power.msg
                      Quaternion.msg
                      Twist.msg
                      TwistWithCovariance.msg
                      Vector3.msg
                 )

add_service_files(DIRECTORY srv
					FILES
					AddTwoInts.srv
					TalkerListener.srv
					SetCameraInfo.srv
					GetPlan.srv
					SetMap.srv
					GetMap.srv
					Empty.srv
					SetBool.srv
					Trigger.srv
				)

add_action_files(DIRECTORY action
                     FILES 
					 AutoDocking.action
					 AddTwoInts.action
					 GetMap.action
					 MoveBase.action
                )

generate_messages(DEPENDENCIES std_msgs actionlib_msgs std_srvs nav_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs nav_msgs sensor_msgs actionlib_msgs std_srvs)



