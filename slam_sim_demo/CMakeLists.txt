cmake_minimum_required(VERSION 2.8.3)
project(slam_sim_demo)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
	controller_manager
	gazebo_ros
	slam_karto
	gmapping
	amcl
	hector_mapping
)

catkin_package(
   CATKIN_DEPENDS urdf xacro
)

