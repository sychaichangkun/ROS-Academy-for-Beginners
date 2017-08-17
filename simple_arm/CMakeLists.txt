cmake_minimum_required(VERSION 2.8.3)
project(simple_arm)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
	controller_manager
	gazebo_ros
)


## Generate services in the 'srv' folder
add_service_files(
   FILES
   GoToPosition.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simple_arm
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(simple_mover_c src/simple_mover_c.cpp)
add_dependencies(simple_mover_c ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_mover_c  ${catkin_LIBRARIES})

