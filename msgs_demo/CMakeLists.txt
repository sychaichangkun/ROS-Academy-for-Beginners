cmake_minimum_required(VERSION 2.8.3)
project(msgs_demo)
find_package(catkin REQUIRED COMPONENTS std_msgs actionlib_msgs message_generation)

add_message_files(DIRECTORY msg
                      FILES 
                      SensorState.msg
                      DebugSensor.msg
                      DockInfraRed.msg
                      KeyboardInput.msg
                      CloudCamera.msg
                      XbotState.msg
                      Lift.msg
                      Power.msg
                      ImuNine.msg
                      ImuYRP.msg
                      Echos.msg
                      InfraRed.msg
                      NaviState.msg
                 )

add_action_files(DIRECTORY action
                     FILES AutoDocking.action
                )

generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs actionlib_msgs)
