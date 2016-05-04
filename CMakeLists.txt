cmake_minimum_required(VERSION 2.8.3)
project(human_control_arm)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  tf
  sensor_msgs
)



catkin_package(
  INCLUDE_DIRS #include
  LIBRARIES human_control_arm
  CATKIN_DEPENDS joint_state_publisher robot_state_publihser roscpp tf 
  DEPENDS system_lib
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)


 add_executable(human_control_arm src/arm_control.cpp src/arm_control.h)

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )

