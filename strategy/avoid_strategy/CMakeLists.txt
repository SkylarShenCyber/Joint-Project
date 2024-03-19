cmake_minimum_required(VERSION 3.0.2)
project(avoid_strategy)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mine_vision
  message_generation

)


add_message_files(
   FILES
   WalkingParam.msg
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES avoid
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


 add_executable(${PROJECT_NAME}_node src/avoid_node.cpp src/avoid.cpp)

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )

