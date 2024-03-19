cmake_minimum_required(VERSION 3.0.2)
project(mine_vision)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
 add_message_files(
  FILES
  Rect.msg   
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES mine_vision
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy std_msgs message_runtime message_generation
  #DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCv_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/mineImg.cpp src/mine_vision_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )
