cmake_minimum_required(VERSION 3.0.2)
project(square_vision)

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

 add_message_files(
   FILES
   Square.msg
   Destination.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES square_vision
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy std_msgs message_runtime message_generation
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCv_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/squareImg.cpp src/square_vision_node.cpp)

 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )
