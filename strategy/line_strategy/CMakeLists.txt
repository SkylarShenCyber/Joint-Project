cmake_minimum_required(VERSION 3.0.2)
project(line_strategy)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
  geometry_msgs
  line_vision
  briker_strategy
)

find_package(OpenCV REQUIRED)
catkin_package(
  #INCLUDE_DIRS include ${vision_INCLUDE_DIRS} 
  #LIBRARIES ${PROJECT_NAME} ${vision_LIBRARIES}
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${OpenCv_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/line_strategy_node.cpp src/strategy.cpp src/lineStrategy)

add_dependencies(
 ${PROJECT_NAME}_node 
${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(
   ${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )
