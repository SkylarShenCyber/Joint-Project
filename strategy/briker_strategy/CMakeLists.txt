cmake_minimum_required(VERSION 3.0.2)
project(briker_strategy)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  square_vision
)

add_message_files(
   FILES
   WalkingParam.msg
   )

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
    )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES briker
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/briker.cpp src/briker_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )

