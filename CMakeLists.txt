set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

cmake_minimum_required(VERSION 2.8.3)
project(s8_object_aligner)

find_package(catkin REQUIRED COMPONENTS roscpp s8_common_node s8_msgs s8_motor_controller actionlib actionlib_msgs)
find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES ObjectAlign.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp s8_common_node s8_msgs s8_motor_controller actionlib actionlib_msgs
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(s8_object_aligner_node src/object_aligner_node.cpp)
target_link_libraries(s8_object_aligner_node ${catkin_LIBRARIES})
