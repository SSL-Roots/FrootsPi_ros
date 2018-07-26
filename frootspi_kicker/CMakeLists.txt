cmake_minimum_required(VERSION 2.8.3)
project(frootspi_kicker)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  roscpp
  frootspi_msg
)

catkin_package(
 CATKIN_DEPENDS rospy std_msgs roscpp frootspi_msg
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/kicker_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  pigpiod_if2
)
