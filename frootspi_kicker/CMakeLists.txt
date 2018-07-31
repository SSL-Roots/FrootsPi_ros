cmake_minimum_required(VERSION 2.8.3)
project(frootspi_kicker)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  frootspi_msg
)

catkin_package(
 CATKIN_DEPENDS rospy std_msgs frootspi_msg
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

