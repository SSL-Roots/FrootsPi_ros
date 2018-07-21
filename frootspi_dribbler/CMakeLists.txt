cmake_minimum_required(VERSION 2.8.3)
project(frootspi_dribbler)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_add_nosetests(scripts/tests/test_motor_driver.py)
