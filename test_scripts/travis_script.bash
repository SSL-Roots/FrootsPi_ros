#!/bin/bash -xve

cd ~/catkin_ws

# Code level tests
catkin_make run_tests # Always returns 0
catkin_test_results   # Output previous test results

# Node level tests
rostest frootspi_core test_frootspi.test
