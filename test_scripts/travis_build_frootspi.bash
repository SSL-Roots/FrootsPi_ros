#!/bin/bash -xve

# Install packages
# sudo apt -y install something ...

# Build FrootsPi
rsync -av ./ ~/catkin_ws/src/FrootsPi

# cd ~/catkin_ws/src/FrootsPi
# git submodule init
# git submodule update

cd ~/catkin_ws
catkin_make
