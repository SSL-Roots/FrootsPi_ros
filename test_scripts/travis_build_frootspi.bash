#!/bin/bash -xve

# Install packages
sudo apt -y install ros-indigo-joy

# Install pigpio
wget abyz.me.uk/rpi/pigpio/pigpio.zip
unzip pigpio.zip
cd PIGPIO
make
sudo make install
cd ../

# Build FrootsPi
rsync -av ./ ~/catkin_ws/src/FrootsPi

# cd ~/catkin_ws/src/FrootsPi
# git submodule init
# git submodule update

cd ~/catkin_ws
catkin_make
