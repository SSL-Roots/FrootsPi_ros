[![Build Status](https://travis-ci.org/SSL-Roots/FrootsPi.svg?branch=master)](https://travis-ci.org/SSL-Roots/FrootsPi)

# FrootsPi
ROS packages on Raspberry Pi for RoboCup SSL Robot

Raspberry Pi + Roots -> **F**ruits(Raspberry) **Pi** + **Roots** -> **FrootsPi** :thumbsup:

## Project Status
- [x] Create this repository
- [x] Create hello world code
- [x] Create simple node test code
- [x] Create simple library test code
- [x] Simple travis-ci test succeeded
- [ ] Create blank ROS packages
- [ ] Each ROS nodes' test succeeded
- [ ] Integrated tests succeeded
- [ ] FrootsPi robot power on
- [ ] FrootsPi robot correctly moved! :)

## Requirements
- Device
  - Raspberry Pi 3 model B/B+
- OS
  - Ubuntu 16.04
  - Raspbian (Untested)
- ROS
  - Kinetic
- and SSL Robot parts
  - Our team's mechanic / electric cad data here -> ( )

## Installation
- ROS Install
  - Ganbatte!!!
  
- FrootsPi Install

```zsh
  $ git clone https://github.com/SSL-Roots/FrootsPi ~/catkin_ws/src/FrootsPi

  $ cd ~/catkin_ws
  $ catkin_make
```

## Test

```zsh
  $ cd ~/catkin_ws
  $ catkin_make run_tests
```

## Author

**Roots** : A RoboCup SSL team on Japan -> [*Roots Home*](https://github.com/SSL-Roots/Roots_home/wiki)
