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
- [x] Create blank ROS packages
- [ ] Each ROS nodes' test succeeded
  - [ ] Core
  - [x] Dribbler
  - [x] HelloWorld
  - [ ] IO
  - [ ] Kicker
  - [ ] VelocityConverter
  - [ ] Joy
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

### ROS Install
Ganbatte!!!

### Libraries Install

```zsh
  $ sudo apt install ros-kinetic-joy
```
  
### FrootsPi Install

```zsh
  $ git clone https://github.com/SSL-Roots/FrootsPi ~/catkin_ws/src/FrootsPi

  $ cd ~/catkin_ws
  $ catkin_make
```

### Raspberry Pi Settings
You have to enable interfaces **i2c**, **spi**.

```zsh
  $ sudo raspi-config
```

### pigpio Install

pigpio is a C/python library for Raspberry Pi control GPIO.

Refere to below link and install pigpio library.

You **must** install pigpio with zip file. Not *apt-get install pigpio*

http://abyz.me.uk/rpi/pigpio/download.html

```zsh
  wget abyz.me.uk/rpi/pigpio/pigpio.zip
  unzip pigpio.zip
  cd PIGPIO
  make
  sudo make install
```

### Joystick settings (Optional)

You can control FrootsPi with joystick.

Refere to below link and setup your joystick.

http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

## How to run FrootsPi

### Run a FrootsPi node

```zsh
  # Start the pigpio daemon
  $ sudo pigpiod 

  $ roscore

  # In other terminal
  $ rosrun frootspi_dribbler dribbler_node.py
  
  ...

  # Stop the pigpio daemon
  $ sudo killall pigpiod
```


### Launch FrootsPi nodes

```zsh
  
  # Start the pigpio daemon
  $ sudo pigpiod 

  # Launch nodes
  $ roslaunch frootspi_core frootspi.launch

  # or Launch node with joystick controller
  $ roslaunch frootspi_core frootspi.launch joy:=true

  # Stop the pigpio daemon
  $ sudo killall pigpiod
```

## How to write test code

### Node test

1. Create **tests** directory to a package tree
  
Example: *frootspi_hello_world*
```zsh
  .
  ├── CMakeLists.txt
  ├── package.xml
  ├── scripts
  └── tests
      └── test_hello_world.py
```

2. Add a test code

Example: *test_hello_world.py*
```python
  #!/usr/bin/env python
  #encoding: utf8

  import rospy, unittest, rostest
  import rosnode
  import time

  class HelloWorldTest(unittest.TestCase):
      def test_node_exist(self):
          nodes = rosnode.get_node_names()
          self.assertIn('/hello_world', nodes, 'node does not exist')


  if __name__ == '__main__':
      time.sleep(3) # テスト対象のノードが立ち上がるのを待つ
      rospy.init_node('test_hello_world.')
      rostest.rosrun('frootspi_hello_world', 'test_hello_world', HelloWorldTest)
```
- This test code checks node existance

3. Write a rostest file

Example: *frootspi_core/tests/test_frootspi.test*
```xml
  <launch>
      <node name="hello_world" pkg="frootspi_hello_world" type="hello_world.py" required="true" />
      <test test-name="test_hello_world" pkg="frootspi_hello_world" type="test_hello_world.py" />

  </launch>
```

- If you create a new rostest file, you have to edit *test_scripts/travis_script.bash* to execute travis_ci test

4. Run the rostest

```zsh
  $ rostest frootspi_core test_frootspi.test
```

### Library test

1. Create **tests** directory to a scripts directory

Example: *frootspi_hello_world*
```zsh
  .
  ├── CMakeLists.txt
  ├── package.xml
  ├── scripts
  │   ├── hello_world.py
  │   ├── tests
  │   │   └── test_hello.py
  │   └── util
  │       ├── __init__.py
  │       └── hello.py
```

2. Add a test code

Example library: *util/hello.py*

```python
  #encoding: utf8

  def add_hello(text):
      return text + 'hello'

  def enclose_hello(text):
      return 'he' + text + 'llo'


  class Calculator(object):
      def __init__(self):

          self._buffer = None

      def add(self, a, b):
          return a + b

      def set_value(self, value):
          self._buffer = value

      def get_value(self):
          return self._buffer

      def delete_value(self):
          self._init_buffer()
          return True

      def _init_buffer(self):
          self._buffer = None
```

Example library test code: *tests/test_hello.py*

```python
  #!/usr/bin/env python
  #encoding: utf8

  import sys, os
  import unittest

  pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
  sys.path.append(pardir)

  import util.hello as hello

  class TestHello(unittest.TestCase):

      def setUp(self):
          self._calculator = hello.Calculator()


      def test_add_hello(self):
          expected = "abcdehello"
          result = hello.add_hello("abcde")
          self.assertEqual(expected, result)


      def test_enclose_hello(self):
          expected = "heabcdello"
          result = hello.enclose_hello("abcde")
          self.assertEqual(expected, result)


      def test_add(self):
          expected = 3
          result = self._calculator.add(1, 2)
          self.assertEqual(expected, result)

          expected = 2.5
          result = self._calculator.add(1.7, 0.8)
          self.assertAlmostEqual(expected, result)


      def test_set_value(self):
          value = 4
          self._calculator.set_value(value)

          result = self._calculator.get_value()
          self.assertEqual(value, result)


      def test_delete_value(self):
          self._calculator.set_value(2.4)
          self._calculator.delete_value()

          expected = None
          result = self._calculator.get_value()
          self.assertEqual(expected, result)


  if __name__ == "__main__":
      import rosunit
      rosunit.unitrun('frootspi_hello_world', 'test_hello', TestHello)
```

3. Edit a CMakeLists.txt

Example: *frootspi_hello_world/CMakeLists.txt*

```txt
  ...
  catkin_add_nosetests(scripts/tests/test_hello.py)
```

4. Run the rostest
```zsh
  # Run the all tests
  $ cd ~/catkin_ws
  $ catkin_make run_tests 
  
  # Get result
  # Caution! `catkin_make run_tests` always returns 0
  $ catkin_test_results
  
  # Run arbitary test
  $ cd ~/catkin_ws
  $ catkin_make run_tests_frootspi_hello_world_nosetests_scripts.tests.test_hello.py
```

## References

- [Roots mbed program](https://os.mbed.com/users/alt0710/code/Roots/)


## Author

**Roots** : A RoboCup SSL team on Japan -> [*Roots Home*](https://github.com/SSL-Roots/Roots_home/wiki)
