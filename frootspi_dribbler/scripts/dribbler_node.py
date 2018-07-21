#!/usr/bin/env python
#encoding: utf8

import rospy
import pigpio
from frootspi_msg.msg import FrootsCommand


class Core(object):
    def __init__(self):
        self._gpio_pwm = 13
        self._gpio_low = 6
        self._gpio_fault = 27

        self._pi = pigpio.pi()
        self._pi.set_mode(self._gpio_pwm, pigpio.OUTPUT)
        self._pi.set_mode(self._gpio_low, pigpio.OUTPUT)

        self._sub_command = rospy.Subscriber('froots_command', FrootsCommand,
                self._callback_command)


    def _callback_command(self, command):
        self._pi.write(self._gpio_pwm, command.dribble_flag)


def main():
    rospy.init_node('dribbler')

    core = Core()
    
    rospy.spin()


if __name__ == '__main__':
    main()
