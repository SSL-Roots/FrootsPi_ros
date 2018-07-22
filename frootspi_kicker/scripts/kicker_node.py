#!/usr/bin/env python
#encoding: utf8

import rospy
import pigpio
import time
from frootspi_msg.msg import FrootsCommand

class Core(object):
    def __init__(self):
        self._GPIO_STRAIGHT = 7

        self._pi = pigpio.pi()
        self._pi.set_mode(self._GPIO_STRAIGHT, pigpio.OUTPUT)

        self._sub_command = rospy.Subscriber('froots_command', FrootsCommand,
                self._callback_command)


    def _callback_command(self, command):

        if command.kick_flag:
            self._pi.write(self._GPIO_STRAIGHT, 1)
            # 0.025 ~ 0.04
            time.sleep(0.04)
            self._pi.write(self._GPIO_STRAIGHT, 0)


    def shutdown(self):
        # roscore停止時に出力をすべて止める
        self._pi.set_mode(self._GPIO_STRAIGHT, pigpio.INPUT)


def main():
    rospy.init_node('kicker')

    core = Core()

    rospy.on_shutdown(core.shutdown)

    rospy.spin()


if __name__ == '__main__':
    main()
