#!/usr/bin/env python
#encoding: utf8

import rospy
import pigpio
import time
from frootspi_msg.msg import FrootsCommand

class Core(object):
    def __init__(self):
        self._GPIO_BALL_SENSOR = 22
        self._GPIO_CHIP = 24
        self._GPIO_STRAIGHT = 7
        self._GPIO_CHARGE_ENABLE = 5
        self._GPIO_CHARGE_DONE = 12
        self._GPIO_CHARGING = 26

        self._pi = pigpio.pi()
        self._pi.set_mode(self._GPIO_BALL_SENSOR, pigpio.INPUT)
        self._pi.set_pull_up_down(self._GPIO_BALL_SENSOR, pigpio.PUD_UP)

        self._pi.set_mode(self._GPIO_CHIP, pigpio.OUTPUT)
        self._pi.set_mode(self._GPIO_STRAIGHT, pigpio.OUTPUT)
        self._pi.set_mode(self._GPIO_CHARGE_ENABLE, pigpio.OUTPUT)
        self._pi.set_mode(self._GPIO_CHARGE_DONE, pigpio.INPUT)
        self._pi.set_mode(self._GPIO_CHARGING, pigpio.OUTPUT)

        self._CHARGE_OFF = rospy.get_param('~charge_off')

        self._sub_command = rospy.Subscriber('froots_command', FrootsCommand,
                self._callback_command)

        self._gpio_init()


    def _gpio_init(self):
        self._pi.write(self._GPIO_CHIP, pigpio.LOW)
        self._pi.write(self._GPIO_STRAIGHT, pigpio.LOW)
        self._pi.write(self._GPIO_CHARGING, pigpio.LOW)

        # ノード起動時に充電許可信号をONする
        # ただし、デバッグ時は充電しないように設定
        if self._CHARGE_OFF:
            self._pi.write(self._GPIO_CHARGE_ENABLE, pigpio.LOW)
        else:
            self._pi.write(self._GPIO_CHARGE_ENABLE, pigpio.HIGH)


    def _callback_command(self, command):

        if command.charge_flag:
            self._pi.write(self._GPIO_CHARGING, pigpio.HIGH)
        else:
            self._pi.write(self._GPIO_CHARGING, pigpio.LOW)

        # kick_flagがHiになり、充電完了かつボールセンサが反応した時にキックする
        if command.kick_flag and \
                self._pi.read(self._GPIO_CHARGE_DONE) == pigpio.LOW and \
                self._pi.read(self._GPIO_BALL_SENSOR) == pigpio.LOW:
            target = self._GPIO_STRAIGHT
            if command.chip_enable:
                target = self._GPIO_CHIP

            self._pi.write(target, pigpio.HIGH)
            # 0.025 ~ 0.04
            time.sleep(0.04)
            self._pi.write(target, pigpio.LOW)


    def shutdown(self):
        # roscore停止時に出力をすべて止める
        self._pi.set_mode(self._GPIO_CHIP, pigpio.INPUT)
        self._pi.set_mode(self._GPIO_STRAIGHT, pigpio.INPUT)
        self._pi.set_mode(self._GPIO_CHARGE_ENABLE, pigpio.INPUT)
        self._pi.set_mode(self._GPIO_CHARGING, pigpio.INPUT)


def main():
    rospy.init_node('kicker')

    core = Core()

    rospy.on_shutdown(core.shutdown)

    rospy.spin()


if __name__ == '__main__':
    main()
