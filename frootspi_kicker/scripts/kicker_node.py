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
        self._STRAIGHT_KICK_POWER = rospy.get_param('~straight_kick_power')
        self._CHIP_KICK_POWER = rospy.get_param('~chip_kick_power')
        self._KICK_POWER_MAX = 15

        self._kicking = False

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
        # キックし続けないようにkickingフラグで管理する
        if command.kick_flag and \
                self._pi.read(self._GPIO_CHARGE_DONE) == pigpio.LOW and \
                self._pi.read(self._GPIO_BALL_SENSOR) == pigpio.LOW:

            if self._kicking is False:
                target, output_time = self._convert_kick_power(command)

                self._pi.write(target, pigpio.HIGH)
                time.sleep(output_time)
                self._pi.write(target, pigpio.LOW)
                # waves = []
                # waves.append(pigpio.pulse(1<<target, 0, output_time))
                # waves.append(pigpio.pulse(0, 1<<target, 100000)) # 100 msec
                # self._pi.wave_clear()
                #
                # self._pi.wave_add_generic(waves)
                # wid = self._pi.wave_create()
                # self._pi.wave_send_once(wid)

            self._kicking = True
        else:
            self._kicking = False


    def _convert_kick_power(self, command):
        kick_power = command.kick_power
        if kick_power > self._KICK_POWER_MAX:
            kick_power = self._KICK_POWER_MAX
        if kick_power < 0:
            kick_power = 0

        target = self._GPIO_STRAIGHT
        output_time = self._STRAIGHT_KICK_POWER[kick_power]

        if command.chip_enable:
            target = self._GPIO_CHIP
            output_time = self._CHIP_KICK_POWER[kick_power]

        # pigpioのdelay timeは u secオーダー
        # output_time *= 1000000

        return target, output_time


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
