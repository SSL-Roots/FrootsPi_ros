#!/usr/bin/env python
#encoding: utf8

import rospy
import pigpio
import time
from frootspi_msg.msg import FrootsCommand

import motor_driver as driver


class Core(object):
    def __init__(self):
        self._GPIO_PWM = 13
        self._GPIO_LOW = 6
        self._GPIO_FAULT = 27

        self._pi = pigpio.pi()
        self._pi.set_mode(self._GPIO_PWM, pigpio.OUTPUT)
        self._pi.set_mode(self._GPIO_LOW, pigpio.OUTPUT)
        self._pi.set_mode(self._GPIO_FAULT, pigpio.INPUT)

        # FAULTを検知したらモータを止める
        self._cb = self._pi.callback(self._GPIO_FAULT, 
                pigpio.FALLING_EDGE, self._callback_fault)

        self._PWM_FREQ = 10000 # 10 kHz
        self._DEAD_TIME = 0.001 # 1 msec

        self._sub_command = rospy.Subscriber('froots_command', FrootsCommand,
                self._callback_command)

    def shutdown(self):
        # roscore停止時に出力をすべて止める
        self._pi.set_mode(self._GPIO_PWM, pigpio.INPUT)
        self._pi.set_mode(self._GPIO_LOW, pigpio.INPUT)


    def _callback_command(self, command):
        fault_flag = self._pi.read(self._GPIO_FAULT)
        can_drive, duty = driver.convert_command(command, fault_flag)

        if can_drive:
            self._drive(duty)
        else:
            self._brake()


    def _drive(self, duty):
        # PWMとLOWが同時にONすると、電源ラインがデッドショートする
        # PWMとLOWの切り替えにデッドタイムを設ける
        self._pi.write(self._GPIO_LOW, pigpio.LOW)
        time.sleep(self._DEAD_TIME)
        self._pi.hardware_PWM(self._GPIO_PWM, self._PWM_FREQ, duty)


    def _brake(self):
        # PWMとLOWが同時にONすると、電源ラインがデッドショートする
        # PWMとLOWの切り替えにデッドタイムを設ける
        self._pi.write(self._GPIO_PWM, pigpio.LOW)
        time.sleep(self._DEAD_TIME)
        self._pi.write(self._GPIO_LOW, pigpio.HIGH)

    
    def _callback_fault(self, gpio, level, tick):
        self._brake()



def main():
    rospy.init_node('dribbler')

    core = Core()

    rospy.on_shutdown(core.shutdown)
    
    rospy.spin()


if __name__ == '__main__':
    main()
