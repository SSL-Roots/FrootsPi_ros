#!/usr/bin/env python
#encoding: utf8

import rospy
import pigpio
import time
import can
from frootspi_msg.msg import FrootsCommand
from calc_motor_order import convert_phasor2vector
from calc_motor_order import convert_vector_robot2wheel
from calc_motor_order import convert_velocity2omega
from calc_motor_order import convert_lsb
from calc_motor_order import convert_can_data

class Core(object):
    def __init__(self):
        self._GPIO_IRQ_CAN = 25

        self._pi = pigpio.pi()
        self._pi.set_mode(self._GPIO_IR_CAN, pigpio.INPUT)
        self._bus = can.interface.Bus(channel = 'can0', bustype='socketcan', bitrate=500000, canfilters=None)
        self._sub_command = rospy.Subscriber('froots_command', FrootsCommand, self._callback_command)

    def shutdown(self):

    def _callback_command(self, command):
        vel_norm  = command.vel_norm, vel_theta = command.vel_theta, vel_omega = command.vel_omega

        vel_x       , vel_y                      = convert_phasor2vector(vel_norm, vel_theta)
        vel_wheel0  , vel_wheel1  , vel_wheel2   = convert_vector_robot2wheel(vel_x, vel_y, vel_omega)
        omega_wheel0, omega_wheel1, omega_wheel2 = convert_velocity2omega(vel_wheel0, vel_wheel1, vel_wheel2)
        order_wheel0, order_wheel1, order_wheel2 = convert_lsb(omega_wheel0, omega_wheel1, omega_wheel2)
        
        self._drive(order_wheel0, order_wheel1, order_wheel2)

    def _drive(self, order_wheel0, order_wheel1, order_wheel2):
        order_wheel0_l, order_wheel0_h = convert_can_data(order_wheel0)
        order_wheel1_l, order_wheel1_h = convert_can_data(order_wheel1)
        order_wheel2_l, order_wheel2_h = convert_can_data(order_wheel2)

        msg_order = can.Message(arbitration_id=0x1AA, data=[order_wheel0_l, order_wheel0_h, order_wheel1_l, order_wheel1_h, order_wheel2_l, order_wheel2_h, order_wheel0_l, order_wheel0_h]) 
        try:
            bus.send(msg_order)
        except can.CanError:
            err_can_count++

def main():
    rospy.init_node('velocity_converter')

    core = Core()

    rospy.on_shutdown(core.shutdown)

    rospy.spin()

if __name__ == '__main__':
    main()
