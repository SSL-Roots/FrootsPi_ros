#!/usr/bin/env python
#encoding: utf8

import sys, os
import unittest
import math
from frootspi_msg.msg import FrootsCommand

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)

import calc_motor_order as order

class TestOrder(unittest.TestCase):
    def setUp(self):
        pass

    
    def test_phasor2vector(self):
        command = FrootsCommand()
        command.vel_norm = 1
        command.vel_theta = math.radians(90)

        vel_x, vel_y = order.convert_phasor2vector(command)

        expected_y = 1.0
        expected_x = 0.0

        self.assertAlmostEqual(expected_x, vel_x)
        self.assertAlmostEqual(expected_y, vel_y)


    def test_vector_robot2wheel(self):
        vel_x = 1
        vel_y = 1
        vel_omega = 1

        vel_wheel0, vel_wheel1, vel_wheel2 = order.convert_vector_robot2wheel(
                vel_x, vel_y, vel_omega)

        expected_0 = -0.5 * vel_x +  0.8660254 * vel_y + 70 * vel_omega
        expected_1 = -0.5 * vel_x + -0.8660254 * vel_y + 70 * vel_omega
        expected_2 =  1   * vel_x +  0         * vel_y + 70 * vel_omega   

        self.assertAlmostEqual(expected_0, vel_wheel0)
        self.assertAlmostEqual(expected_1, vel_wheel1)
        self.assertAlmostEqual(expected_2, vel_wheel2)
        

    def test_velocity2omega(self):
        vel_wheel0 = 1
        vel_wheel1 = 1
        vel_wheel2 = 1

        omega_wheel0, omega_wheel1, omega_wheel2 = order.convert_velocity2omega(
                vel_wheel0, vel_wheel1, vel_wheel2)

        expected_0 = 1 * 2.83 / 26
        expected_1 = 1 * 2.83 / 26
        expected_2 = 1 * 2.83 / 26

        self.assertAlmostEqual(expected_0, omega_wheel0)
        self.assertAlmostEqual(expected_1, omega_wheel1)
        self.assertAlmostEqual(expected_2, omega_wheel2)


    def test_convert_lsb(self):
        omega_wheel0 = 1
        omega_wheel1 = 1
        omega_wheel2 = 1

        omega_wheel0_int, omega_wheel1_int, omega_wheel2_int = order.convert_lsb(
                omega_wheel0, omega_wheel1, omega_wheel2)

        expected_0 = 10
        expected_1 = 10
        expected_2 = 10

        self.assertAlmostEqual(expected_0, omega_wheel0_int)
        self.assertAlmostEqual(expected_1, omega_wheel1_int)
        self.assertAlmostEqual(expected_2, omega_wheel2_int)
        

    def test_convert_can_data(self):
        order_omega = 1

        order_Lside, order_Hside = order.convert_can_data(order_omega)

        expected_Lside = 1
        expected_Hside = 0

        self.assertAlmostEqual(expected_Lside, order_Lside)
        self.assertAlmostEqual(expected_Hside, order_Hside)
    

if __name__ == "__main__":
    import rosunit
    rosunit.unitrun('frootspi_velocity_converter', 'test_order', TestOrder)
