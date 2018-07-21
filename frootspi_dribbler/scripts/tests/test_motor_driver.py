#!/usr/bin/env python
#encoding: utf8

import sys, os
import unittest
from frootspi_msg.msg import FrootsCommand

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)

import motor_driver as driver


class TestDriver(unittest.TestCase):

    def setUp(self):
        pass

    def test_convert_command(self):
        self._test(False, 0, 1, 
                False, 0, "Cannot brake motor")
        self._test(True, 0, 1, 
                True, 0, "Cannot drive motor with power 0")
        self._test(True, 15, 1, 
                True, 1000000, "Cannot drive with max power")
        self._test(True, 90, 1, 
                True, 1000000, "Cannot drive with overflow power")
        self._test(True, -90, 1, 
                True, 0, "Cannot drive with underflow power")
        self._test(True, 10, 1, 
                True, 666666.67, "Cannot drive with correct duty")
        self._test(True, 15, 0, 
                False, 0, "Cannot brake moter with fault_flag")

    def _test(self, flag, power, fault_flag, expected_flag, expected_duty, message=""):
        command = FrootsCommand()
        command.dribble_flag = flag
        command.dribble_power = power

        result_flag, result_duty = driver.convert_command(command, fault_flag)
        self.assertEqual(expected_flag, result_flag, message)
        self.assertAlmostEqual(expected_duty, result_duty, 2, message)


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun('frootspi_dribbler', 'test_motor_driver', TestDriver)

