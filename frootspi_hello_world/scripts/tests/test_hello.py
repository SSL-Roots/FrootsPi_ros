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

