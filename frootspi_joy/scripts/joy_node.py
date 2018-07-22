#!/usr/bin/env python
#encoding: utf8

import rospy
from sensor_msgs.msg import Joy
from frootspi_msg.msg import FrootsCommand


class Core(object):
    def __init__(self):
        self._sub_joy = rospy.Subscriber('joy', Joy, self._callback_joy)

        self._pub_command = rospy.Publisher('froots_command', FrootsCommand, queue_size=1)

        self._A = rospy.get_param('~button_A')
        self._B = rospy.get_param('~button_B')
        self._X = rospy.get_param('~button_X')
        self._Y = rospy.get_param('~button_Y')
        self._L = rospy.get_param('~button_L')
        self._R = rospy.get_param('~button_R')
        self._SEL = rospy.get_param('~button_SEL')
        self._START = rospy.get_param('~button_START')

        self._dribble_power = 6
        self._dribble_MAX = 15


    def _callback_joy(self, msg):
        command = FrootsCommand()

        # Dribble command update
        if msg.buttons[self._Y]:
            self._dribble_power = self._dribble_power_control(
                    msg, self._dribble_power)

            command.dribble_flag = True
            command.dribble_power = self._dribble_power
        else:
            command.dribble_flag = False
            command.dribble_power = 0

        self._pub_command.publish(command)


    def _dribble_power_control(self, joy_msg, dribble_power):
        if joy_msg.buttons[self._X]:
            dribble_power += 1
        elif joy_msg.buttons[self._B]:
            dribble_power -= 1

        if dribble_power > self._dribble_MAX:
            dribble_power = self._dribble_MAX
        elif dribble_power < 0:
            dribble_power = 0
        return dribble_power


    def shutdown(self):
        pass



def main():
    rospy.init_node('joy_node')

    core = Core()

    rospy.on_shutdown(core.shutdown)

    rospy.spin()


if __name__ == '__main__':
    main()

