#!/usr/bin/env python
#encoding: utf8

import rospy
from sensor_msgs.msg import Joy
from frootspi_msg.msg import FrootsCommand


class Core(object):
    def __init__(self):
        self._sub_joy = rospy.Subscriber('joy', Joy, self._callback_joy)

        self._pub_command = rospy.Publisher('froots_command', FrootsCommand, queue_size=1)


    def _callback_joy(self, msg):
        command = FrootsCommand()

        if msg.buttons[3]:
            command.dribble_flag = True
            command.dribble_power = 15
        else:
            command.dribble_flag = False
            command.dribble_power = 0

        self._pub_command.publish(command)


    def shutdown(self):
        pass



def main():
    rospy.init_node('joy_node')

    core = Core()

    rospy.on_shutdown(core.shutdown)

    rospy.spin()


if __name__ == '__main__':
    main()

