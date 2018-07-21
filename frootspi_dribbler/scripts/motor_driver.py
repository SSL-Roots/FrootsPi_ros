#encoding: utf8

from frootspi_msg.msg import FrootsCommand

def convert_command(command, fault_flag):
    MAX_POWER = 15.0

    can_drive = False
    duty = 0

    if command.dribble_flag and fault_flag:
        power = command.dribble_power
        if power > MAX_POWER:
            power = MAX_POWER
        if power < 0:
            power = 0

        # duty 100 % が1000000となるのはpigpioの仕様
        duty = 1000000 * power / MAX_POWER
        can_drive = True

    return can_drive, duty




