#encoding: utf8

from frootspi_msg.msg import FrootsCommand
from math import cos
from math import sin
from math import pi

def convert_phasor2vector(command):
    vel_X = command.vel_norm * cos(command.vel_theta)
    vel_Y = command.vel_norm * sin(command.vel_theta)

    return vel_X, vel_Y

def convert_vector_robot2wheel(vel_X, vel_Y, vel_Omega):
    _radius_robot = 0.07 #[m]

    _coff_Vel0_Vx = -0.5
    _coff_Vel0_Vy = 0.8660254
    _coff_Vel1_Vx = -0.5
    _coff_Vel1_Vy = -0.8660254
    _coff_Vel2_Vx = 1
    _coff_Vel2_Vy = 0

    vel_Wheel0 = _coff_Vel0_Vx * vel_X + _coff_Vel0_Vy * vel_Y + _radius_robot * vel_Omega
    vel_Wheel1 = _coff_Vel1_Vx * vel_X + _coff_Vel1_Vy * vel_Y + _radius_robot * vel_Omega
    vel_Wheel2 = _coff_Vel2_Vx * vel_X + _coff_Vel2_Vy * vel_Y + _radius_robot * vel_Omega

    return vel_Wheel0, vel_Wheel1, vel_Wheel2

def convert_velocity2omega(vel_Wheel0, vel_Wheel1, vel_Wheel2):
    _ratio_gear   = 2.83
    _radius_wheel = 26 #[mm]
    
    omega_wheel0 = _ratio_gear / _radius_wheel * vel_Wheel0
    omega_wheel1 = _ratio_gear / _radius_wheel * vel_Wheel1
    omega_wheel2 = _ratio_gear / _radius_wheel * vel_Wheel2

    return omega_wheel0, omega_wheel1, omega_wheel2

def convert_lsb(omega_wheel0, omega_wheel1, omega_wheel2):
    _lsb = 10

    omega_wheel0_int = int(omega_wheel0 * _lsb)
    omega_wheel1_int = int(omega_wheel1 * _lsb)
    omega_wheel2_int = int(omega_wheel2 * _lsb)

    return omega_wheel0_int, omega_wheel1_int, omega_wheel2_int
    
def convert_can_data(order_omega):
    order_omega_int = int(round(order_omega))
    order_Lside = 0x0000FF & order_omega_int
    order_Hside = (0x00FF00 & order_omega_int) >> 8
    return order_Lside, order_Hside

