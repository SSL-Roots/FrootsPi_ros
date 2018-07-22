# FrootsPi Joy

(Joystick) Gamepad Controller to send FrootsCommand

## Pin Assignments

None

## Pub & Sub

### Publish

- FrootsCommand *froots_command*

### Subscribe

- sensor_msgs/Joy *joy*

## Function

FrootsPi Joyは

## Specification

- DRIBBLE PWM HIGH SIDE
  - 周波数: 10 kHz
  - Duty: 0 ~ 100 % を16段階に分割
- DRIBBLE LOW SIDE
  - OFF : ハーフブリッジのLow側をOFF
  - ON  : ハーフブリッジのHi側をON
- DRIBBLE FAULT
  - LOW : モータ過電流を検知
  - HIGH  : 異常なし

## Caution

None
