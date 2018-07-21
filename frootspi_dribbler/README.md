# FrootsPi Dribbler

Dribble motor controller

## Pin Assignments

|Pin|GPIO|Function|
|:---:|:---:|:---|
|33|GPIO 13|PWM1 (DRIBBLE PWM HIGH SIDE)|
|31|GPIO 6 |DIGITAL OUT (DRIBBLE LOW SIDE)|
|13|GPIO 27|DIGITAL IN (DRIBBLE FAULT)|

## Pub & Sub

### Publish

None

### Subscribe

- FrootsCommand *froots_command* 
  - bool *dribble_flag*
  - int *dribble_power*
  

## Function

FrootsPi Dribblerはドリブルモータが接続されたハーフブリッジ回路を制御します。
DRIBBLE PWM HIGH SIDEがブリッジのHi側、DRIBBLE LOW SIDEがブリッジのLow側に接続されています。
DRIBBLE PWM HIGH SIDEがPWM信号を出力することで、ドリブルモータが回転します。
ドリブルモータを止めるときは、DRIBBLE PWM HIGH SIDEを停止し、DRIBBLE LOW SIDEをONしてください。

froots_commandのdribble_flagがTrueのとき、モータを回してください。
Falseのとき、モータを止めてください。

froots_commandのdribble_powerはDRIBBLE PWM HIGH SIDEのDutyです。
dribble_power=0でDuty=0%、dribble_power=15でDuty=100%となるように変換してください。

ドリブルモータから過電流が検出されるとDRIBBLE FAULTがLOWになります。
安全のためモータを停止してください。

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
- HIGH SIDEとLOW SIDEを同時にONすると電源ラインがデッドショートします。
HIGH SIDEとLOW SIDEの切り替えに1 msecのデッドタイムを設けてください。
