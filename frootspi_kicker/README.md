# FrootsPi Kicker

Kick device controller

## Pin Assignments

|Pin|GPIO|Function|
|:---:|:---:|:---|
|15|GPIO 22|DIGITAL IN (BALL SENSOR IN)|
|18|GPIO 24|DIGITAL OUT (CHIP KICK)|
|26|GPIO 7 |DIGITAL OUT (STRAIGHT KICK)|
|29|GPIO 5 |DIGITAL OUT (BOOST CONVERTER CHARGE ENABLE)|
|32|GPIO 12|DIGITAL IN (BOOST CONVERTER CHARGE DONE)|
|37|GPIO 26|DIGITAL OUT (BOOST CONVERTER CHARGING)|

## Pub & Sub

### Publish

None

### Subscribe

- FrootsCommand *groots_command*
  - bool *kick_flag*
  - int32 *kick_power*
  - bool *chip_enable*
  - bool *charge_flag*

## Parameter

- bool *frootspi_kicker/charge_off*

## Function

FrootsPi Kickerはソレノイドが接続された昇圧回路・スイッチ回路を制御します。
キックデバイスの操作手順は以下のとおりです。

1. 昇圧回路に充電許可信号をONにする (BOOST CONVERTER CHARGE ENABLE)
1. 昇圧回路に充電信号をONにする (BOOST CONVERTER CHARGING)
1. 昇圧回路から充電完了信号がLOWになる (BOOST CONVERTER CHARGE DONE)
1. ボールセンサ信号がLOWになる (BALL SENSOR IN)
1. キック威力を計算する
1. キック威力の時間(数 msec)だけキック信号をONにする (STRAIGHT KICK or CHIP KICK)

ノード起動時に充電許可信号をONにしてください。
ただし、rosparameterのCHARGE_OFF がTrueのときは充電許可信号を常にOFFにしてください。（デバッグモード）

froots_commandのcharge_flagがTrueのとき、充電信号をONにしてください。

froots_commandのkick_flagがTrueのとき、STRAIGHT KICKをONにしてください。
ただし、froots_commandのchip_enableがTrueのときは、CHIP KICKをONにしてください。

kick_powerは0~15の16段階のキック威力です。
kick_power=0で 0 sec間キック信号をONに、
kick_power=1で 0.025 sec間キック信号をONに、
kick_power=15で、0.04 sec間キック信号をONにしてください。


## Specification
- BALL SENSOR IN
  - **内部プルアップの設定をすること**
  - LOW : ボールを未検知
  - HIGH : ボールを検知
- CHIP KICK
  - OFF : なにもしない
  - ON : CHIPキックのスイッチをオン。（0.1 sec以上ONにしないこと）
- STRAIGHT KICK
  - OFF : なにもしない
  - ON : STRAIGHTキックのスイッチをオン。（0.1 sec以上ONにしないこと）
- BOOST CONVERTER CHARGE ENABLE
  - OFF : 充電禁止
  - ON : 充電許可
- BOOST CONVERTER CHARGE DONE
  - LOW : 充電完了
  - HIGH : 充電未完了
- BOOST CONVERTER CHARGING
  - OFF : なにもしない
  - ON : 充電する

## Caution

- 26 Pin (STRAIGHT KICK) はSPI0のCE1機能がデフォルトのため、電源ONでHi出力になります。
- STRAIGHT KICK、CHIP KICKを常時ONにしないこと

