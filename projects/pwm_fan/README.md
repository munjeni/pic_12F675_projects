
Overview
--------------------------------------------
* Name: PWM FAN (PWM fan controler)
* Description: A program to read an LM35 via analog input and then set
an FAN speed proportionaly by PWM increase, with a PIC12F675.
* Author: munjeni.

Table of contents
---------------------------

  * [Overview](#overview)
  * [Features](#features)


Features
----------------------

In this folder the source code in c can be found in pwm_fan.c.


The temperature is read via an analog input on GPIO4, based
on value the FAN speed is set. i.e PWM = 0 (FAN OFF) , PWM > 0 (FAN speed is increased by PWM value increase)

GPIO function

1. PWM       GP0
2. UART_TX   GP1
3. LM35      GP4


![PIC](https://github.com/munjeni/pic_12F675_projects/blob/master/images/pwm_fan.png)

