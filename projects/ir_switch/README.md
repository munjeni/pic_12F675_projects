
Overview
--------------------------------------------
* Name: IR Switch
* Description: A program to read an IR transistor via digital input and then power
on or power off something e.g. an PC machine.
* Author: munjeni.

Table of contents
---------------------------

  * [Overview](#overview)
  * [Features](#features)


Features
----------------------

In this folder the source code in c can be found in ir_switch.c.


The IR transistor is read via an digital input on GPIO4, ir protocol is SAMSUNG, remote control can be done using an Samsung tv remote
and button REC for controling switch which is an 2n2222a transistor.

GPIO function

1. 2n2222a base  GP0
2. UART_TX       GP1
3. IR            GP4


![PIC](https://github.com/munjeni/pic_12F675_projects/blob/master/images/ir_switch.png)

