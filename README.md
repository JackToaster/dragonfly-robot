# Dragonfly Robot
This repository contains hardware and firmware files for a flight controller & 4-channel servo driver, intended for my dragonfly ornithopter robot.
The hardware is based on the WCH CH32V203, a low-cost RISC-V microcontroller.
Motor drives use the TI DRV8212 full-bridge driver and INA199 current sense amplifier for current feedback.
## Firmware
The firmware in this repository utilizes CNLohr's [CH32Fun](https://github.com/cnlohr/ch32fun) framework for register mapping and hardware initialization functionality.
