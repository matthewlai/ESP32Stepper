# ESP32Stepper

This is the PCB design and firmware for a Trinamic TMC2590-based stepper motor driver with an ESP32 microcontroller.

## Firmware Instructions

The firmware is Arduino based, using the arduino-esp32 port by Espressif.

1. Install the official Arduino IDE from [https://www.arduino.cc/en/software](https://www.arduino.cc/en/software).
2. [Install the ESP32 core](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html) (either stable or development branch, using Board Manager).
3. Install the required libraries (see below) using Arduino library manager.
4. Clone this repository, and open ESP32Stepper/firmware/main/main.ino in the Arduino IDE.
5. In Tools, use the following settings:
    * Board: ESP32 Dev Module
    * Port: serial port for the board
    * All other settings as default (240 MHz CPU, 80 MHz flash, QIO, 4MB flash, default 4MB partition scheme, no PSRAM)

## Required Libraries

1. Adafruit Neopixel 1.10.0x
