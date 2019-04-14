# Dance Wheelchair
In this document the functioning of the Internet-connected Wheelchair called Dance Wheelchair will be explained. This internet-connected wheelchair lets wheelchair users dance in the same way that is being done at a gaming arcade hall. Just as the game screen shows different arrow keys for people to step on the dance platform, LED light signals of Dance Wheelchair  guide the user which direction to move, giving her/him cues to choreograph the dance. If the user moves correctly, she/he is notified with brief vibration and gets next dance cue from the LED lights. In this way, people on a wheelchair can enjoy their own DANCE DANCE REVOLUTION!

1. [Components](#1-Components)
2. [Assembling the wheelchair](#2-Assembling-the-Wheelchair)
3. [Libraries](#3-Libraries)
4. [Code](#4-Code)
5. [Wiring](#5-Wiring)
6. [Poster](#6-Poster)


## 1 Components
- Any Wheelchair
- Two Pieces of Thin Ply Wood
- +- 1 Meter of Velcro
- +- 6 Cable Ties
- Tape
- 1 Raspberry Pi
- 1 Adafruit Bluefruit Feather
- 1 Arduino Mega 2560
- 1 Small Powerbank
- 1 Big Powerbank
- +- 10 male to male jumper wires
- 1 Vibration Motor
- 2 Neopixel LEDs
- 2 Condensators
- 2 Resistors of 470Ω

## 2 Assembling the wheelchair
By using two pieces of plywood and applying these to the inside of the wheels, a space is created to add the Raspberry Pi, Arduino Mega and a big powerbank. The feather and a small powerbank are attached to a spoke of the wheel to detect the rotation. Use cable ties and tape to fix these components to the wheelchair.

## 3 Libraries
The following libraries need to be installed in order to successfully run the code. In the code you can find the comments to what the code does and how it works.



#### In python on Raspberry pi: ####
```python
import pygatt  # To access BLE GATT support
import signal  # To catch the Ctrl+C and end the program properly
import os  # To access environment variables
from dotenv import load_dotenv  # To load environment variables from .env file
import serial
import time
import random # to generate random movements

# DCD
from dcd.entities.thing import Thing
from dcd.entities.property_type import PropertyType
```

#### On Feather:

```C
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#include "BluefruitConfig.h"
```

#### On Arduino:
Download Adafruit Neopixel library to control the LEDs.

```C
#include <Adafruit_NeoPixel.h>
```


## 4 Code

### 4.1. Code for Raspberry Pi
This code needs to run on the Raspberry pi:

### 4.2. Code for Adafruit Bluefruit feather

### 4.3. Code for Arduino Mega
This code enabled Arduino Mega to receive signals from Raspberry Pi through its serial port and actuates 2 LED lights and the vibration motor. 



## 5 Wiring

![](images/wheelchair_madness.jpg)

https://www.tinkercad.com/things/7NosIb3B7m8-stunning-blad/editel?tenant=circuits?sharecode=-iT8iiwYHXB-MzWQWOSuqNqyqwceIcqcFFds_0Dsx5k=

## 6 Poster

![](images/Poster-IOT.jpeg)
