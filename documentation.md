# Dance Wheelchair
In this document the functioning of the Internet-connected Wheelchair called Dance Wheelchair will be explained. This internet-connected wheelchair lets wheelchair uses dance in the same way that is being done at a gaming arcade hall. LED signals tell the user what to do in a way how to choreograph the dance. If the user moves correctly, he gets input in the form of the LED lights and a vibration. Subsequently, the user gets input on the next move to perform.

<<<<<<< HEAD
![](images/Poster-IOT.jpeg)

### Libraries
The following libraries need to be installed in order to successfully run the code. In the code you can find the comments to what the code does and how it works.
=======
### Components
- Any Wheelchair
- Two pieces of thin ply wood
- +- 1 meter of velcro
- +- 6 tie-wraps
- Tape
- 1 Raspberry Pi
- 1 Adafruit bluefruit feather
- 1 Arduino mega
- 1 Small Powerbank
- 1 Big Powerbank
- 10 male to male jumper wires
- 1 vibration motor
- 2 rgb LED's
- 2 condensators
- 2 resistors of 350kΩ

### Assembling the wheelchair
By using two pieces of plywood and applying these to the inside of the wheels, a space is created to add the Raspberry Pi, Arduino Mega and a powerbank. The feather is attached to the wheel with its separate powerbank. By using tie wraps and tape, these parts are added on the wheelchair.

### Libraries
The following libraries need to be installed in order to successfully run the code. In the code you can find the comments to what the code does and how it works.


>>>>>>> ba9a6e61e1032dbb5fa0c57cfa1d39b8b72b0053

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

### Code
This code needs to run on the Raspberry pi:


### Wiring

![](images/wheelchair_madness.jpg)

https://www.tinkercad.com/things/7NosIb3B7m8-stunning-blad/editel?tenant=circuits?sharecode=-iT8iiwYHXB-MzWQWOSuqNqyqwceIcqcFFds_0Dsx5k=
