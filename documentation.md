# Dance Wheelchair
In this document the functioning of the Internet-connected Wheelchair called Dance Wheelchair will be explained. This internet-connected wheelchair lets wheelchair uses dance in the same way that is being done at a gaming arcade hall. LED signals tell the user what to do in a way how to choreograph the dance. If the user moves correctly, he gets input in the form of the LED lights and a vibration. Subsequently, the user gets input on the next move to perform.

![](images/Poster-IOT.jpeg)

### Libraries
The following libraries need to be installed in order to successfully run the code. In the code you can find the comments to what the code does and how it works.

#### In python on Raspberry pi:
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


### Wiring

![](images/wheelchair_madness.jpg)

https://www.tinkercad.com/things/7NosIb3B7m8-stunning-blad/editel?tenant=circuits?sharecode=-iT8iiwYHXB-MzWQWOSuqNqyqwceIcqcFFds_0Dsx5k=
