# Dance Wheelchair
In this document the functioning of the Internet-connected Wheelchair called Dance Wheelchair will be explained. This internet-connected wheelchair lets wheelchair users dance in the same way that is being done at a gaming arcade hall. Just as the game screen shows different arrow keys for people to step on the dance platform, LED light signals of Dance Wheelchair  guide the user which direction to move, giving her/him cues to choreograph the dance. If the user moves correctly, she/he is notified with brief vibration and gets next dance cue from the LED lights. In this way, people on a wheelchair can enjoy their own DANCE DANCE REVOLUTION!

<<<<<<< HEAD
1. [Components](#1-Components)
2. [Assembling the wheelchair](#2-Assembling-the-Wheelchair)
3. [Libraries](#3-Libraries)
4. [Code](#4-Code)
5. [Wiring](#5-Wiring)
6. [Poster](#6-Poster)

## 1 Components
=======
1. [Components/Hardware](#1-Components/Hardware)
2. [Software](#2-Software)
3. [Assembling the wheelchair](#3-Assembling-the-Wheelchair)
4. [Libraries](#4-Libraries)
5. [Code](#5-Code)
6. [Wiring](#6-Wiring)
7. [Poster](#7-Poster)


## 1. Components/Hardware
>>>>>>> b46d1f8887d4e471581ea15cb3a504a178a8cdac
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
- 1 big breadboard
- 1 small breadboard
- 1 Vibration Motor
- 2 Neopixel LEDs
- 2 Condensators
- 2 Resistors of 470Ω

## 2 Assembling the wheelchair
By using two pieces of plywood and applying these to the frame of the wheelchair, a space is created to add the Raspberry Pi, Arduino Mega and a big powerbank. For your own convenience: Make sure to attach the plywood to the frame in a way that the wheelchair is still foldable.

<<<<<<< HEAD
<<<<<<< HEAD
The feather + small breadboard and a small powerbank are attached to a spoke of the wheel to detect the rotation. First check part 5 to wire the feather correctly on the small breadboard. Check part 4 for the code to flash on the feather. If the wiring is correct and the right code flashed, you can use cable ties and tape to fix these components to the wheelchair.
=======
## 3. Assembling the wheelchair
By using two pieces of plywood and applying these to the inside of the wheels, a space is created to add the Raspberry Pi, Arduino Mega and a big powerbank. The feather and a small powerbank are attached to a spoke of the wheel to detect the rotation. Use cable ties and tape to fix these components to the wheelchair. The assembly of the wiring can be found [here](#6-Wiring) whereas an overview of the wheelchair with its components can be found here (#7-Poster).
>>>>>>> b46d1f8887d4e471581ea15cb3a504a178a8cdac
=======
## 3. Assembling The Wheelchair
By using two pieces of plywood and applying these to the inside of the wheels, a space is created to add the Raspberry Pi, Arduino Mega and a big powerbank. The feather and a small powerbank are attached to a spoke of the wheel to detect the rotation. Use cable ties and tape to fix these components to the wheelchair. The assembly of the wiring can be found [here](#6-Wiring) whereas an overview of the wheelchair with its components can be found [here](#7-Poster).
>>>>>>> 3f523b2a0a03f76fef765fa6e6b7f05c5032eabe

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
<<<<<<< HEAD

=======
```C
#include <Adafruit_NeoPixel.h> // Necessary Library include

#define LED_PIN1 2 // Defining the left LED
#define LED_PIN2 7 // Defining the right LED
#define VIB_PIN A10 // Defining Vibration Motor

Adafruit_NeoPixel LED_controller1 = Adafruit_NeoPixel( 1, LED_PIN1, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel LED_controller2 = Adafruit_NeoPixel( 1, LED_PIN2, NEO_RGB + NEO_KHZ800);

int i = 127;
uint8_t R = 0, G = 0, B = 0; // Unsigned integer with 8 bits
uint32_t counter = 0; // 32 bits unsigned integer, we only need 24 to go through all the colors

bool left_red = false;
bool right_red = false;

void setup() {
  Serial.begin(9600); //Set serial to 9600 baud
  pinMode(VIB_PIN, OUTPUT);
  LED_controller1.begin(); // We're starting up the library for the left LED
  LED_controller2.begin(); // We're starting up the library for the right LED

  LED_controller1.setPixelColor( 0, 0x008000);
  LED_controller2.setPixelColor( 0, 0x008000);
  // Red = 0xFF0000 and Green = 0x008000
}

void loop() {

  LED_controller1.show(); // Sending updated pixel color to the hardware
  LED_controller2.show(); // Sending updated pixel color to the hardware

  if (Serial.available() > 0 ) {
    int command = Serial.read();
//  int inByte = Serial.read();

    switch (command) {
      // user needs to go forward
      case '0' :
        LED_controller1.setPixelColor( 0, 0x008000);
        LED_controller2.setPixelColor( 0, 0x008000);
        LED_controller1.show();
        LED_controller2.show();
        break;
      //user needs to go backward
      case '1' :
        LED_controller1.setPixelColor( 0, 0xFF0000);
        LED_controller2.setPixelColor( 0, 0xFF0000);
        LED_controller1.show();
        LED_controller2.show();
        break;
      //user needs to go right
      case '2' :
        LED_controller1.setPixelColor( 0, 0xFF0000);
        LED_controller2.setPixelColor( 0, 0x008000);
        LED_controller1.show();
        LED_controller2.show();
        break;
      //user needs to go left
      case '3' :
        LED_controller1.setPixelColor( 0, 0x008000);
        LED_controller2.setPixelColor( 0, 0xFF0000);
        LED_controller1.show();
        LED_controller2.show();
        break;
      //Vibrate the motor
      case '4' :
        analogWrite(VIB_PIN, 153);
        delay(2000);
        analogWrite(VIB_PIN, 0);
        // Add a default state that makes sure the LED lights are shining white.
      default:
        LED_controller1.setPixelColor( 0, 0xFFFFFF);
        LED_controller2.setPixelColor( 0, 0xFFFFFF);
        LED_controller1.show();
        LED_controller2.show();

      }

    }
}
```



## 6 Wiring
>>>>>>> b46d1f8887d4e471581ea15cb3a504a178a8cdac

## 5 Wiring
Wiring Arduino uno, vibration motor and two RGB LEDs
![](images/wheelchair_madness.jpg)
Wiring 

## 6 Poster

![](images/Poster-IOT.jpeg)
