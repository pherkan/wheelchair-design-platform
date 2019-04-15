# Dance Wheelchair
In this document the functioning of the Internet-connected Wheelchair called Dance Wheelchair will be explained. This internet-connected wheelchair lets wheelchair users dance in the same way that is being done at a gaming arcade hall. Just as the game screen shows different arrow keys for people to step on the dance platform, LED light signals of Dance Wheelchair  guide the user which direction to move, giving her/him cues to choreograph the dance. If the user moves correctly, she/he is notified with brief vibration and gets next dance cue from the LED lights. In this way, people on a wheelchair can enjoy their own DANCE DANCE REVOLUTION!

1. [Components](#1-Components)
2. [Assembling the wheelchair](#2-Assembling-the-Wheelchair)
3. [Libraries](#3-Libraries)
4. [Code](#4-Code)
5. [Wiring](#5-Wiring)
6. [Poster](#6-Poster)


## 1. Components
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

## 2. Assembling the wheelchair
By using two pieces of plywood and applying these to the inside of the wheels, a space is created to add the Raspberry Pi, Arduino Mega and a big powerbank. The feather and a small powerbank are attached to a spoke of the wheel to detect the rotation. Use cable ties and tape to fix these components to the wheelchair.

## 3. Libraries
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


## 4. Code

### 4.1. Code for Raspberry Pi
This code needs to run on the Raspberry pi:

```C
#!/usr/bin/env python3

# In this example, we connect to a GATT service 'rotation' on the wheel, sending
# the rotation count over time. When we reach the recommended number of rotation,
# we send a command to the Arduino to turn on the vibration motors.

# Import required library
import pygatt  # To access BLE GATT support
import signal  # To catch the Ctrl+C and end the program properly
import os  # To access environment variables
from dotenv import load_dotenv  # To load environment variables from .env file
import serial
import time
import random

# DCD Hub
from dcd.entities.thing import Thing
from dcd.entities.property_type import PropertyType

# The thing ID and access token
load_dotenv()
THING_ID = os.environ['THING_ID']
THING_TOKEN = os.environ['THING_TOKEN']
BLUETOOTH_DEVICE_MAC = os.environ['BLUETOOTH_DEVICE_MAC']

# UUID of the GATT characteristic to subscribe
GATT_CHARACTERISTIC_ROTATION = "02118733-4455-6677-8899-AABBCCDDEEFF"
# Many devices, e.g. Fitbit, use random addressing, this is required to connect.
ADDRESS_TYPE = pygatt.BLEAddressType.random

# Recommended number of rotation
RECOMMENDED_NUM_ROTATION = 1

# Did we already nudged
nudged = False

# points are used to keep track of the amount of correctly executed movements
points = 0

# the first value is saved to be used as starting point
first_values = [0,0]
is_first_value = True

# Start reading the serial port
ser = serial.Serial(
    port = os.environ['SERIAL'],
    baudrate = 9600,
    timeout = 2)


def find_or_create(property_name, property_type):
    """Search a property by name, create it if not found, then return it."""
    if my_thing.find_property_by_name(property_name) is None:
        my_thing.create_property(name=property_name,
                                 property_type=property_type)
    return my_thing.find_property_by_name(property_name)


def handle_rotation_data(handle, value_bytes):
    """
    handle -- integer, characteristic read handle the data was received on
    value_bytes -- bytearray, the data returned in the notification
    """
    print("Received data: %s (handle %d)" % (str(value_bytes), handle))

    rotation_values = [float(x) for x in value_bytes.decode('utf-8').split(",")]
    find_or_create("dance",
                   PropertyType.TWO_DIMENSIONS).update_values(rotation_values)

    # this function generates a random movement and checks if the user completed the movement
    check_movement(rotation_values)

def keyboard_interrupt_handler(signal_num):
    """Make sure we close our program properly"""
    print("Exiting...".format(signal_num))
    left_wheel.unsubscribe(GATT_CHARACTERISTIC_ROTATION)
    exit(0)

def check_movement(rotation_values):
    global is_first_value, first_values, points
    print("point count:", points)
    if is_first_value == True:
        first_values = rotation_values
        random_movement = random.randint(0,1)
        is_first_value = False

    # Start movements
    global random_movement
    dif_forward = rotation_values[0]-first_values[0]
    dif_reverse = rotation_values[1]-first_values[1]

    # Check if user has made the right movement
    if random_movement == 0:
        # tell the user what move to make:
        print("move FORWARD")
        # send sign to the arduino via serial to turn both LED's green
        ser.write('0'.encode())
        # if rotation reached threshhold:
        if (dif_forward) > RECOMMENDED_NUM_ROTATION:
            # send sign to arduino to turn on vibration motor for 2 seconds
            ser.write('4'.encode())
            time.sleep(2)
            global points
            points+=1
            # set current values as starting point of next movement
            first_values = rotation_values
            # generate new random movement
            random_movement = random.randint(0,1)

    elif random_movement == 1:
        # tell the user what move to make:
        print ("move BACKWARD")
        # send the sign to the arduino via serial to turn both LED's red
        ser.write('1'.encode())
        # if rotation reached threshhold:
        if (dif_reverse) > RECOMMENDED_NUM_ROTATION:
            # send sign to arduino to turn on vibration motor for 2 seconds
            ser.write('4'.encode())
            time.sleep(2)
            global points
            points+=1
            # set current values as starting point of next movement
            first_values = rotation_values
            # generate new random movement
            random_movement = random.randint(0,1)

    else :
        exit(0)

# Instantiate a thing with its credential, then read its properties from the DCD Hub
my_thing = Thing(thing_id=THING_ID, token=THING_TOKEN)
my_thing.read()

# Start a BLE adapter
bleAdapter = pygatt.GATTToolBackend()
bleAdapter.start()

# Use the BLE adapter to connect to our device
left_wheel = bleAdapter.connect(BLUETOOTH_DEVICE_MAC, address_type=ADDRESS_TYPE, timeout = 100.0)

# Subscribe to the GATT services
left_wheel.subscribe(GATT_CHARACTERISTIC_ROTATION, callback=handle_rotation_data)

# Register our Keyboard handler to exit
signal.signal(signal.SIGINT, keyboard_interrupt_handler)
```

### 4.2. Code for Adafruit Bluefruit feather
This code needs to run on the Raspberry pi:

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

// LED error flag
#define LED_PIN 2

// Create the Bluefruit object for Feather 32u4
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// BNO settings
#define BNO055_SAMPLERATE_DELAY_MS (200)
// Creating our sensor object to handle the sensor, with initialization 12345
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// structure to store total rotations since IMU  initialized, forward and reverse
// initialized with a global variable global_rotations, this variable stores rotations
// on a particular axis, in both directions, since startup
struct Rotations {
  double forward_rotations = 0;
  double reverse_rotations = 0;
} global_rotations;

bool not_first_loop = false; // Boolean variable to stop logging of first loop
float previous_axis_value = 666;  // Initial value so we don't account for it

// GATT service information
int32_t imuServiceId;
int32_t rotationCharId;
int32_t orientationCharId;

// A small helper
void error(const __FlashStringHelper*err) {
  if (Serial.available()) {
    Serial.println(err);
  }
  // In any case, turn on the LED to signal the error
  analogWrite(LED_PIN, HIGH);
  while (1);
}

// Initializes BNO055 sensor
void initSensor(void) {
  if(!bno.begin()) {
    error(F("No BNO055 detected. Check your wiring or I2C ADDR!"));
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}

// Sets up the HW an the BLE module (this function is called
// automatically on startup)
void setup(void) {
  delay(500);
  boolean success;

  // Set LED error flag

  pinMode(LED_PIN, OUTPUT);
  analogWrite(LED_PIN, LOW);
  Serial.begin(115200);

  // Initialise the module
  if ( !ble.begin(VERBOSE_MODE) ) {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring."));
  }

   // Setup the BNO055 sensor
  initSensor();

  // Perform a factory reset to make sure everything is in a known state
  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset."));
  }

  // Disable command echo from Bluefruit
  ble.echo(false);

  // Print Bluefruit information
  ble.info();
  ble.verbose(true);

  // Change the device name to fit its purpose
  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=dance")) ) {
    error(F("Could not set device name."));
  }

  // Add the IMU Service definition
  success = ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID128=00-11-00-11-44-55-66-77-88-99-AA-BB-CC-DD-EE-FF"), &imuServiceId);
  if (! success) {
    error(F("Could not add Orientation service."));
  }

  // Add the Rotation characteristic
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID128=02-11-87-33-44-55-66-77-88-99-AA-BB-CC-DD-EE-FF,PROPERTIES=0x10,MIN_LEN=1,MAX_LEN=12,VALUE=\"\""), &rotationCharId);
  if (! success) {
    error(F("Could not add Rotation characteristic."));
  }

  // Add the Orientation characteristic
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID128=02-11-88-33-44-55-66-77-88-99-AA-BB-CC-DD-EE-FF,PROPERTIES=0x10,MIN_LEN=1,MAX_LEN=17,VALUE=\"\""), &orientationCharId);
  if (! success) {
    error(F("Could not add Orientation characteristic."));
  }

  // Add the Orientation Service to the advertising data
  // (needed for Nordic apps to detect the service)
  ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-05-02-0d-18-0a-18") );

  // Reset the device for the new service setting changes to take effect
  ble.reset();
}

void orientation() {
  // Get Euler angle data
  imu::Vector<3> euler_vector = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float angleX = euler_vector.x();
  float angleY = euler_vector.y();
  float angleZ = euler_vector.z();

  // Command is sent when \n (\r) or println is called
  // AT+GATTCHAR=CharacteristicID,value
  ble.print( F("AT+GATTCHAR=") );
  ble.print( orientationCharId );
  ble.print( F(",") );
  ble.print(String(angleX));
  ble.print( F(",") );
  ble.print(String(angleY));
  ble.print( F(",") );
  ble.println(String(angleZ));
}

bool compute_rotations(float axis, Rotations * rotations) {
  static float initial_axis_value = axis;
  // variable to store initial axis value in compute rotations - declared static so that it stores
  // this value in between function calls, but no other functions can change its value
  //Variables declared as static will only be created and initialized the first time a function is called

  float offset_rot = (axis-previous_axis_value) / 360; // offset since previous measurement, in rotations

  // so we do not account for anything in the setup phase
  if (previous_axis_value == 666) {
    offset_rot = 0;
  }

  if(offset_rot >= 0) {
    (rotations->forward_rotations) += offset_rot;
  } else {
    (rotations->reverse_rotations) += offset_rot;
  }

  // place previous axis value
  previous_axis_value = axis;

  return(true); // returns true by default, do not remove, as it helps with the initial setup.
}

void rotation() {
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  // if this is the first loop iteration, ignore position data (always zero)
  //if its second loop iteration set the starting position for your axis
  // if its another iteration, just continue computing the rotation data

  float axis_value = event.orientation.x;   // replace this with whatever axis you're tracking
  not_first_loop = (not_first_loop)?compute_rotations(axis_value, &global_rotations) : true;

  // Command is sent when \n (\r) or println is called
  // AT+GATTCHAR=CharacteristicID,value
  ble.print( F("AT+GATTCHAR=") );
  ble.print( rotationCharId );
  ble.print( F(",") );
  ble.print(String(global_rotations.forward_rotations));
  ble.print( F(",") );
  ble.println(String(-global_rotations.reverse_rotations));
}

void loop(void) {

  orientation();
  rotation();

  // Check if command executed OK
  if ( !ble.waitForOK() ) {
    error(F("Failed to get response!"));
  }
  // if (global_rotations.forward_rotations >= 4) {
  //   global_rotations.forward_rotations = 0;
  // }
  // if (global_rotations.reverse_rotations >= 4) {
    // global_rotations.reverse_rotations = 0;
  // }

  // Delay before next measurement update
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
```

### 4.3. Code for Arduino Mega
This code enabled Arduino Mega to receive signals from Raspberry Pi through its serial port and actuates 2 LED lights and the vibration motor.
```C
#include <Adafruit_NeoPixel.h> // Necessary Library include

#define LED_PIN1 2 // Defining the pin of the arduino that sends the data stream.
#define LED_PIN2 7
#define VIB_PIN A10

Adafruit_NeoPixel LED_controller1 = Adafruit_NeoPixel( 1, LED_PIN1, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel LED_controller2 = Adafruit_NeoPixel( 1, LED_PIN2, NEO_RGB + NEO_KHZ800);

int i = 127;
uint8_t R = 0, G = 0, B = 0; // Unsigned integer with 8 bits
uint32_t counter = 0; // 32 bits unsigned integer, we only need 24 to go through all the colors

bool left_red = false;
bool right_red = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //Set serial to 9600 baud
  pinMode(VIB_PIN, OUTPUT);
  LED_controller1.begin(); // We're starting up the library
  LED_controller2.begin(); // We're starting up the library

  LED_controller1.setPixelColor( 0, 0x008000);
  LED_controller2.setPixelColor( 0, 0x008000);
  // Red = 0xFF0000 Green = 0x008000
}

void loop() {

  LED_controller1.show(); // Sending updated pixel color to the hardware
  LED_controller2.show(); // Sending updated pixel color to the hardware

  if (Serial.available() > 0 ) {
    int command = Serial.read();
//  int inByte = Serial.read();

    switch (command) {
      //forward
      case '0' :
        LED_controller1.setPixelColor( 0, 0x008000);
        LED_controller2.setPixelColor( 0, 0x008000);
        LED_controller1.show();
        LED_controller2.show();
        break;
      //backward
      case '1' :
        LED_controller1.setPixelColor( 0, 0xFF0000);
        LED_controller2.setPixelColor( 0, 0xFF0000);
        LED_controller1.show();
        LED_controller2.show();
        break;
      //right
      case '2' :
        LED_controller1.setPixelColor( 0, 0xFF0000);
        LED_controller2.setPixelColor( 0, 0x008000);
        LED_controller1.show();
        LED_controller2.show();
        break;
      //left
      case '3' :
        LED_controller1.setPixelColor( 0, 0x008000);
        LED_controller2.setPixelColor( 0, 0xFF0000);
        LED_controller1.show();
        LED_controller2.show();
        break;
      //vibration
      case '4' :
        analogWrite(VIB_PIN, 153);
        delay(2000);
        analogWrite(VIB_PIN, 0);
      default:
        LED_controller1.setPixelColor( 0, 0xFFFFFF);
        LED_controller2.setPixelColor( 0, 0xFFFFFF);
        LED_controller1.show();
        LED_controller2.show();

      }

    }
}
```



## 5 Wiring

![](images/wheelchair_madness.jpg)

https://www.tinkercad.com/things/7NosIb3B7m8-stunning-blad/editel?tenant=circuits?sharecode=-iT8iiwYHXB-MzWQWOSuqNqyqwceIcqcFFds_0Dsx5k=

## 6 Poster
The poster below shows an overview of the project. Although this project only implemented the IMU Sensor, LED Lights and a vibration motor, the poster shows the use of an extra IMU Sensor and the gesture sensor.
Therefore the data flow shows that the user can move forward, backward, to the left and right. In the code given above it is only possible to let the user move forward and backward.

![](images/Poster-IOT.jpeg)
