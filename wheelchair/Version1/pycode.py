#!/usr/bin/env python3

# In this example, we connect to a GATT service 'rotation' on the wheel, sending
# the rotation count over time. We create a random movement which needs to be executed by the user.
# When we reach the recommended number of rotations,
# we send a command to the Arduino to turn on the vibration motors for 2 seconds and to
# color the LED's correctly for the next movement.
# This code is based on the example code created by Jacky 

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
