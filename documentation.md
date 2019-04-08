# Dance wheelchair

In this document we will describe the functioning of our IoT wheelchair. We designed a wheelchair to let users dance. It works like guitar hero. LED signals tell the user what move to make, if the move was correct, the seat will vibrate and the LEDs will tell the user what move to make next.

###Libraries
The following libraries need to be installed in order to successfully run the code:

####In python:
'''python
import pygatt  # To access BLE GATT support
import signal  # To catch the Ctrl+C and end the program properly
import os  # To access environment variables
from dotenv import load_dotenv  # To load environment variables from .env file
import serial
import time
import random # to generate random movements
from dcd.entities.thing import Thing
from dcd.entities.property_type import PropertyType
'''

#### In arduino:
'''


###Code


###Wiring
