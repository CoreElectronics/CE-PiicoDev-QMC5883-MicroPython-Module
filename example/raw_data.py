# Prints the raw axis readings in micro-Tesla

from PiicoDev_QMC5883 import PiicoDev_QMC5883
from PiicoDev_Unified import sleep_ms
import math
magSensor = PiicoDev_QMC5883(range=200) # Initialise the sensor

while True:
    raw_data = magSensor.read() # Read the field strength on each axis in uT
#     raw_data = magSensor.read(raw=True) # Read the raw, unscaled data on each axis

    print(raw_data)             # Print the data
    sleep_ms(200)

