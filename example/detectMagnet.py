# Read the magnetic field strength and determine if a magnet is nearby

from PiicoDev_QMC5883 import PiicoDev_QMC5883
from PiicoDev_Unified import sleep_ms
import math
magSensor = PiicoDev_QMC5883(range=800) # initialise the magnetometer
# magSensor.calibrate() # it may be a good idea to calibrate first, in case your sensor has been magnetised

threshold = 120 # microTesla or 'uT'.

while True:

    strength = magSensor.readMagnitude()   # Reads the magnetic-field strength in microTesla (uT)
    myString = str(strength) + ' uT'       # create a string with the field-strength and the unit
    print(myString)                        # Print the field strength
    
    if strength > threshold:               # Check if the magnetic field is strong
        print('Strong Magnet!')

    sleep_ms(1000)
