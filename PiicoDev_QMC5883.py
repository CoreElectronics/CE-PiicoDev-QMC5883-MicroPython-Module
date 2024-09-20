# Class and methods for the QMC5883 3-axis magnetometer.
# Written by Michael Ruppe at Core Electronics: https://core-electronics.com.au/

# 2024 SEP M.Ruppe Initial Release

import math
from PiicoDev_Unified import *
from struct import unpack
compat_str = '\nUnified PiicoDev library out of date.  Get the latest module: https://piico.dev/unified \n'

_I2C_ADDRESS = 0x0D
# Registers
_ADDRESS_XOUT = 0x00
_ADDRESS_YOUT = 0x02
_ADDRESS_ZOUT = 0x04
_ADDRESS_STATUS = 0x06
_ADDRESS_CONTROL1 = 0x09
_ADDRESS_CONTROL2 = 0x0A
_ADDRESS_ID = 0x0D

_BIT_MODE = 0
_BIT_ODR = 2
_BIT_OSR = 6
_BIT_RANGE = 4

def _readBit(x, n):
    return x & 1 << n != 0

def _setBit(x, n):
    return x | (1 << n)

def _clearBit(x, n):
    return x & ~(1 << n)

def _writeBit(x, n, b):
    if b == 0:
        return _clearBit(x, n)
    else:
        return _setBit(x, n)

def _writeCrumb(x, n, c):
    x = _writeBit(x, n, _readBit(c, 0))
    return _writeBit(x, n+1, _readBit(c, 1))

class PiicoDev_QMC5883(object):
    range_microtesla = {800: 33, 200: 120} # Maps the uT range (key) to resolution (LSB/microtesla)
    def __init__(self, bus=None, freq=None, sda=None, scl=None, addr=_I2C_ADDRESS, odr=3, osr=0, range=800, calibrationFile='calibration.cal', suppress_warnings=False):
        try:
            if compat_ind >= 1:
                pass
            else:
                print(compat_str)
        except:
            print(compat_str)
        self.i2c = create_unified_i2c(bus=bus, freq=freq, sda=sda, scl=scl)
        self.addr = addr
        self.odr = odr
        self.calibrationFile = calibrationFile
        self.suppress_warnings = suppress_warnings
        self._CR1 = 0x00
        self._CR2 = 0x00
        try:
            self.reset()
            
            # Get control registers
            self._CR1 = int.from_bytes(self.i2c.readfrom_mem(self.addr, _ADDRESS_CONTROL1, 1), 'big')
            self._CR2 = int.from_bytes(self.i2c.readfrom_mem(self.addr, _ADDRESS_CONTROL2, 1), 'big')

            self._setMode(1)
            self.setOutputDataRate(odr)
            self.setOverSamplingRatio(osr)
            self.setRange(range)
            self.i2c.writeto_mem(self.addr, _ADDRESS_CONTROL2, bytes([0x41])) # enable address rollover for sequential reads, and enable interrupt pin
                        
        except Exception as e:
            print(i2c_err_str.format(self.addr))
            raise e
        self.x_offset = 0
        self.y_offset = 0
        self.z_offset = 0
        self.declination = 0
        self.data = {}
        self._dataValid = False
        if calibrationFile is not None:
            self.loadCalibration()
        sleep_ms(5)
    
    def reset(self):
        self.i2c.writeto_mem(self.addr, _ADDRESS_CONTROL1, bytes([0x80]))
        
    def _setMode(self, mode):
        self._CR1 = _writeCrumb(self._CR1, _BIT_MODE, mode)
        self.i2c.writeto_mem(self.addr, _ADDRESS_CONTROL1, bytes([self._CR1]))

    def setOutputDataRate(self, odr):
        self._CR1 = _writeCrumb(self._CR1, _BIT_ODR, odr)
        self.i2c.writeto_mem(self.addr, _ADDRESS_CONTROL1, bytes([self._CR1]))

    def setOverSamplingRatio(self, osr):
        self._CR1 = _writeCrumb(self._CR1, _BIT_OSR, osr)
        self.i2c.writeto_mem(self.addr, _ADDRESS_CONTROL1, bytes([self._CR1]))

    def setRange(self, range):
        assert range in [800,200], "range must be 200 or 800 (uTesla)"
        r={800:1, 200:0} # get the range (RNG) crumb value (00 or 01) from the desired range
        self.sensitivity=1/self.range_microtesla[range]
        self._CR1 = _writeCrumb(self._CR1, _BIT_RANGE, r[range])
        self.i2c.writeto_mem(self.addr, _ADDRESS_CONTROL1, bytes([self._CR1]))

    def _convertAngleToPositive(self, angle):
        if angle >= 360.0:
            angle = angle - 360.0
        if angle < 0:
            angle = angle + 360.0
        return angle
    
    def _getStatusReady(self, status):
        return _readBit(status, 0)
        
    def _getStatusOverflow(self, status):
        return _readBit(status, 1)
    
    def read(self, raw=False):
        self._dataValid = False
        NaN = {'x':float('NaN'),'y':float('NaN'),'z':float('NaN')}
        try:
            status = int.from_bytes(self.i2c.readfrom_mem(self.addr, _ADDRESS_STATUS, 1), 'big')
        except:
            print(i2c_err_str.format(self.addr))
            self.sample = NaN
            return NaN
        if self._getStatusReady(status) is True:
            try:
                x,y,z = unpack('<HHH', self.i2c.readfrom_mem(self.addr, _ADDRESS_XOUT, 6))
            except:
                print(i2c_err_str.format(self.addr))
                self.sample = NaN
                return self.sample
            if self._getStatusOverflow(status) is True:
                return NaN
            if raw == False:
                if (x >= 0x8000):
                    x = -((65535 - x) + 1)
                x = (x - self.x_offset)
                if (y >= 0x8000):
                    y = -((65535 - y) + 1)
                y = (y - self.y_offset)
                if (z >= 0x8000):
                    z = -((65535 - z) + 1)
                z = (z - self.z_offset)
                x *= -self.sensitivity
                y *= -self.sensitivity
                z *= -self.sensitivity
            self.sample = {'x':x,'y':y,'z':z}
            self._dataValid = True
            return self.sample
        else:
            print('Not Ready')
            self.sample = NaN
            return self.sample
    
    def dataValid(self):
        return self._dataValid
    
    def readPolar(self):
        data = self.read()
        magnitude = math.sqrt(data['x']*data['x'] + data['y']*data['y'] + data['z']*data['z'])
        # for atan2(), angle is zero along the X-axis. We want angle to be zero along the Y-axis
        angle = ( math.atan2(data['x'],-data['y']) /math.pi)*180.0 + self.declination
        angle = self._convertAngleToPositive(angle)
        
        return {'polar':angle, 'Gauss':magnitude/100, 'uT':magnitude}
    
    def readMagnitude(self):
        data = self.readPolar()
        magnitude = data['uT']
        return magnitude
    
    def readHeading(self):
        return self.readPolar()['polar']
    
    def setDeclination(self, dec):
        self.declination = dec
    
    def calibrate(self, enable_logging=False):
        try:
            self.setOutputDataRate(3)
        except Exception as e:
            print(i2c_err_str.format(self.addr))
            raise e
        x_min = 65535
        x_max = -65535
        y_min = 65535
        y_max = -65535
        z_min = 65535
        z_max = -65535
        log = ''
        print('*** Calibrating.\n    Slowly rotate your sensor until the bar is full')
        print('[          ]', end='')
        range = 1000
        i = 0
        x=0;y=0;z=0;
        a=0.5 # EMA filter weight
        while i < range:
            i += 1
            sleep_ms(5)
            d = self.read(raw=True)
            x = a*d['x'] + (1-a)*x # EMA filter
            y = a*d['y'] + (1-a)*y
            z = a*d['z'] + (1-a)*z
            if x < x_min: x_min = x; i=0
            if x > x_max: x_max = x; i=0
            if y < y_min: y_min = y; i=0
            if y > y_max: y_max = y; i=0
            if z < z_min: z_min = z; i=0
            if z > z_max: z_max = z; i=0
            j = round(10*i/range);
            print( '\015[' + int(j)*'*' + int(10-j)*' ' + ']', end='') # print a progress bar
            if enable_logging:
                log = log + (str(d['x']) + ',' + str(d['y']) + ',' + str(d['z']) + '\n')
        self.setOutputDataRate(self.odr) # set the output data rate back to the user selected rate
        self.x_offset = (x_max + x_min) / 2
        self.y_offset = (y_max + y_min) / 2
        self.z_offset = (z_max + z_min) / 2
        f = open(self.calibrationFile, "w")
        f.write('x_min:\n' + str(x_min) + '\nx_max:\n' + str(x_max) + '\ny_min:\n' + str(y_min) + '\ny_max:\n' + str(y_max) + '\nz_min\n' + str(z_min) + '\nz_max:\n' + str(z_max) + '\nx_offset:\n')
        f.write(str(self.x_offset) + '\ny_offset:\n' + str(self.y_offset) + '\nz_offset:\n' + str(self.z_offset))
        f.close()
        if enable_logging:
            flog = open("calibration.log", "w")
            flog.write(log)
            flog.close

    def loadCalibration(self):
        try:
            f = open(self.calibrationFile, "r")
            for i in range(13): f.readline()
            self.x_offset = float(f.readline())
            f.readline()
            self.y_offset = float(f.readline())
            f.readline()
            self.z_offset = float(f.readline())
            sleep_ms(5)
        except:
            if not self.suppress_warnings:
                print("No calibration file found. Run 'calibrate()' for best results.  Visit https://piico.dev/p33 to read the guide.")
            sleep_ms(1000)
