#
#    Copyright (c) 2020 Alger Pike <Alger_P@pacbell.net>
#
#    See the file LICENSE.txt for your full rights.   
#
#    Based on the Simulator driver from Weewx Simulator.py
#    Copyright (c) 2009-2020 Tom Keffer <tkeffer@gmail.com>
#
#    See the file LICENSE.txt for your full rights.
#
"""Raspberry Pi hardware driver for the weewx weather system"""

from __future__ import with_statement
from __future__ import absolute_import
from __future__ import print_function
import math
import random
import time
import subprocess
import re

import weedb
import weewx.drivers
import weeutil.weeutil

# raspberry pi io pin driver
import RPi.GPIO as GPIO

# Required ADAFruit circuitPython driver which is shared
# between multiple different I2C devices so this object is
# created here. Using circuit python version in order to
# be compatible with Python3. It also seems more stable on
# a ReapberryPi 4.
import board
import busio
import adafruit_am2320
import adafruit_bmp280
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

DRIVER_NAME = 'WeatherPi'
DRIVER_VERSION = "1.0"

def loader(config_dict, engine):

    # Original code and comments from Simulator driver
    #
    # This loader uses a bit of a hack to have the simulator resume at a later
    # time. It's not bad, but I'm not enthusiastic about having special
    # knowledge about the database in a driver, albeit just the loader.

    start_ts = resume_ts = None
    if 'start' in config_dict[DRIVER_NAME]:
        # A start has been specified. Extract the time stamp.
        start_tt = time.strptime(config_dict[DRIVER_NAME]['start'], "%Y-%m-%dT%H:%M")        
        start_ts = time.mktime(start_tt)
        # If the 'resume' keyword is present and True, then get the last
        # archive record out of the database and resume with that.
        if weeutil.weeutil.to_bool(config_dict[DRIVER_NAME].get('resume', False)):
            import weewx.manager
            try:
                # Resume with the last time in the database. If there is no such
                # time, then fall back to the time specified in the configuration
                # dictionary.
                with weewx.manager.open_manager_with_config(config_dict, 'wx_binding') as dbmanager:
                        resume_ts = dbmanager.lastGoodStamp()
            except weedb.OperationalError:
                pass
        else:
            # The resume keyword is not present. Start with the seed time:
            resume_ts = start_ts

    # Instean of loading the Simulator load WeatherPi        
    station = WeatherPi(start_time=start_ts, resume_time=resume_ts, **config_dict[DRIVER_NAME])    
    return station
        
class WeatherPi(weewx.drivers.AbstractDevice):
    """WeatherPi Station driver for supporting various raspberry pi hardware"""
    
    def __init__(self, **stn_dict):
        """Initialize the WeatherPi
        
        NAMED ARGUMENTS:
        
        loop_interval: The time (in seconds) between emitting LOOP packets.
        [Optional. Default is 10.0]

        time_between_observations: The time (in seconds) between each successive
        observation [Optional. Default is 0.1]. I have found that sedning too
        many I2C commands too quickly can cause devices to hang.  This slows down
        the rate to a maximum of 10 commands per second.  Using this seems to
        be more stable.
        
        start_time: The start (seed) time for the generator in unix epoch time
        [Optional. If 'None', or not present, then present time will be used.]

        resume_time: The start time for the loop.
        [Optional. If 'None', or not present, then start_time will be used.]
        
        mode: Controls the frequency of packets.  One of either:
            'polled': - sleep between LOOP packets (once per loop_interval)
            'generator': Emit packets as fast as possible (useful for testing) but could be too fast for real hardwre...
        [Required. Default is polled.]

        observations: Comma-separated list of observations that should be
                      generated.  If nothing is specified, then all
                      observations will be generated. Whe defining the list
                      each observation is intialized with the device responsible
                      for making the measrement.
        [Optional. Default is not defined.]

        wind_direction_offset_angle:  Use this number to calibrate the wind vane measurements.
        For example for my weather station it is most practical to have the North face of 
        the wind vane to actually be pointing to the west.  I can now simply subtract 90 
        degrees from the vane measurements so that the measurement indicates west.
        [Optional. Default is 0 in which case the wind vane is truly pointing north when it
        measures north.]
        """

        self.loop_interval = float(stn_dict.get('loop_interval', 10.0))
        self.time_between_observations = float(stn_dict.get('time_between_observations', 0.1))
        self.wind_direction_offset_angle = float(stn_dict.get('wind_direction_offset_angle', 0.0))
        if 'start_time' in stn_dict and stn_dict['start_time'] is not None:
            # A start time has been specified. We are not in real time mode.
            self.real_time = False
            # Extract the generator start time:
            start_ts = float(stn_dict['start_time'])
            # If a resume time keyword is present (and it's not None), 
            # then have the generator resume with that time.
            if 'resume_time' in stn_dict and stn_dict['resume_time'] is not None:
                self.the_time = float(stn_dict['resume_time'])
            else:
                self.the_time = start_ts
        else:
            # No start time specified. We are in realtime mode.
            self.real_time = True
            start_ts = self.the_time = time.time()

        # default to polled mode
        self.mode = stn_dict.get('mode', 'polled')
        
        # Create ADA fruit circuit python devices.
        # See ADA fruit docs on the latest on how to install the
        # hardware driver libraries.
        #
        # In general the procedure will be similair to the following:
        # sudo pip3 install adafruit-circuitpython-am2320
        # sudo pip3 install adafruit-circuitpython-bmp280
        # sudo pip3 install adafruit-circuitpython-ads1x15
        #
        # More infomration can be found at:
        # https://learn.adafruit.com/am2315-encased-i2c-temperature-humidity-sensor/python-circuitpython
        # https://learn.adafruit.com/adafruit-bmp280-barometric-pressure-plus-temperature-sensor-breakout/circuitpython-test
        # https://learn.adafruit.com/adafruit-4-channel-adc-breakouts/python-circuitpython
        self.i2c = busio.I2C(board.SCL, board.SDA) 

        from adafruit_am2320 import AM2320DeviceNotFound
        try:       
            self.am2315 = adafruit_am2320.AM2320(self.i2c) 
        except AM2320DeviceNotFound :
            self.am2315 = None

        self.bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(self.i2c)
        self.adc = ADS.ADS1115(self.i2c)
        self.chan = AnalogIn(self.adc, ADS.P0)

        # Using the RPi.GPIO library for pin access in python gere
        # Set GPIO pins to use BCM pin numbers.
        # The procedure to install is similar to above
        #
        # sudo apt-get install rpi.gpio
        # sudo python3 setup.py install
        GPIO.setmode(GPIO.BCM)

        # Provide the class that will be used to measure
        # the labelled observed measurement! Use the special
        # none sensor class to provide None for values you
        #  are not tracking (or alternatively do not include)
        # those values in the list.
        self.observations = {
            'outTemp'    : noneSensor() if self.am2315 == None else am2320TemperatureSensor(self.am2315),
            'inTemp'     : bmp280TemperatureSensor(self.bmp280),
            'altimeter'  : bmp280AltitudeSensor(self.bmp280),
            'barometer'  : bmp280BarometerSensor(self.bmp280),
            'pressure'   : noneSensor(),
            'windSpeed'  : sen08942Anemometer(26),
            'windDir'    : sen08942VaneSensor(self.chan, self.wind_direction_offset_angle),
            'windGust'   : sen08942GustAnemometer(26),
            'windGustDir': noneSensor(),
            'outHumidity': noneSensor() if self.am2315 == None else am2320HumiditySensor(self.am2315),
            'inHumidity' : noneSensor(),
            'radiation'  : noneSensor(),
            'UV'         : noneSensor(),
            'rain'       : sen08942RainSensor(12),
            'txBatteryStatus': noneSensor(),
            'windBatteryStatus': noneSensor(),
            'rainBatteryStatus': noneSensor(),
            'outTempBatteryStatus': noneSensor(),
            'inTempBatteryStatus': noneSensor(),
            'consBatteryVoltage': noneSensor(),
            'heatingVoltage': noneSensor(),
            'supplyVoltage': noneSensor(),
            'referenceVoltage': noneSensor(),
            'rxCheckPercent': SignalStrength()}

        # calculate only the specified observations, or all if none specified
        if 'observations' in stn_dict and stn_dict['observations'] is not None:
            desired = [x.strip() for x in stn_dict['observations'].split(',')]
            for obs in self.observations:
                if obs not in desired:
                    del self.observations[obs]

    def genLoopPackets(self):

        while True:

            # If we are in simulator mode, sleep first (as if we are gathering
            # observations). If we are in generator mode, don't sleep at all.
            if self.mode == 'polled':
                # Determine how long to sleep
                if self.real_time:
                    # We are in real time mode. Try to keep synched up with the
                    # wall clock
                    sleep_time = self.the_time + self.loop_interval - time.time()
                    if sleep_time > 0: 
                        time.sleep(sleep_time)
                else:
                    # A start time was specified, so we are not in real time.
                    # Just sleep the appropriate interval
                    time.sleep(self.loop_interval)

            # Update the simulator clock:
            self.the_time += self.loop_interval
            
            # Because a packet represents the measurements observed over the
            # time interval, we want the measurement values at the middle
            # of the interval.
            avg_time = self.the_time - self.loop_interval/2.0
            
            _packet = {'dateTime': int(self.the_time+0.5),
                       'usUnits' : weewx.METRIC }
            for obs_type in self.observations:
                _packet[obs_type] = self.observations[obs_type].value_at(avg_time)
                time.sleep(self.time_between_observations)
            yield _packet

    def getTime(self):
        return self.the_time
    
    @property
    def hardware_name(self):
        return "WeatherPi"

class am2320TemperatureSensor(object):
    """Read temperature from am23xx temperature sensor.  Note: this
    device may not always have a successful read. If it is queried too
    many times too quickly it can overheat and not respond.  An interval
    time of ten seconds with a time between observations o 0.1 seconds
    seems to be robust."""    

    def __init__(self, sensor):
        self._sensor = sensor

    def value_at(self, time_ts):
        temperature = None

        from adafruit_am2320 import AM2320ReadError
        try:
            temperature = self._sensor.temperature
        except AM2320ReadError:
            pass
        return temperature 

class am2320HumiditySensor(object):
    """Read humidity from am23xx temperature sensor.  Note: this
    device may not always have a successful read. If it is queried too
    many times too quickly it can overheat and not respond.  An interval
    time of ten seconds with a time between observations o 0.1 seconds
    seems to be robust."""  

    def __init__(self, sensor):
        self._sensor = sensor

    def value_at(self, time_ts):
        humidity = None

        from adafruit_am2320 import AM2320ReadError
        try:
            humidity = self._sensor.relative_humidity
        except AM2320ReadError:
            pass
        return humidity  

class bmp280TemperatureSensor(object):
    """Read temperature from bmp280 weather sensor.  Note: this
    device may not always have a successful read. If it is queried too
    many times too quickly may not respond.  An interval time of ten
    seconds with a time between observations o 0.1 seconds seems to be
    robust.  When this happens the device may disconnect from the I2C
    bus and not be visiable with i2cdetect -y 1. If this happens a power
    cycle of the Raspberry Pi seems to recover the devicve."""  

    def __init__(self, sensor):
        self._sensor = sensor

    def value_at(self, time_ts):
        temperature = self._sensor.temperature
        return temperature
        
class bmp280AltitudeSensor(object):
    """Read altitude from bmp280 weather sensor.  Note: this
    device may not always have a successful read. If it is queried too
    many times too quickly may not respond.  An interval time of ten
    seconds with a time between observations o 0.1 seconds seems to be
    robust.  When this happens the device may disconnect from the I2C
    bus and not be visiable with i2cdetect -y 1. If this happens a power
    cycle of the Raspberry Pi seems to recover the devicve."""

    def __init__(self, sensor):
        self._sensor = sensor

    def value_at(self, time_ts):
        altitude = self._sensor.altitude
        return altitude

class bmp280BarometerSensor(object):
    """Read the Barometer of the bmp280 weather sensor.  Note: this
    device may not always have a successful read. If it is queried too
    many times too quickly may not respond.  An interval time of ten
    seconds with a time between observations o 0.1 seconds seems to be
    robust.  When this happens the device may disconnect from the I2C
    bus and not be visiable with i2cdetect -y 1. If this happens a power
    cycle of the Raspberry Pi seems to recover the devicve."""

    def __init__(self, sensor):
        self._sensor = sensor

    def value_at(self, time_ts):
        pressure = self._sensor.pressure
        return pressure

class sen08942Anemometer(object):
    """Perform a wind speed measurement using the sen08942 Anemometer.
    This is the weather sensor that is shipped as part of the sparkfun
    weather station.  This code also seems to work on the latest revision
    as well which is sen15901.  This class defines static data that is
    also shared and used by the derived Wind Gust class below."""

    _windTimes = []
    _lastWindSample = time.time()
    _windGustRate = None
    _gpioIntialized = False

    def __init__(self, ioPin):             

        # since this library is static intialization can
        # only be performed once without error. Could also
        # define __del__(self): to unint using library
        if(sen08942Anemometer._gpioIntialized == False):
            GPIO.setup(ioPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(ioPin, GPIO.RISING)

            GPIO.add_event_callback(ioPin, sen08942Anemometer.windEvent)
            sen08942Anemometer._gpioIntialized = True

    @staticmethod
    def windEvent(arg): 
        currentWindSample = time.time()
        debounce = currentWindSample - sen08942Anemometer._lastWindSample
        if debounce > 0.02 :
            sen08942Anemometer._windTimes.append(currentWindSample)
            sen08942Anemometer._lastWindSample = currentWindSample

    def value_at(self, time_ts):
        windTimes = sen08942Anemometer._windTimes
        sen08942Anemometer._windTimes = []

        windSamples = len(windTimes)
        if windSamples <= 1 :
            _windGustRate = None
            return 0.0

        windRate = [windTimes[i + 1] - windTimes[i] for i in range(windSamples - 1)]
        sen08942Anemometer._windGustRate = min(windRate)

        avgRate = sum(windRate) / len(windRate)
        windSpeed = (2.4 / avgRate) # 2.4 km / h = 0.667 meters / second
        return windSpeed

class sen08942GustAnemometer(sen08942Anemometer):
    """Perform a wind speed measurement using the sen08942 Anemometer.
    This is the weather sensor that is shipped as part of the sparkfun
    weather station.  This code also seems to work on the latest revision
    as well which is sen15901.  This class uses shared static data with
    its base class."""   

    def value_at(self, time_ts):
        windGustRate = sen08942Anemometer._windGustRate
        sen08942Anemometer._windGustRate = None
        if windGustRate == None or windGustRate < 0.0001 :
           sen08942Anemometer._windGustRate = None
           return 0.0

        windGustSpeed = (2.4 / windGustRate) # 2.4 km / h = 0.667 meters / second
        return windGustSpeed

class sen08942RainSensor(object):
    """Perform a rain measurement using the sen08942 Anemometer.
    This is the weather sensor that is shipped as part of the sparkfun
    weather station.  This code also seems to work on the latest revision
    as well which is sen15901.""" 

    _rainTick = 0  
    _lastRainSample = time.time()  
    _gpioIntialized = False

    def __init__(self, ioPin):             

        # since this library is static intialization can
        # only be performed once without error. Could also
        # define __del__(self): to unint using library
        if(sen08942RainSensor._gpioIntialized == False):
            GPIO.setup(ioPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(ioPin, GPIO.RISING)

            GPIO.add_event_callback(ioPin, sen08942RainSensor.rainEvent)
            sen08942RainSensor._gpioIntialized = True

    @staticmethod
    def rainEvent(arg): 
        currentRainSample = time.time()
        debounce = currentRainSample - sen08942RainSensor._lastRainSample
        if debounce > 2.0 :
            sen08942RainSensor._rainTick += 1
            sen08942RainSensor._lastRainSample = currentRainSample

    def value_at(self, time_ts):
        rainTicks = sen08942RainSensor._rainTick
        sen08942RainSensor._rainTick = 0.0
        rainAmount = rainTicks * 0.02794 # 1 tick = 0.2794 mm or 0.02794
        return rainAmount

class sen08942VaneSensor(object):
    """Perform a wind diection measurement using the sen08942 Anemometer.
    This is the weather sensor that is shipped as part of the sparkfun
    weather station.  This code also seems to work on the latest revision
    as well which is sen15901. I found one of the reed switches on my device
    was permanently stuck open.  Therefor the calibrated volatge values may
    be slightly different for your wind vane.  I found it useful to tape the
    rotor to the base to keep it still while I measured the voltage at each
    position.  The values below will be close but may need to be updated for
    your particular wind vane.""" 

    def __init__(self, sensor, wind_direction_offset_angle):
        self._sensor = sensor
        self.wind_direction_offset_angle = wind_direction_offset_angle

    def value_at(self, time_ts):
        degrees = None
        voltage = self._sensor.voltage        

        # Voltages measured at each reed position when closed.
        # Sparkfun documentation outlines what ideal values
        # should be for 3.3 and 5 volt reference sources.
        if 1.3 <= voltage <= 1.5:
            degrees = 0.0        
        if 3.0 <= voltage <= 3.3:
            degrees = 45.0
        if 4.0 <= voltage <= 4.1:
            degrees = 90.0
        if 3.6 <= voltage <= 3.8:
            degrees = 135.0
        if 3.8 <= voltage <= 4.0:
            degrees = 180.0
        if 2.2 <= voltage <= 2.4:
            degrees = 225.0
        if 0.3 <= voltage <= 0.5:
            degrees = 270.0
        if 0.8 <= voltage <= 1.0:
            degrees = 315.0
        if degrees == None :
            calibratedResult = None
        
        calibratedResult = None
        if degrees != None :
            calibratedResult = 360.0 + degrees + self.wind_direction_offset_angle
            calibratedResult = calibratedResult % 360
        return calibratedResult 

class noneSensor(object):
    """This class is useful for provideing a None for any obsrvation that
    would like to turn off.  This can be useful for debugging too."""

    def value_at(self, time_ts):        
        return None

class SignalStrength(object):
    """Query Raspberry Pi WiFi signal strength"""      

    def value_at(self, time_ts):
        proc = subprocess.Popen(["iwconfig", "wlan0"],stdout=subprocess.PIPE, universal_newlines=True)
        out, err = proc.communicate()

        m = re.search("Link Quality=(?P<strength>[0-9][0-9])/(?P<max_strength>[0-9][0-9])", out)

        strength = int(m.group('strength'))
        max_strength = int(m.group('max_strength'))
        resultPerCent = strength / max_strength * 100
        return resultPerCent

def confeditor_loader():
    return WeatherPiConfEditor()


class WeatherPiConfEditor(weewx.drivers.AbstractConfEditor):
    @property
    def default_stanza(self):
        return """
[WeatherPi]
    # This section is for the weewx WeatherPi station driver

    # The time (in seconds) between LOOP packets.
    loop_interval = 10.0

    # The simulator mode can be either 'simulator' or 'generator'.
    # Real-time simulator. Sleep between each LOOP packet.
    mode = simulator
    # Generator.  Emit LOOP packets as fast as possible (useful for testing).
    #mode = generator

    # The start time. Format is YYYY-mm-ddTHH:MM. If not specified, the default 
    # is to use the present time.
    #start = 2011-01-01T00:00

    # The driver to use:
    driver = weewx.drivers.WeatherPi
"""


if __name__ == "__main__":
    station = WeatherPi(mode='simulator',loop_interval=2.0)
    for packet in station.genLoopPackets():
        print(weeutil.weeutil.timestamp_to_string(packet['dateTime']), packet)
