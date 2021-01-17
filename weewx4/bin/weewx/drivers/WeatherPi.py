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
from enum import Enum
import math
import sys
import json
import random
import time
import subprocess
import logging
import re

import weedb
import weewx.drivers
import weeutil.weeutil
import weeutil.logger

# raspberry pi io pin driver
import RPi.GPIO as GPIO

# Required ADAFruit circuitPython driver which is shared
# between multiple different I2C devices so this object is
# created here. Using circuit python version in order to
# be compatible with Python3. It also seems more stable on
# a RaspberryPi 4.
import board
import busio
import adafruit_am2320
import adafruit_bmp280
import adafruit_sht31d
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

DRIVER_NAME = 'WeatherPi'
DRIVER_VERSION = "1.0"

log = logging.getLogger(__name__)

class TemperatureSensors(Enum):    
    BMP280 = 1
    AM2315 = 2
    BSHT30A = 3
    FT020T = 4 # switchdoc wireless weather station
    F016TH = 5 # switchdoc wireless Thermo-Hygrometer

# The following emumertations define what hardware is supported
# a string with the same name should appear next the entry in the
#  config file.
class HumiditySensors(Enum):    
    AM2315 = 2
    BSHT30A = 3
    FT020T = 4 # switchdoc wireless weather station
    F016TH = 5 # switchdoc wireless Thermo-Hygrometer

class AltitudeSensors(Enum):
    BMP280 = 1

class BarometerSensors(Enum):
    BMP280 = 1

class WindSpeedSensors(Enum):
    SparkFun08942 = 1
    SparkFun15901 = 2
    WeatherRack0015WRDSBT = 3
    FT020T = 4 # switchdoc wireless weather station

class WindDirectionSensors(Enum):
    SparkFun08942 = 1
    SparkFun15901 = 2
    WeatherRack0015WRDSBT = 3

class WindGustSensors(Enum):
    SparkFun08942 = 1
    SparkFun15901 = 2
    WeatherRack0015WRDSBT = 3
    FT020T = 4 # switchdoc wireless weather station

class RainSensors(Enum):
    SparkFun08942 = 1
    SparkFun15901 = 2
    WeatherRack0015WRDSBT = 3
    FT020T = 4 # switchdoc wireless weather station

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

        observations: Comma-separated  device list of observations that should be
                      generated.  Each entry is a the name of an observation
                      followed vy a colon followed by the name of the hardware
                      device to use for the observation
                      i.e.  "outTemp: None, inTemp: BSHT30A"

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

        # Init devicesa to None
        self.BSHT30A = None
        self.am2315 = None
        self.bmp280 = None
        self.adc = None
        self.chan = None
        self.sdr = None

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
        deviceList = dict(device.split(":") for device in stn_dict['observations'])
            

        self.observations = {
            'outTemp'    : self.BuildTemperatureSensor(deviceList.get('outTemp', "None").strip()),
            'inTemp'     : self.BuildTemperatureSensor(deviceList.get('inTemp', "None").strip()),
            'altimeter'  : self.BuildAltitudeSensor(deviceList.get('altimeter', "None").strip(), self.i2c),
            'barometer'  : self.BuildBarometerSensor(deviceList.get('barometer', "None").strip(), self.i2c),
            'pressure'   : noneSensor(),
            'windSpeed'  : self.BuildWindSpeedSensor(deviceList.get('windSpeed', "None").strip(), 26),
            'windDir'    : self.BuildWindDirectionSensor(deviceList.get('windDir', "None").strip(), self.wind_direction_offset_angle), 
            'windGust'   : self.BuildWindGustSensor(deviceList.get('windGust', "None").strip(), 26),
            'windGustDir': noneSensor(),
            'outHumidity': self.BuildHumiditySensor(deviceList.get('outHumidity', "None").strip()),
            'inHumidity' : self.BuildHumiditySensor(deviceList.get('inHumidity', "None").strip()),
            'radiation'  : noneSensor(),
            'UV'         : noneSensor(),
            'rain'       : self.BuildRainSensor(deviceList.get('rain', "None").strip(), 12), 
            'txBatteryStatus': noneSensor(),
            'windBatteryStatus': noneSensor(),
            'rainBatteryStatus': noneSensor(),
            'outTempBatteryStatus': noneSensor(),
            'inTempBatteryStatus': noneSensor(),
            'consBatteryVoltage': noneSensor(),
            'heatingVoltage': noneSensor(),
            'supplyVoltage': noneSensor(),
            'referenceVoltage': noneSensor(),
            'rxCheckPercent': SignalStrength()
            }

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

    def BuildBSHT3xATemperatureSensor(self, sensorType):
        try:       
            if self.BSHT30A == None :
                self.BSHT30A = adafruit_sht31d.SHT31D(self.i2c)
            sensor = bsht30TemperatureSensor(self.BSHT30A)
        # ealier version of circuit python uses this,
        #except AM2320DeviceNotFound :
        except ValueError:
            log.error(sensorType + " not found continue as None")            
            self.am2315 = None
            sensor = None
        return sensor

    def BuildAM23xxTemperatureSensor(self, sensorType):
        # ealier version of circuit python uses this,
        # from adafruit_am2320 import AM2320DeviceNotFound
        try: 
            if self.am2315 == None :
                self.am2315 = adafruit_am2320.AM2320(self.i2c) 
            sensor = am2320TemperatureSensor(self.am2315)
        # ealier version of circuit python uses this,
        #except AM2320DeviceNotFound :
        except ValueError:
            log.error(sensorType + " not found continue as None")            
            self.am2315 = None
            sensor = None
        return sensor

    def BuildBMPx80TemperatureSensor(self, sensorType):
        try:
            if self.bmp280 == None :
                self.bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(self.i2c)
            sensor = bmp280TemperatureSensor(self.bmp280)
        except ValueError:
            log.error(sensorType + " sensor not found")  
            self.bmp280 = None
            sensor = None
        return sensor

    def BuildFT020TTemperatureSensor(self, sensorType):
        try:
            if self.sdr == None :                
                self.sdr = softwareDefinedRadio()
                self.sdr.readSensors()
 
            sensor = ft020tTemperatureSensor(self.sdr)
        except:
            log.error(sensorType + " sensor not found")
            sensor = None
        return sensor

    def BuildF016THTemperatureSensor(self, sensorType):
        try:
            if self.sdr == None :                
                self.sdr = softwareDefinedRadio()
                self.sdr.readSensors()
 
            sensor = f016thTemperatureSensor(self.sdr)
        except:
            log.error(sensorType + " sensor not found")
            sensor = None
        return sensor

    def BuildTemperatureSensor(self, sensorType):
        if sensorType == "None": 
            return noneSensor()

        try:
            sensor = TemperatureSensors[sensorType]
        except KeyError:
            log.error("Invalid temperature sensor: %s", sensorType)      
            return None

        switcher = {
            TemperatureSensors.BSHT30A: self.BuildBSHT3xATemperatureSensor, 
            TemperatureSensors.AM2315: self.BuildAM23xxTemperatureSensor,
            TemperatureSensors.BMP280: self.BuildBMPx80TemperatureSensor,
            TemperatureSensors.FT020T: self.BuildFT020TTemperatureSensor,
            TemperatureSensors.F016TH: self.BuildF016THTemperatureSensor
        }
        create = switcher.get(sensor)
        sensor = create(sensorType)
        return sensor

    def BuildBSHT3xAHumiditySensor(self, sensorType):
        try: 
            if self.BSHT30A == None :
                self.BSHT30A = adafruit_sht31d.SHT31D(self.i2c) 
            sensor = bsht30HumiditySensor(self.BSHT30A)
        # ealier version of circuit python uses this,
        #except AM2320DeviceNotFound :
        except ValueError:
            log.error(sensorType + " not found continue as None")            
            self.am2315 = None
            sensor = None
        return sensor

    def BuildAM23xxHumiditySensor(self, sensorType):
        # ealier version of circuit python uses this,
        # from adafruit_am2320 import AM2320DeviceNotFound
        try: 
            if self.am2315 == None :      
                self.am2315 = adafruit_am2320.AM2320(self.i2c) 
            sensor = am2320HumiditySensor(self.am2315)
        # ealier version of circuit python uses this,
        #except AM2320DeviceNotFound :
        except ValueError:
            log.error(sensorType + " not found continue as None")            
            self.am2315 = None
            sensor = None
        return sensor

    def BuildFT020THumiditySensor(self, sensorType):
        try:
            if self.sdr == None :                
                self.sdr = softwareDefinedRadio()
                self.sdr.readSensors()
 
            sensor = ft020tHumiditySensor(self.sdr)
        except:
            log.error(sensorType + " sensor not found")
            sensor = None
        return sensor

    def BuildF016THHumiditySensor(self, sensorType):
        try:
            if self.sdr == None :                
                self.sdr = softwareDefinedRadio()
                self.sdr.readSensors()
 
            sensor = f016thHumiditySensor(self.sdr)
        except:
            log.error(sensorType + " sensor not found")
            sensor = None
        return sensor    

    def BuildHumiditySensor(self, sensorType):
        if sensorType == "None": 
            return noneSensor()

        try:
            sensor = HumiditySensors[sensorType]
        except KeyError:
            log.error("Invalid humidity sensor: %s", sensorType)      
            return None

        switcher = {
            HumiditySensors.BSHT30A: self.BuildBSHT3xAHumiditySensor, 
            HumiditySensors.AM2315: self.BuildAM23xxHumiditySensor,
            HumiditySensors.FT020T: self.BuildFT020THumiditySensor,
            HumiditySensors.F016TH: self.BuildF016THHumiditySensor                      
        }
        create = switcher.get(sensor)
        sensor = create(sensorType)
        return sensor

    def BuildBMP280AltitdueSensor(self, sensorType, i2c):
        try:
            if self.bmp280 == None :
                self.bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(self.i2c)
            sensor = bmp280AltitudeSensor(self.bmp280)
        except ValueError:
            log.error(sensorType + " sensor not found")  
            self.bmp280 = None
            sensor = None
        return sensor


    def BuildAltitudeSensor(self, sensorType, i2c):
        if sensorType == "None": 
            return noneSensor()

        try:
            sensor = AltitudeSensors[sensorType]
        except KeyError:
            log.error("Invalid altitude sensor: %s", sensorType)      
            return None

        switcher = {
            AltitudeSensors.BMP280: self.BuildBMP280AltitdueSensor                    
        }
        create = switcher.get(sensor)
        sensor = create(sensorType, i2c)
        return sensor
    
    def BuildBMP280BarometerSensor(self, sensorType, i2c):
        try:
            if self.bmp280 == None :
                self.bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(self.i2c)
            sensor = bmp280BarometerSensor(self.bmp280)
        except ValueError:
            log.error(sensorType + " sensor not found")  
            self.bmp280 = None
            sensor = None
        return sensor


    def BuildBarometerSensor(self, sensorType, i2c):
        if sensorType == "None": 
            return noneSensor()

        try:
            sensor = BarometerSensors[sensorType]
        except KeyError:
            log.error("Invalid barometer sensor: %s", sensorType)      
            return None

        switcher = {
            BarometerSensors.BMP280: self.BuildBMP280BarometerSensor                    
        }
        create = switcher.get(sensor)
        sensor = create(sensorType, i2c)
        return sensor

    def BuildSparkFun08942WindSpeedSensor(self, sensorType, ioPin):
        try:            
            sensor = sen08942Anemometer(ioPin)
        except ValueError:
            log.error("%s sensor not found", sensorType) 
            sensor = None
        return sensor

    def BuildFT020TWindSpeedSensor(self, sensorType, ioPin):
        # io pin is not used for this sensor.
        try:            
            sensor = ft020tAnemometer(self.sdr)
        except:
            log.error("%s sensor not found", sensorType) 
            sensor = None
        return sensor


    def BuildWindSpeedSensor(self, sensorType, ioPin):
        if sensorType == "None": 
            return noneSensor()

        try:
            sensor = WindSpeedSensors[sensorType]
        except KeyError:
            log.error("Invalid wind speed sensor: %s", sensorType)      
            return None

        switcher = {
            WindSpeedSensors.SparkFun08942: self.BuildSparkFun08942WindSpeedSensor,
            WindSpeedSensors.SparkFun15901: self.BuildSparkFun08942WindSpeedSensor,
            WindSpeedSensors.WeatherRack0015WRDSBT: self.BuildSparkFun08942WindSpeedSensor,
            WindSpeedSensors.FT020T: self.BuildFT020TWindSpeedSensor
        }
        create = switcher.get(sensor)
        sensor = create(sensorType, ioPin)
        return sensor

     
    def BuildSparkFun08942WindDirectionSensor(self, sensorType, offsetAngle):
        try:
            if self.chan == None:      
                self.adc = ADS.ADS1115(self.i2c)
                self.chan = AnalogIn(self.adc, ADS.P0)          
            sensor = sen08942VaneSensor(self.chan, offsetAngle)
        except ValueError:
            log.error("%s sensor not found", sensorType)
            self.adc = None
            self.chan = None 
            sensor = None
        return sensor


    def BuildWindDirectionSensor(self, sensorType, ioPin):
        if sensorType == "None": 
            return noneSensor()

        try:
            sensor = WindDirectionSensors[sensorType]
        except KeyError:
            log.error("Invalid wind direction sensor: %s", sensorType)      
            return None

        switcher = {
            WindDirectionSensors.SparkFun08942: self.BuildSparkFun08942WindDirectionSensor,
            WindDirectionSensors.SparkFun15901: self.BuildSparkFun08942WindDirectionSensor,
            WindDirectionSensors.WeatherRack0015WRDSBT: self.BuildSparkFun08942WindDirectionSensor
        }
        create = switcher.get(sensor)
        sensor = create(sensorType, ioPin)
        return sensor

    def BuildSparkFun08942WindGustSensor(self, sensorType, ioPin):
        try:            
            sensor = sen08942GustAnemometer(26)
        except ValueError:
            log.error(sensorType + " sensor not found") 
            sensor = None
        return sensor

    def BuildFT020TWindGustSensor(self, sensorType, ioPin):
        # ioPin is not used for this sensor
        try:            
            sensor = ft020tGustAnemometer(self.sdr)
        except ValueError:
            log.error(sensorType + " sensor not found") 
            sensor = None
        return sensor

    def BuildWindGustSensor(self, sensorType, ioPin):
        if sensorType == "None": 
            return noneSensor()

        try:
            sensor = WindGustSensors[sensorType]
        except KeyError:
            log.error("Invalid wind speed sensor: %s", sensorType)      
            return None

        switcher = {
            WindGustSensors.SparkFun08942: self.BuildSparkFun08942WindGustSensor,
            WindGustSensors.SparkFun15901: self.BuildSparkFun08942WindGustSensor,
            WindGustSensors.WeatherRack0015WRDSBT: self.BuildSparkFun08942WindGustSensor,
            WindGustSensors.FT020T: self.BuildFT020TWindGustSensor
        }
        create = switcher.get(sensor)
        sensor = create(sensorType, ioPin)
        return sensor

    def BuildSparkFun08942RainSensor(self, sensorType, ioPin):
        try:            
            sensor = sen08942RainSensor(ioPin)
        except ValueError:
            log.error("%s sensor not found", sensorType) 
            sensor = None
        return sensor

    def BuildFT020TRainSensor(self, sensorType, ioPin):
        # pin not used on this sensor.
        try:            
            sensor = ft020tRainSensor(self.sdr)
        except:
            log.error("%s sensor not found", sensorType) 
            sensor = None
        return sensor


    def BuildRainSensor(self, sensorType, ioPin):
        if sensorType == "None": 
            return noneSensor()

        try:
            sensor = RainSensors[sensorType]
        except KeyError:
            log.error("Invalid rain sensor: %s", sensorType)      
            return None

        switcher = {
            RainSensors.SparkFun08942: self.BuildSparkFun08942RainSensor,
            RainSensors.SparkFun15901: self.BuildSparkFun08942RainSensor,
            RainSensors.WeatherRack0015WRDSBT: self.BuildSparkFun08942RainSensor,
            RainSensors.FT020T: self.BuildFT020TRainSensor
        }
        create = switcher.get(sensor)
        sensor = create(sensorType, ioPin)
        return sensor

    
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
        except (AM2320ReadError, ValueError, OSError) as e:
            log.error("Could not read AM2320 sensor %s", e)
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
        except (AM2320ReadError, ValueError, OSError) as e:
            log.error("Could not read AM2320 sensor %s", e)
            pass
        return humidity 

class bsht30TemperatureSensor(object):
    """Read temperature from bsht30 temperature sensor."""    

    def __init__(self, sensor):
        self._sensor = sensor

    def value_at(self, time_ts):
        temperature = None
        
        try:
            temperature = self._sensor.temperature
        except OSError as e:
            log.error("Could not read bsht30 sensor %s", e)
            pass
        return temperature 

class bsht30HumiditySensor(object):
    """Read temperature from bsht30 humidity sensor."""    

    def __init__(self, sensor):
        self._sensor = sensor

    def value_at(self, time_ts):
        humidity = None
        
        try:
            humidity = self._sensor.relative_humidity
        except OSError as e:
            log.error("Could not read bsht30 sensor %s", e)
            pass
        return humidity

class ft020tHumiditySensor(object):
    """Read humidity from ft020t weather sensor.  Note: this
    device is wireless and hardware control is via the switchdoc
    labs driver for the RTL433 using a compatble software define
    radio.""" 

    def __init__(self, sdr):
        self._sdr = sdr

    def value_at(self, time_ts):
        humidity = None
        try:
            humidity = self._sdr.LastKnownSensorInfo["SwitchDoc Labs FT020T AIO"]["humidity"]

            
            if (humidity > 100.0):
                humidity = None # error skip sample
            
        except:
            log.error("Could not read ft020t sensor %s")
            pass
        return humidity

class f016thHumiditySensor(object):
    """Read humidity from inside wireless sensor.  Note: this
    device is wireless and hardware control is via the switchdoc
    labs driver for the RTL433 using a compatble software define
    radio.""" 

    def __init__(self, sdr):
        self._sdr = sdr

    def value_at(self, time_ts):
        humidity = None
        try:
            humidity = self._sdr.LastKnownSensorInfo["SwitchDoc Labs F016TH Thermo-Hygrometer"]["humidity"]
            
            if (humidity > 100.0):
                humidity = None # error skip sample
        except:
            log.error("Could not read f016th sensor %s")
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
        temperature = None
        try:
            temperature = self._sensor.temperature
        except OSError as e:
            log.error("Could not read bmp280 sensor %s", e)
            pass
        return temperature

class ft020tTemperatureSensor(object):
    """Read temperature from ft020t weather sensor.  Note: this
    device is wireless and hardware control is via the switchdoc
    labs driver for the RTL433 using a compatble software define
    radio.""" 

    def __init__(self, sdr):
        self._sdr = sdr

    def value_at(self, time_ts):
        temperature = None
        try:
            temperature = self._sdr.LastKnownSensorInfo["SwitchDoc Labs FT020T AIO"]["temperature"]

            temperature = (temperature - 400) / 10.0
            if (temperature > 140.0):
                temperature = None # error skip sample

            # convert to Celcius
            temperature = round(((temperature - 32.0)/(9.0/5.0)),2)
        except:
            log.error("Could not read ft020t sensor %s")
            pass
        return temperature

class f016thTemperatureSensor(object):
    """Read temperature from inside wireless sensor.  Note: this
    device is wireless and hardware control is via the switchdoc
    labs driver for the RTL433 using a compatble software define
    radio.""" 

    def __init__(self, sdr):
        self._sdr = sdr

    def value_at(self, time_ts):
        temperature = None
        try:
            temperature = self._sdr.LastKnownSensorInfo["SwitchDoc Labs F016TH Thermo-Hygrometer"]["temperature_F"]
            
            if (temperature > 140.0):
                temperature = None # error skip sample

            # convert to Celcius
            temperature = round(((temperature - 32.0)/(9.0/5.0)),2)
        except:
            log.error("Could not read f016th sensor %s")
            pass
        return temperature

class softwareDefinedRadio(object):
    """Read the sensors from a wireless 433 MHz device.  This
    currently supports the switchdoc 433 MHz radio driver and
    chipsets recomended by switch doc labs.  It runs the
    external process for reading the radio provided by those
    drivers.""" 

    def __init__(self):        
        self.cmd = [ '/usr/local/bin/rtl_433', '-q', '-F', 'json', '-R', '146', '-R', '147']
        self._pulse = 0
        self._lastTimeSensorReceived = 0
        self._timeSinceLastSample = 0
        self.LastKnownSensorInfo = {}
        self._p = None
        self._q = None
        self._t = None

    def enqueueOutput(self, src, out, queue):
        try:
            for line in iter(out.readline, b''):
                queue.put((src, line))
            out.close()
        except:
            pass 

    def readSensors(self):
        from subprocess import PIPE, Popen, STDOUT
        from threading  import Thread
        from queue import Queue, Empty
        ON_POSIX = 'posix' in sys.builtin_module_names

        log.debug("")
        log.debug("######")
        #   Create our sub-process...
        #   Note that we need to either ignore output from STDERR or merge it with STDOUT due to a limitation/bug somewhere under the covers of "subprocess"
        #   > this took awhile to figure out a reliable approach for handling it...

        self._p = Popen(self.cmd, stdout=PIPE, stderr=STDOUT, bufsize=1, close_fds=ON_POSIX)
        self._q = Queue()

        self._t = Thread(target=self.enqueueOutput, args=('stdout', self._p.stdout, self._q))
    
        self._t.daemon = True # thread dies with the program
        self._t.start()

        self._pulse = 0
        log.debug("starting 433MHz scanning")
        log.debug("######")
        self._lastTimeSensorReceived = time.time()

        t2 = Thread(target=self.dequeueOutput, args=())
        t2.daemon = True # thread dies with the program
        t2.start()

    def dequeueOutput(self):
        from subprocess import PIPE, Popen, STDOUT
        from threading  import Thread
        from queue import Queue, Empty
        ON_POSIX = 'posix' in sys.builtin_module_names

        while True:

            #   Other processing can occur here as needed...
            #sys.stdout.write('Made it to processing step. \n')
            self._timeSinceLastSample = time.time() - self._lastTimeSensorReceived

            if (self._timeSinceLastSample > 720.0):   # restart if no reads in 12 minutes
                log.debug(">>>>>>>>>>>>>>restarting SDR thread.....")
                self._lastTimeSensorReceived = time.time()
                
                log.debug("Killing SDR Thread")
                self._p.kill()
                self._t.join()

                # Wait for 10 seconds: Allow previous
                # instance proper time to shutdown and
                # free resources.
                time.sleep(10)

                log.debug("starting SDR Thread again")
                log.debug("")
                log.debug("######")
                log.debug("Read Wireless Sensors")
                log.debug("######")

                self._p = Popen(self.cmd, stdout=PIPE, stderr=STDOUT, bufsize=1, close_fds=ON_POSIX)
                self._q = Queue()

                self._t = Thread(target=self.enqueueOutput, args=('stdout', self._p.stdout, self._q))
    
                self._t.daemon = True # thread dies with the program
                self._t.start()
            
            try:
                src, line = self._q.get(timeout = 1)
                log.debug(line.decode())
            except Empty:
                self._pulse += 1
            else: # got line
                self._pulse -= 1
                sLine = line.decode()

                self._lastTimeSensorReceived = time.time()

                try:
                    jsonLine = json.loads(sLine)
                except:
                    # only some of the lines are json
                    # for now we only care about json lines
                    continue

                self.LastKnownSensorInfo[jsonLine["model"]] = jsonLine
        
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
        altitude = None
        try:
            altitude = self._sensor.altitude
        except OSError as e:
            log.error("Could not read bmp280 sensor %s", e)
            pass
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
        pressure = None
        try:
            pressure = self._sensor.pressure
        except OSError as e:
            log.error("Could not read bmp280 sensor %s", e)
            pass
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
        if debounce > 0.04 :
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
        windRate.sort()
        gustIndex = self.__percentile_index(windRate, 0.75)
        sen08942Anemometer._windGustRate = windRate[gustIndex]

        speedStartIndex = self.__percentile_index(windRate, 0.25)
        speedEndIndex = gustIndex

        speedSum = 0.0
        if speedStartIndex == speedEndIndex :
            speedSum = windRate[speedStartIndex]
        else :
            for j in range(speedStartIndex, speedEndIndex):
                speedSum += windRate[j]   

        avgRate = speedSum / (speedEndIndex - speedStartIndex + 1)
        windSpeed = (2.4 / avgRate) # 2.4 km / h = 0.667 meters / second
        return windSpeed

    def __percentile_index(self, N, P):
        """
        Find the percentile of a list of values

        @parameter N - A list of values.  N must be sorted.
        @parameter P - A float value from 0.0 to 1.0

        @return - The index where the percentile lives.
        """
        n = int(round(P * len(N) + 0.5))
        return n - 1

class ft020tAnemometer(object):
    """Read wind speed from ft020t weather sensor.  Note: this
    device is wireless and hardware control is via the switchdoc
    labs driver for the RTL433 using a compatble software defined
    radio.""" 

    def __init__(self, sdr):
        self._sdr = sdr

    def value_at(self, time_ts):
        windSpeed = None
        try:
            windSpeed  = self._sdr.LastKnownSensorInfo["SwitchDoc Labs FT020T AIO"]["avewindspeed"]
            windSpeed = windSpeed / 10.0 # return m/s
        except:
            log.error("Could not read ft020t sensor %s")
            pass
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

class ft020tGustAnemometer(object):
    """Read wind gust speed from ft020t weather sensor.  Note: this
    device is wireless and hardware control is via the switchdoc
    labs driver for the RTL433 using a compatble software defined
    radio.""" 

    def __init__(self, sdr):
        self._sdr = sdr

    def value_at(self, time_ts):
        windGustSpeed = None
        try:
            windGustSpeed  = self._sdr.LastKnownSensorInfo["SwitchDoc Labs FT020T AIO"]["gustwindspeed"]
            windGustSpeed = windGustSpeed / 10.0 # return m/s
        except:
            log.error("Could not read ft020t sensor %s")
            pass
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
        if debounce > 0.5 :
            sen08942RainSensor._rainTick += 1
            sen08942RainSensor._lastRainSample = currentRainSample

    def value_at(self, time_ts):
        rainTicks = sen08942RainSensor._rainTick
        sen08942RainSensor._rainTick = 0.0
        rainAmount = rainTicks * 0.02794 # 1 tick = 0.2794 mm or 0.02794
        return rainAmount

class ft020tRainSensor(object):
    """Read rain amount from ft020t weather sensor.  Note: this
    device is wireless and hardware control is via the switchdoc
    labs driver for the RTL433 using a compatble software define
    radio.""" 

    def __init__(self, sdr):
        self._sdr = sdr
        self._lastRain = None

    def value_at(self, time_ts):
        totalRain = None
        rainDelta = None
        try:
            # this device is cumulative rain whereas weewx
            # reports rain since last measurement.  Here we
            # get the delta since last measurement. The
            # weewx helper function handles first rain measurent
            # and other error conditions properly so I used
            # that function.
            totalRain = self._sdr.LastKnownSensorInfo["SwitchDoc Labs FT020T AIO"]["cumulativerain"]    
            totalRain = round(totalRain / 100.0, 2) # return cm 

            rainDelta = weewx.wxformulas.calculate_rain(totalRain, self._lastRain)
            self._lastRain = totalRain
        except:
            log.error("Could not read ft020t sensor %s")
            pass
        return rainDelta

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
        resultPerCent = None
        try:
            proc = subprocess.Popen(["iwconfig", "wlan0"],stdout=subprocess.PIPE, universal_newlines=True)
            out, err = proc.communicate()

            m = re.search("Link Quality=(?P<strength>[0-9][0-9])/(?P<max_strength>[0-9][0-9])", out)

            strength = int(m.group('strength'))
            max_strength = int(m.group('max_strength'))
            resultPerCent = strength / max_strength * 100
        except:

            # failed signal strength should not cause program stop.
            pass
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
