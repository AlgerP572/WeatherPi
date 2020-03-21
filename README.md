# WeatherPi
A Weewx 4.0 python 3.0 driver for Raspberry Pi 4B weather station.

This python driver is designed for use with the Weewx 4.0 weather station software.
As of this writing Weewx 4.0 is still under development and can be found here:

http://weewx.com/downloads/development_versions/

Currently this driver is known to work with Weewx 4.0 versions:
4.0.0b14, 4.0.0b16

![WeatherPi](media/Screen.png)

# Supported Hardware
The following list describes the hardware that this driver currently supports:

## Temperature Sensors
```
BMP280
AM2315
```

## Humidity Sensors
```
BMP280
AM2315
```

## Pressure Sensors
```
BMP280
```

## Anemometer
```
sen08942 (Spark fun weather station)
sen15901 (Spark fun weather station)
```

## Wind Vanes
```
sen08942 (Spark fun weather station)
sen15901 (Spark fun weather station)
```

## Rain Guage
```
sen08942 (Spark fun weather station)
sen15901 (Spark fun weather station)
```

## Altimeter
```
BMP280
```

# Installation

If you have issues installing this driver via the weewx script it can alternatively be installed manually.  The process to install the driver manually is outlined below:

1) Copy the WeatherPi.py file to the weewx/bin/weewx/drivers/ folder

2) Modify the weewx/weewx.conf file to point the the WeartherPi.py driver:
    a) Change sation_type in the [Station] config section to station_type = WeatherPi
    b) Add a new [WeatherPi] config section to the conf file and fill it with default values.
```
        [WeatherPi]

          # The time (in seconds) between LOOP packets. Default is:
          loop_interval = 10

          # Debug level - the level of message logging. The higher
          # the number, the more info is logged.
          debug_read = 4

          # Used to calibrate the wind direction of the weather vane to
          # to real world coordinates in case it is off some.
          wind_direction_offset_angle = -93.0

          # The driver to use:
          driver = weewx.drivers.WeatherPi
```
          
3) Update the observations section to reflect.  This driver follows the design pattern of the Simulator.py driver.  As such it is easy to change the hardware used for each observation.  Just pass in a new class that defines a value_at() function. See the list of currently support hardware above.
```
      self.observations = {
          'outTemp'    : am2320TemperatureSensor(self.am2315),
          'inTemp'     : bmp280TemperatureSensor(self.bmp280),
          'altimeter'  : bmp280AltitudeSensor(self.bmp280),
          'barometer'  : bmp280BarometerSensor(self.bmp280),
          'pressure'   : noneSensor(),
          ...
```

