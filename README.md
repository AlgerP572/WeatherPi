# WeatherPi
A Weewx 4.x python 3.x driver for using a Raspberry Pi 4B as a weather station.

This python driver is designed for use with the Weewx 4.0 weather station software.
As of this writing Weewx 4.x is still under development and can be found here:

http://weewx.com/downloads/development_versions/

Currently this driver is known to work with Weewx 4.x versions:

Weewx version | Branch or Label | Notes
------------ | ------------- | -------------
4.0.0b14 | Weewxb14
4.0.0b16 | Weewxb16
4.0.0b17 | Weewxb17 (master) | Some users may also require Weewx changset d077a8b
4.0.0b18 | Weewxb18 (master) | This version includes the above changset.

Special thanks to the Weewx team for fast turn around time on an issue encountered by some users when attempting to use the new console logging mechanism. Details are on github. [issue 519](https://github.com/weewx/weewx/issues/519)

![WeatherPi](media/Screen.png)

## Contents
- [Supported Hardware](#supported-hardware)
- [Installation](#installation)
- [Required Libraies](#required-libraies)
- [The Hardware](#the-hardware)
- [Compiling Weewx](#compiling-weewx)

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
sen08942 (Sparkfun weather station)
sen15901 (Sparkfun weather station)
0015-WR-DSBT/SF (WeatherRack Weather SwitchDoc Labs) 
```

## Wind Vanes
```
sen08942 (Sparkfun weather station)
sen15901 (Sparkfun weather station)
0015-WR-DSBT/SF (WeatherRack Weather SwitchDoc Labs) 
```

## Rain Guage
```
sen08942 (Sparkfun weather station)
sen15901 (Sparkfun weather station)
0015-WR-DSBT/SF (WeatherRack Weather SwitchDoc Labs) 
```

## Altimeter
```
BMP280
```

## WiFi Signal Strength
```
Raspberry Pi 3B
Raspberry Pi 4B

(just uses iwconfig so in theory any Pi model with WiFi should work.)
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

# Required Libraies

This driver requires the ADA Fruit Circuit python libraries.  Installation instructions can be readily found on the Web with a google search for ADAfruit circuit python BMP280 etc.

It also requires the general purpose IO libray by Ben Croston.

# The Hardware

![WeatherPi](media/IMG_7013.png)

![WeatherPi](media/IMG_7015.png)

![WeatherPi](media/IMG_7017.png)

# Compiling Weewx

Compiling Weewx is very straightforwad with visual studio code which is free.  It is also lightweight enough to run happily directly on the Raspberry Pi. Steps are:

1) Download and install VS code for linux
2) Install the python add on extension
3) Open the code.
4) If you want to use integrated debugging you will need to create a custom .json file.  Here is an example:
```
{
    // Use IntelliSense to learn about possible attributes.
    // Hover to view descriptions of existing attributes.
    // For more information, visit: https://go.microsoft.com/fwlink/?linkid=830387
    "version": "0.2.0",
    "configurations": [       
    
        {
            "name": "Python: Current File",
            "type": "python",
            "request": "launch",
            "program": "${workspaceFolder}/bin/weewxd",
            "args": ["weewx.conf"],
            "console": "integratedTerminal"
        }
    ]
}
```
![WeatherPi](media/VSCodeWeewx.png)
