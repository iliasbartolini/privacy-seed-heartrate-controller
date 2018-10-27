# Privacy Seed - Heartrate controller

Heartrate controller based on Arduino Nano and MAX30101

The hardware interface uses two custom made shield boards:
- Voltage regulator and I2C voltage converter (TODO: add KiCAD project link)
- Sensor board (TODO: add KiCAD project link)

## Wiring:
-  I2C SDA (purple):  28
-  I2C SCL (grey):    27

## TODO
- Improve 'beat' detection
- Read data also from green LED light (currently using only IR light)
- Improve rate estimation (using simple AVG) and prediction
