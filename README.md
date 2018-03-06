# Chempro-Sensor-Serial-Python
Program for accessing Chempro100i Chemical Gas Sensor measurement and passing the data to Mavlink Protocol. Developed using Python.

# Usage

```ssh
python Chempro.py
```

# Pre-requisites
Set the port for Chempro Sensor and Pixhawk 
 ```python
Sensor = Chempro100i("/dev/ttyUSB0",38400)
GPS = PixhawkMavlink("/dev/ttyACM0",57600)
```
