# ros_bno08x

A ROS node for communicating with the [BNO080](https://www.sparkfun.com/products/14686) or [BNO085](https://www.adafruit.com/product/4754) 9DOF IMU.

## Description

The node communicates with the BNO08x via i2c on the Raspberry Pi using Adafruit's Python library: https://github.com/adafruit/Adafruit_CircuitPython_BNO08x.  The i2c address is preset to `0x4b` for the BNO080.  This is configurable for the BNO085 on `0x4a`.  The data is stored in the following:

* Accelerometer and Gyroscope (sensor_msgs/Imu): `bno08x/raw`
* Magnometer (sensor_msgs/MagneticField): `bno08x/mag`
* Temperature (sensor_msgs/Temperature): `bno08x/temp`
* Diagnostics (diagnostic_msgs/DiagnosticStatus): `bno08x/status`

## Installation Instructions

* Enable i2c
  ```
  sudo apt-get install i2c-tools
  i2cdetect -l
  ```
* Add `i2c-devl` to boot with `sudo nano /etc/modules-load.d/modules.conf`
* Install wiringpi `sudo apt install wiringpi`
* Connect i2c devices to Sparkfun Qwiic hat and run `i2cdetect -y 1` to identify channels
* Install Circuit Python: https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/installing-circuitpython-on-raspberry-pi
* Install driver for BNO08x IMU: `sudo pip3 install adafruit-circuitpython-bno08x`.

## Running the Node

* Option 1: `rosrun ros_bno08x talker.py` 
* Option 2: `roslaunch launches/bno08x.launch`
  
## Tested Setup

It should work on other versions but Python 3 is a requirement.

* Platform: Raspberry Pi 4
* OS: Ubuntu MATE 20.04
* ROS: Noetic
* Python: 3.8.5

