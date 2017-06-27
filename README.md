# Pi_IMU
Python class for handling the Berry IMU. Currently only the Accelerometer, Gyro and
Magnetometer are supported.

To use this class in your project download the IMU.py file.

You will then need to import the class into your project using:
from IMU import IMU

The IMU object can then be created using:
imu = IMU()

Before using any of the sensors they must be activated, you can either use the default 
setup by calling imu.setup_default() or setup each register yourself by calling
imu.writeReg(address, register, value). If you do then you may want to import the values
in LSM9DS0 by calling: 
from LSM9DS0 import *  (see the file LSM9DS0.py for more the variable names)

Once setup you can read the sensors using the imu.readAccAxis, imu.readGyrAxis or 
imu.readMagAxis (which are used to get the value from a specific axis) or by calling 
imu.readAcc, imu.readGyr or imu.readMag

It is also possible to continuously read data from the sensors in the background and
save these to a file. To do that call: 
imu.take_measurements_process(freq, file_name, cut=None)

where freq is the frequency of measurements in Hz, file_name is the name of the file
where the measurements will be saved (a new directory called IMU_Data will also be
created) and cut is the amount of time the process will run for before switching to a
new file.
