"""
Program for controlling the IMU connected to Pi_1
"""

import smbus
from LSM9DS0 import *
import multiprocessing
import time
from pathlib import Path
import os


def ensure_dir(file_path):
    '''Create a directory if it doesn't exist'''
    if not file_path.exists():
        file_path.mkdir(parents=True)


class SensorInactiveError(Exception):
    '''Exception raised when attempt to access inactive sensor

    Attributes:
        sensor -- the sensor to which access was attempted
        message -- explanation of the error
    '''
    def __init__(self, sensor):
        self.sensor = sensor
        self.message = ('Attempted to read data from the {} failed due to'
                        ' inactive sensor').format(sensor)


class IMU():
    '''
    Class for controlling the BerryIMU gyroscope, accelerometer and
    magnetometer.

    Before reading data each sensor must be setup either with default values
    by calling self.setup_default or with any other values desired by calling
    the self.writeReg command. For more details on the various options
    available see the documentation at:
        http://ozzmaker.com/wp-content/uploads/2014/12/LSM9DS0.pdf
    The registers of importance are:
        CTRL_REG1_XM and CTRL_REG2_XM for the accelerometer
        CTRL_REG5_XM, CTRL_REG6_XM and CTRL_REG7_XM for the magnetometer
        CTRL_REG1_G and CTRL_REG2_G for the gyro

    Values can be read from each axis individually via the readAccAxis/
    readGryAxis/readMagAxis respectively with the values 0,1 and 2
    representing the x, y and z axis respectively.
    Values can also be read from all three axes simultaneously using the
    corresponding readAcc, readGry and ReadMag commands
    '''
    def __enter__(self):
        self.__init__()

    def __init__(self):
        '''Setup the bus for the IMU'''
        self.bus = smbus.SMBus(1)
        self._processes = []
        self._flags = []
        self._acc_active = False
        self._gyr_active = False
        self._mag_active = False
        self.save_path = Path(os.path.dirname(__file__)) / 'IMU_Data'
        ensure_dir(self.save_path)

    def __exit__(self):
        '''Reset all registers and close the bus'''
        self.reset_registers()
        self.bus.close()

    def reset_registers(self):
        '''
        Reset all registers for the accelerometer, gyro and magnetometer to
        zero
        '''
        self.writeReg(ACC_ADDRESS, CTRL_REG1_XM, 0)
        self.writeReg(ACC_ADDRESS, CTRL_REG2_XM, 0)
        self.writeReg(MAG_ADDRESS, CTRL_REG5_XM, 0)
        self.writeReg(MAG_ADDRESS, CTRL_REG6_XM, 0)
        self.writeReg(MAG_ADDRESS, CTRL_REG7_XM, 0)
        self.writeReg(GYR_ADDRESS, CTRL_REG1_G, 0)
        self.writeReg(GYR_ADDRESS, CTRL_REG4_G, 0)
        self._acc_active = False
        self._gyr_active = False
        self._mag_active = False

    def setup_default(self):
        '''
        Setup the default values to activate the gyro, magnetometer and
        accelerometer
        '''
        # Initialise accelerometer
        self.writeReg(ACC_ADDRESS, CTRL_REG1_XM, 0b01100111)
        self.writeReg(ACC_ADDRESS, CTRL_REG2_XM, 0b00100000)
        # Initialise the magnetometer
        self.writeReg(MAG_ADDRESS, CTRL_REG5_XM, 0b11110000)
        self.writeReg(MAG_ADDRESS, CTRL_REG6_XM, 0b01100000)
        self.writeReg(MAG_ADDRESS, CTRL_REG7_XM, 0b00000000)
        # Initialise the gyroscope
        self.writeReg(GYR_ADDRESS, CTRL_REG1_G, 0b00001111)
        self.writeReg(GYR_ADDRESS, CTRL_REG4_G, 0b00110000)
        self._acc_active = True
        self._mag_active = True
        self._gyr_active = True

    def writeReg(self, address, register, value):
        '''Used to write values to various addresses for setting up the IMU'''
        self.bus.write_byte_data(address, register, value)
        return -1

    def readAccAxis(self, axis):
        '''Axis should be 0,1 or 2 (0=>x,1=>y,2=>z)'''
        # Check which axis we are using to make measurements
        if self._acc_active == False:
            raise SensorInactiveError('Accelerometer')
        if axis == 0:
            register_l = OUT_X_L_A
            register_h = OUT_X_H_A
        elif axis == 1:
            register_l = OUT_Y_L_A
            register_h = OUT_Y_H_A
        elif axis == 2:
            register_l = OUT_Z_L_A
            register_h = OUT_Z_H_A
        else:
            raise ValueError(
                    'Expected axis to be 0,1 or 2 corresponding to x,y,z')
        # Get the values from the register
        acc_l = self.bus.read_byte_data(ACC_ADDRESS, register_l)
        acc_h = self.bus.read_byte_data(ACC_ADDRESS, register_h)
        acc_combined = (acc_l | acc_h << 8)
        # Return the acceleration value (accounting for positive and negative acceleration)
        return acc_combined if acc_combined < 32768 else acc_combined - 65536

    def readMagAxis(self, axis):
        '''Axis should be 0,1 or 2 (0=>x,1=>y,2=>z)'''
        if self._mag_active == False:
            raise SensorInactiveError('Accelerometer')
        # Check which axis we are using to make measurements
        if axis == 0:
            register_l = OUT_X_L_M
            register_h = OUT_X_H_M
        elif axis == 1:
            register_l = OUT_Y_L_M
            register_h = OUT_Y_H_M
        elif axis == 2:
            register_l = OUT_Z_L_M
            register_h = OUT_Z_H_M
        else:
            raise ValueError(
                    'Expected axis to be 0,1 or 2 corresponding to x,y,z')
        # Get the values from the register
        mag_l = self.bus.read_byte_data(MAG_ADDRESS, register_l)
        mag_h = self.bus.read_byte_data(MAG_ADDRESS, register_h)
        mag_combined = (mag_l | mag_h << 8)
        # Return the acceleration value (accounting for positive and negative acceleration)
        return mag_combined if mag_combined < 32768 else mag_combined - 65536

    def readGyrAxis(self, axis):
        '''Axis should be 0,1 or 2 (0=>x,1=>y,2=>z)'''
        # Check which axis we are using to make measurements
        if self._acc_active == False:
            raise SensorInactiveError('Accelerometer')
        if axis == 0:
            register_l = OUT_X_L_G
            register_h = OUT_X_H_G
        elif axis == 1:
            register_l = OUT_Y_L_G
            register_h = OUT_Y_H_G
        elif axis == 2:
            register_l = OUT_Z_L_G
            register_h = OUT_Z_H_G
        else:
            raise ValueError(
                    'Expected axis to be 0,1 or 2 corresponding to x,y,z')
        # Get the values from the register
        gyr_l = self.bus.read_byte_data(GYR_ADDRESS, register_l)
        gyr_h = self.bus.read_byte_data(GYR_ADDRESS, register_h)
        gyr_combined = (gyr_l | gyr_h << 8)
        # Return the acceleration value
        return gyr_combined if gyr_combined < 32768 else gyr_combined - 65536

    def readAcc(self):
        '''
        Reads all data from the accelerometer and returns the results as a
        dictionary with the keys x, y and z
        '''
        return {'x': self.readAccAxis(0),
                'y': self.readAccAxis(1),
                'z': self.readAccAxis(2)}

    def readGyr(self):
        '''
        Reads all data from the gyro and returns the results as a
        dictionary with the keys x, y and z
        '''
        return {'x': self.readGyrAxis(0),
                'y': self.readGyrAxis(1),
                'z': self.readGyrAxis(2)}

    def readMag(self):
        '''
        Reads all data from the accelerometer and returns the results as a
        dictionary with the keys x, y and z
        '''
        return {'x': self.readMagAxis(0),
                'y': self.readMagAxis(1),
                'z': self.readMagAxis(2)}

    def take_measurements_process(self, freq, file_name, cut=None):
        '''
        Generates a python process for taking measurements with the IMU.

        Returns a pipe for recieving values from the IMU in the form
        [acc, mag, gyr] where acc, mag and gyr are dictionaries containing the
        keys 'x', 'y' and 'z'
        '''
        exit_flag = multiprocessing.Value('i', 0)
        # Creates a queue where data will be stored (max size of 100)
        pipe_1, pipe_2 = multiprocessing.Pipe()
        p = multiprocessing.Process(target=self._take_measurements,
                                    args=(freq, file_name, cut, exit_flag, pipe_1))
        self._processes.append(p)
        self._flags.append(exit_flag)
        p.start()
        # Return the pipe so user can access values if desired
        return pipe_2

    def end_measurements_processes(self):
        '''
        End all active processes taking measurements
        '''
        for flag in self._flags:
            with flag.get_lock():
                flag.value = 1

        for i, process in enumerate(self._processes):
            process.join()
            print('IMU Process {} joined'.format(i))

        return

    def _take_measurements(self, freq, file_name, cut, exit_flag, pipe):
        '''
        Reads from all activated sensors at the specified frequency and saves
        to the location in the given file name.

        If cut is set, a new file will be written to every n seconds (where n
        is denoted by the vaiable cut)
        '''
        try:
            with exit_flag.get_lock():
                local_flag = exit_flag.value

            while local_flag == 0:
                if cut is not None:
                    n = 0
                    # Creates a new file every cut seconds
                    for i in range(0, cut*freq):
                        file_path = self.save_path / '{}_{}.txt'.format(file_name, n)
                        n += 1
                        with open(str(file_path), 'w') as file:
                            # Take lots of measurements
                            acc = self.readAcc()
                            mag = self.readMag()
                            gyr = self.readGyr()
                            file_output = ('Acc: {} {} {} Gyr: {} {} {} Mag: '
                                           '{} {} {}\n').format(
                                               acc['x'], acc['y'], acc['z'],
                                               gyr['x'], gyr['y'], gyr['z'],
                                               mag['x'], mag['y'], mag['z'])
                            file.write(file_output)
                            pipe.send([acc, mag, gyr])
                            with exit_flag.get_lock():
                                local_flag = exit_flag.value
                            time.sleep(1/freq)
            pipe.close()
        except SensorInactiveError as e:
            print(e.message)
