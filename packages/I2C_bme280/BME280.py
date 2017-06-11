#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# BME280 driver for Linux (e.g. Fischertechnik TXT, Raspberry PI, Orange PI and every host
# that supports the i2c-dev module.
#
# This library is inspired and heavily based on the library of the
# same name by Andreas Schroeder: https://github.com/motlib/bme280
#
# Copyright (c) 2017 Christian Hehr
#
# Distributed under the terms of the MIT license. See LICENSE file for
# details.


'''BME280 sensor driver access by Linux i2c-dev driver.

You can import this module into your own application and instanciate
the BME280 class or you can run this module as a standalone
application. Then it will read the sensor once and print the result to
the console.

See the end of this file for an example of how to use the BME280
class.

'''

from collections import namedtuple
from struct import unpack
from datetime import datetime

# Register addresses
BME280_REG_CAL1 = 0x88
BME280_REG_ID = 0xD0
BME280_REG_CAL2 = 0xE1
BME280_REG_CTRL_HUM = 0xF2
BME280_REG_STATUS = 0xF3
BME280_REG_CTRL_MEAS = 0xF4
BME280_REG_RAWDATA = 0xF7

# Chip Id value
BME280_CHIP_ID = 0x60

# Oversampling values for T, P and H
BME280_OSRS_1 = 1
BME280_OSRS_2 = 2
BME280_OSRS_4 = 3
BME280_OSRS_8 = 4
BME280_OSRS_16 = 5

# Mode register values
BME280_MODE_SLEEP = 0
BME280_MODE_FORCED = 1
BME280_MODE_NORMAL = 3

# Raw sensor values register set length
BME280_RAWDATA_LEN = 8

BME280_CAL1_LEN = 26
BME280_CAL2_LEN = 7

# Default I2C address
BME280_DEF_I2C_ADDR = 0x76

# Status register bit masks
BME280_STATUS_IM_UPD_MASK = 0x01
BME280_STATUS_MEAS_MASK = 0x08

# According to datasheet with 16x oversampling for all values, the
# maximum sampling time is 112.8ms. So add a bit to be on the save
# side.
BME280_MEAS_TIMEOUT = 0.200

# Result data structure
BME280Result = namedtuple(
    'BME280Result',
    'temperature humidity pressure')

# Calibration data structure
BME280CalParams = namedtuple(
    'BME280CalParams',
    ('dig_T1 dig_T2 dig_T3 dig_P1 dig_P2 dig_P3 '
     'dig_P4 dig_P5 dig_P6 dig_P7 dig_P8 dig_P9 '
     'dig_H1 dig_H2 dig_H3 dig_H4 dig_H5 dig_H6'))

# Raw sensor data structure
BME280RawData = namedtuple(
    'BME280RawData',
    't h p')


class BME280(object):
    '''Control the Bosch Sensortec BME280 sensor.'''

    def __init__(self, bus, address=BME280_DEF_I2C_ADDR):
        '''Initialize and read the calibration data from the sensor.

        '''

        self.bus = bus
        self.address = address

        # check for correct chip id
        chip_id = self._read_chip_id()
        if chip_id != BME280_CHIP_ID:
            msg = 'Invalid chip id detected. Read {0}, expected {1}.'

            raise Exception(msg.format(chip_id, BME280_CHIP_ID))

        self.cal = self._read_calibration_params()


    def _calc_tfine(self, cal, raw):
        '''Calculate the t_fine value based on the raw sensor value and the
        calibration data.

        '''

        v1 = (raw.t / 16384.0 - cal.dig_T1 / 1024.0) * cal.dig_T2
        v2 = ((raw.t / 131072.0 - cal.dig_T1 / 8192.0) ** 2) * cal.dig_T3

        return v1 + v2


    def _calc_humidity(self, cal, raw, t_fine):
        '''Calculate the humidity based on the raw sensor values and the
        calibration data.

        '''

        res = t_fine - 76800.0

        res = (raw.h - (cal.dig_H4 * 64.0 + cal.dig_H5 / 16384.0 * res)) * \
            (cal.dig_H2 / 65536.0 * (1.0 + cal.dig_H6 / 67108864.0 * res *
                                            (1.0 + cal.dig_H3 / 67108864.0 * res)))
        res = res * (1.0 - (cal.dig_H1 * res / 524288.0))

        return max(0.0, min(res, 100.0))


    def _calc_pressure(self, cal, raw, t_fine):
        '''Calculate the pressure based on the raw sensor values and the
        calibration data.

        '''

        v1 = t_fine / 2.0 - 64000.0
        v2 = v1 * v1 * cal.dig_P6 / 32768.0
        v2 = v2 + v1 * cal.dig_P5 * 2.0
        v2 = v2 / 4.0 + cal.dig_P4 * 65536.0
        v1 = (cal.dig_P3 * v1 * v1 / 524288.0 + cal.dig_P2 * v1) / 524288.0
        v1 = (1.0 + v1 / 32768.0) * cal.dig_P1

        # Prevent divide by zero
        if v1 == 0:
            return 0

        res = 1048576.0 - raw.p
        res = ((res - v2 / 4096.0) * 6250.0) / v1
        v1 = cal.dig_P9 * res * res / 2147483648.0
        v2 = res * cal.dig_P8 / 32768.0
        res = res + (v1 + v2 + cal.dig_P7) / 16.0

        return (res / 100.0)


    def _calc_temp(self, t_fine):
        '''Calculate the temperature based on t_fine.'''

        return t_fine / 5120


    def _calc_result(self, cal, raw):
        '''Calulate the temperature, humidity and pressure based on the raw
        sensor readings and the calibration values.

        '''

        t_fine = self._calc_tfine(self.cal, raw)

        res = BME280Result(
            temperature=self._calc_temp(t_fine),
            humidity=self._calc_humidity(self.cal, raw, t_fine),
            pressure=self._calc_pressure(self.cal, raw, t_fine))

        return res


    def _read_calibration_params(self):
        '''Read the calibration data from the sensor.'''

        # Read data block 1, unsigned and signed short values and an
        # unsigned byte.
        blk1 = self.bus.read_i2c_block_data(
            self.address,
            BME280_REG_CAL1,
            BME280_CAL1_LEN);
        dta1 = unpack("<HhhHhhhhhhhhxB", bytearray(blk1))

        # Read data block 2, five unsigned bytes
        blk2 = self.bus.read_i2c_block_data(
            self.address,
            BME280_REG_CAL2,
            BME280_CAL2_LEN);
        dta2 = unpack("<hBBBBB", bytearray(blk2))

        cal = BME280CalParams(
            dig_T1 = dta1[0],
            dig_T2 = dta1[1],
            dig_T3 = dta1[2],
            dig_P1 = dta1[3],
            dig_P2 = dta1[4],
            dig_P3 = dta1[5],
            dig_P4 = dta1[6],
            dig_P5 = dta1[7],
            dig_P6 = dta1[8],
            dig_P7 = dta1[9],
            dig_P8 = dta1[10],
            dig_P9 = dta1[11],
            dig_H1 = dta1[12],
            dig_H2 = dta2[0],
            dig_H3 = dta2[1],
            dig_H4 = ((dta2[2] << 4) | (dta2[3] & 0x0F)),
            dig_H5 = (((dta2[3] & 0xF0) << 12) | dta2[4]),
            dig_H6 = dta2[5],
        )

        return cal


    def _read_chip_id(self):
        chip_id = self.bus.read_byte_data(
                self.address,
                BME280_REG_ID)

        return chip_id


    def _read_raw_data(self):
        '''Read the raw values from the sensor. To be called after a
        measurement has been completed.'''

        block = self.bus.read_i2c_block_data(
            self.address,
            BME280_REG_RAWDATA,
            BME280_RAWDATA_LEN)

        raw = BME280RawData(
            t=(block[3] << 16 | block[4] << 8 | block[5]) >> 4,
            h=block[6] << 8 | block[7],
            p=(block[0] << 16 | block[1] << 8 | block[2]) >> 4)

        return raw


    def _wait_meas(self):
        '''Wait for the measurement to be completed by polling the status
        register.'''
        # Maximum iterations to wait for sensor result
        max_it = 10000

        status = BME280_STATUS_MEAS_MASK

        start = datetime.now()

        while (status & BME280_STATUS_MEAS_MASK) != 0:
            status = self.bus.read_byte_data(
                self.address,
                BME280_REG_STATUS)

            # Check timeout
            convtime = (datetime.now() - start).total_seconds()
            if convtime > BME280_MEAS_TIMEOUT:
                raise Exception('Sensor conversion timeout')


    def _start_meas(self, os_t=BME280_OSRS_8, os_p=BME280_OSRS_8, os_h=BME280_OSRS_8):
        '''Start the measurement.

        The sensor is set to forced mode to do one convesion. '''

        mode = BME280_MODE_FORCED

        # Configure sensor
        self.bus.write_byte_data(
            self.address,
            BME280_REG_CTRL_HUM,
            os_h)

        self.bus.write_byte_data(
            self.address,
            BME280_REG_CTRL_MEAS,
            os_t << 5 | os_p << 2 | mode)




    def sample(self, os=BME280_OSRS_16):
        '''Trigger a measurement and return the result.

        Returns a data structure with temperature (degree celsius),
        humitidy (percent relative humidity) and
        pressure (hPa) members. '''

        self._start_meas(os_t=os, os_p=os, os_h=os)
        self._wait_meas()

        raw = self._read_raw_data()

        return self._calc_result(self.cal, raw)


def _parse_args():
    from argparse import ArgumentParser

    parser = ArgumentParser()
    parser.add_argument(
        '--bus',
        help='I2C Bus Number, set to 1 to access /dev/i2c-1.',
        default='1')
    parser.add_argument(
        '--addr',
        help='Sensor I2C address',
        default='0x76')
    parser.add_argument(
        '--os',
        help='Oversampling setting. 1=1x, 2=2x, 3=4x, 4=8x, 5=16x.',
        default=1)

    args =parser.parse_args()

    return args

if __name__ == '__main__':
    from smbus import SMBus

    # some command-line handling
    args = _parse_args()
    bus_number = int(args.bus)
    i2c_addr = int(args.addr, 0)
    os = min(5, max(args.os, 0))

    bus = SMBus(bus_number)
    sensor = BME280(bus, i2c_addr)
    res = sensor.sample(os)

    print("temperature={0}".format(res.temperature))
    print("pressure={0}".format(res.pressure))
    print("humidity={0}".format(res.humidity))
