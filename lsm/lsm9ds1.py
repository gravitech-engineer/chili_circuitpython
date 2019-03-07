import time
try:
    import struct
except ImportError:
    import ustruct as struct

import adafruit_bus_device.i2c_device as i2c_device
import adafruit_bus_device.spi_device as spi_device
#https://learn.adafruit.com/circuitpython-essentials/circuitpython-expectations#how-can-i-create-my-own-mpy-files-18-6

_LSM9DS1_ADDRESS_ACCELGYRO       = const(0x6A)
_LSM9DS1_ADDRESS_MAG             = const(0x1C)
_LSM9DS1_XG_ID                   = const(0b01101000)
_LSM9DS1_MAG_ID                  = const(0b00111101)
_LSM9DS1_ACCEL_MG_LSB_2G         = 0.061
_LSM9DS1_ACCEL_MG_LSB_4G         = 0.122
_LSM9DS1_ACCEL_MG_LSB_8G         = 0.244
_LSM9DS1_ACCEL_MG_LSB_16G        = 0.732
_LSM9DS1_MAG_MGAUSS_4GAUSS       = 0.14
_LSM9DS1_MAG_MGAUSS_8GAUSS       = 0.29
_LSM9DS1_MAG_MGAUSS_12GAUSS      = 0.43
_LSM9DS1_MAG_MGAUSS_16GAUSS      = 0.58
_LSM9DS1_GYRO_DPS_DIGIT_245DPS   = 0.00875
_LSM9DS1_GYRO_DPS_DIGIT_500DPS   = 0.01750
_LSM9DS1_GYRO_DPS_DIGIT_2000DPS  = 0.07000
_LSM9DS1_TEMP_LSB_DEGREE_CELSIUS = 8 
_LSM9DS1_REGISTER_WHO_AM_I_XG    = const(0x0F)
_LSM9DS1_REGISTER_CTRL_REG1_G    = const(0x10)
_LSM9DS1_REGISTER_CTRL_REG2_G    = const(0x11)
_LSM9DS1_REGISTER_CTRL_REG3_G    = const(0x12)
_LSM9DS1_REGISTER_TEMP_OUT_L     = const(0x15)
_LSM9DS1_REGISTER_TEMP_OUT_H     = const(0x16)
_LSM9DS1_REGISTER_STATUS_REG     = const(0x17)
_LSM9DS1_REGISTER_OUT_X_L_G      = const(0x18)
_LSM9DS1_REGISTER_OUT_X_H_G      = const(0x19)
_LSM9DS1_REGISTER_OUT_Y_L_G      = const(0x1A)
_LSM9DS1_REGISTER_OUT_Y_H_G      = const(0x1B)
_LSM9DS1_REGISTER_OUT_Z_L_G      = const(0x1C)
_LSM9DS1_REGISTER_OUT_Z_H_G      = const(0x1D)
_LSM9DS1_REGISTER_CTRL_REG4      = const(0x1E)
_LSM9DS1_REGISTER_CTRL_REG5_XL   = const(0x1F)
_LSM9DS1_REGISTER_CTRL_REG6_XL   = const(0x20)
_LSM9DS1_REGISTER_CTRL_REG7_XL   = const(0x21)
_LSM9DS1_REGISTER_CTRL_REG8      = const(0x22)
_LSM9DS1_REGISTER_CTRL_REG9      = const(0x23)
_LSM9DS1_REGISTER_CTRL_REG10     = const(0x24)
_LSM9DS1_REGISTER_OUT_X_L_XL     = const(0x28)
_LSM9DS1_REGISTER_OUT_X_H_XL     = const(0x29)
_LSM9DS1_REGISTER_OUT_Y_L_XL     = const(0x2A)
_LSM9DS1_REGISTER_OUT_Y_H_XL     = const(0x2B)
_LSM9DS1_REGISTER_OUT_Z_L_XL     = const(0x2C)
_LSM9DS1_REGISTER_OUT_Z_H_XL     = const(0x2D)
_LSM9DS1_REGISTER_WHO_AM_I_M     = const(0x0F)
_LSM9DS1_REGISTER_CTRL_REG1_M    = const(0x20)
_LSM9DS1_REGISTER_CTRL_REG2_M    = const(0x21)
_LSM9DS1_REGISTER_CTRL_REG3_M    = const(0x22)
_LSM9DS1_REGISTER_CTRL_REG4_M    = const(0x23)
_LSM9DS1_REGISTER_CTRL_REG5_M    = const(0x24)
_LSM9DS1_REGISTER_STATUS_REG_M   = const(0x27)
_LSM9DS1_REGISTER_OUT_X_L_M      = const(0x28)
_LSM9DS1_REGISTER_OUT_X_H_M      = const(0x29)
_LSM9DS1_REGISTER_OUT_Y_L_M      = const(0x2A)
_LSM9DS1_REGISTER_OUT_Y_H_M      = const(0x2B)
_LSM9DS1_REGISTER_OUT_Z_L_M      = const(0x2C)
_LSM9DS1_REGISTER_OUT_Z_H_M      = const(0x2D)
_LSM9DS1_REGISTER_CFG_M          = const(0x30)
_LSM9DS1_REGISTER_INT_SRC_M      = const(0x31)
_MAGTYPE                         = True
_XGTYPE                          = False
_SENSORS_GRAVITY_STANDARD        = 9.80665

ACCELRANGE_2G                = (0b00 << 3)
ACCELRANGE_16G               = (0b01 << 3)
ACCELRANGE_4G                = (0b10 << 3)
ACCELRANGE_8G                = (0b11 << 3)
MAGGAIN_4GAUSS               = (0b00 << 5)  
MAGGAIN_8GAUSS               = (0b01 << 5)  
MAGGAIN_12GAUSS              = (0b10 << 5)  
MAGGAIN_16GAUSS              = (0b11 << 5)  
GYROSCALE_245DPS             = (0b00 << 3)  
GYROSCALE_500DPS             = (0b01 << 3)  
GYROSCALE_2000DPS            = (0b11 << 3)  


def _twos_comp(val, bits):
    
    if val & (1 << (bits - 1)) != 0:
        return val - (1 << bits)
    return val

class LSM9DS1:
    _BUFFER = bytearray(6)

    def __init__(self):
        self._write_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG8, 0x05)
        self._write_u8(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG2_M, 0x0C)
        time.sleep(0.01)
        if self._read_u8(_XGTYPE, _LSM9DS1_REGISTER_WHO_AM_I_XG) != _LSM9DS1_XG_ID or \
           self._read_u8(_MAGTYPE, _LSM9DS1_REGISTER_WHO_AM_I_M) != _LSM9DS1_MAG_ID:
            raise RuntimeError('Could not find LSM9DS1, check wiring!')
        self._write_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG1_G, 0xC0) 
        self._write_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG5_XL, 0x38)
        self._write_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG6_XL, 0xC0)
        self._write_u8(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG3_M, 0x00)
        self._accel_mg_lsb = None
        self._mag_mgauss_lsb = None
        self._gyro_dps_digit = None
        self.accel_range = ACCELRANGE_2G
        self.mag_gain = MAGGAIN_4GAUSS
        self.gyro_scale = GYROSCALE_245DPS

    @property
    def accel_range(self):
        
        reg = self._read_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG6_XL)
        return (reg & 0b00011000) & 0xFF

    @accel_range.setter
    def accel_range(self, val):
        assert val in (ACCELRANGE_2G, ACCELRANGE_4G, ACCELRANGE_8G,
                       ACCELRANGE_16G)
        reg = self._read_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG6_XL)
        reg = (reg & ~(0b00011000)) & 0xFF
        reg |= val
        self._write_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG6_XL, reg)
        if val == ACCELRANGE_2G:
            self._accel_mg_lsb = _LSM9DS1_ACCEL_MG_LSB_2G
        elif val == ACCELRANGE_4G:
            self._accel_mg_lsb = _LSM9DS1_ACCEL_MG_LSB_4G
        elif val == ACCELRANGE_8G:
            self._accel_mg_lsb = _LSM9DS1_ACCEL_MG_LSB_8G
        elif val == ACCELRANGE_16G:
            self._accel_mg_lsb = _LSM9DS1_ACCEL_MG_LSB_16G

    def read_accel_raw(self):
        self._read_bytes(_XGTYPE, 0x80 | _LSM9DS1_REGISTER_OUT_X_L_XL, 6,
                         self._BUFFER)
        raw_x, raw_y, raw_z = struct.unpack_from('<hhh', self._BUFFER[0:6])
        return (raw_x, raw_y, raw_z)
  
    @property
    def acceleration(self):
        raw = self.read_accel_raw()
        return map(lambda x: x * self._accel_mg_lsb / 1000.0 * _SENSORS_GRAVITY_STANDARD,raw)

    def mag_gain(self):
        
        reg = self._read_u8(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG2_M)
        return (reg & 0b01100000) & 0xFF

    @mag_gain.setter
    def mag_gain(self, val):
        assert val in (MAGGAIN_4GAUSS, MAGGAIN_8GAUSS, MAGGAIN_12GAUSS,
                       MAGGAIN_16GAUSS)
        reg = self._read_u8(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG2_M)
        reg = (reg & ~(0b01100000)) & 0xFF
        reg |= val
        self._write_u8(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG2_M, reg)
        if val == MAGGAIN_4GAUSS:
            self._mag_mgauss_lsb = _LSM9DS1_MAG_MGAUSS_4GAUSS
        elif val == MAGGAIN_8GAUSS:
            self._mag_mgauss_lsb = _LSM9DS1_MAG_MGAUSS_8GAUSS
        elif val == MAGGAIN_12GAUSS:
            self._mag_mgauss_lsb = _LSM9DS1_MAG_MGAUSS_12GAUSS
        elif val == MAGGAIN_16GAUSS:
            self._mag_mgauss_lsb = _LSM9DS1_MAG_MGAUSS_16GAUSS
    #@property
    def read_mag_raw(self):
        self._read_bytes(_MAGTYPE, 0x80 | _LSM9DS1_REGISTER_OUT_X_L_M, 6,self._BUFFER)
        raw_x, raw_y, raw_z = struct.unpack_from('<hhh', self._BUFFER[0:6])
        return (raw_x, raw_y, raw_z)

    @property
    def magnetic(self):
        raw = self.read_mag_raw()
        return map(lambda x: x * self._mag_mgauss_lsb / 1000.0, raw)

    @property
    def gyro_scale(self):
        
        reg = self._read_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG1_G)
        return (reg & 0b00011000) & 0xFF

    @gyro_scale.setter
    def gyro_scale(self, val):
        assert val in (GYROSCALE_245DPS, GYROSCALE_500DPS, GYROSCALE_2000DPS)
        reg = self._read_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG1_G)
        reg = (reg & ~(0b00011000)) & 0xFF
        reg |= val
        self._write_u8(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG1_G, reg)
        if val == GYROSCALE_245DPS:
            self._gyro_dps_digit = _LSM9DS1_GYRO_DPS_DIGIT_245DPS
        elif val == GYROSCALE_500DPS:
            self._gyro_dps_digit = _LSM9DS1_GYRO_DPS_DIGIT_500DPS
        elif val == GYROSCALE_2000DPS:
            self._gyro_dps_digit = _LSM9DS1_GYRO_DPS_DIGIT_2000DPS


    def read_gyro_raw(self):
        self._read_bytes(_XGTYPE, 0x80 | _LSM9DS1_REGISTER_OUT_X_L_G, 6,
                         self._BUFFER)
        raw_x, raw_y, raw_z = struct.unpack_from('<hhh', self._BUFFER[0:6])
        return (raw_x, raw_y, raw_z)

    @property
    def gyro(self):
        raw = self.read_gyro_raw()
        return math.map(lambda x: x * self._gyro_dps_digit, raw)

    def read_temp_raw(self):
        self._read_bytes(_XGTYPE, 0x80 | _LSM9DS1_REGISTER_TEMP_OUT_L, 2,
                         self._BUFFER)
        temp = ((self._BUFFER[1] << 8) | self._BUFFER[0]) >> 4
        return _twos_comp(temp, 12)

    @property
    def temperature(self):
        temp = self.read_temp_raw()
        temp = 27.5 + temp/16
        return temp

    def _read_u8(self, sensor_type, address):
        raise NotImplementedError()

    def _read_bytes(self, sensor_type, address, count, buf):
        raise NotImplementedError()

    def _write_u8(self, sensor_type, address, val):
        raise NotImplementedError()



class LSM9DS1_I2C(LSM9DS1):
    def __init__(self, i2c):
        self._mag_device = i2c_device.I2CDevice(i2c, _LSM9DS1_ADDRESS_MAG)
        self._xg_device = i2c_device.I2CDevice(i2c, _LSM9DS1_ADDRESS_ACCELGYRO)
        super().__init__()

    def _read_u8(self, sensor_type, address):
        if sensor_type == _MAGTYPE:
            device = self._mag_device
        else:
            device = self._xg_device
        with device as i2c:
            self._BUFFER[0] = address & 0xFF
            i2c.write(self._BUFFER, end=1, stop=False)
            i2c.readinto(self._BUFFER, end=1)
        return self._BUFFER[0]

    def _read_bytes(self, sensor_type, address, count, buf):
        if sensor_type == _MAGTYPE:
            device = self._mag_device
        else:
            device = self._xg_device
        with device as i2c:
            buf[0] = address & 0xFF
            i2c.write(buf, end=1, stop=False)
            i2c.readinto(buf, end=count)

    def _write_u8(self, sensor_type, address, val):
        if sensor_type == _MAGTYPE:
            device = self._mag_device
        else:
            device = self._xg_device
        with device as i2c:
            self._BUFFER[0] = address & 0xFF
            self._BUFFER[1] = val & 0xFF
            i2c.write(self._BUFFER, end=2)

