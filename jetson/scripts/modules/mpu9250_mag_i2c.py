#########################################
# Copyright (c) 2020 Maker Portal LLC
# Author: Joshua Hrisko
#########################################
#
# This code handles the smbus 
# communications between the RPi and the
# MPU9250 IMU. For testing the MPU9250
# see: imu_test.py
#
#########################################
#
import smbus,time


def AK8963_start():
    bus.write_byte_data(AK8963_ADDR,AK8963_CNTL,0x00)
    time.sleep(0.1)
    bus.write_byte_data(AK8963_ADDR,AK8963_CNTL,0x0F)
    time.sleep(0.1)
    coeff_data = bus.read_i2c_block_data(AK8963_ADDR,AK8963_ASAX,3)
    AK8963_coeffx = (0.5*(coeff_data[0]-128)) / 256.0 + 1.0
    AK8963_coeffy = (0.5*(coeff_data[1]-128)) / 256.0 + 1.0
    AK8963_coeffz = (0.5*(coeff_data[2]-128)) / 256.0 + 1.0
    time.sleep(0.1)
    bus.write_byte_data(AK8963_ADDR,AK8963_CNTL,0x00)
    time.sleep(0.1)
    AK8963_bit_res = 0b0001 # 0b0001 = 16-bit
    AK8963_samp_rate = 0b0110 # 0b0010 = 8 Hz, 0b0110 = 100 Hz
    AK8963_mode = (AK8963_bit_res <<4)+AK8963_samp_rate # bit conversion
    bus.write_byte_data(AK8963_ADDR,AK8963_CNTL,AK8963_mode)
    time.sleep(0.1)
    return [AK8963_coeffx,AK8963_coeffy,AK8963_coeffz] 
    
def AK8963_reader(register):
    # read magnetometer values
    low = bus.read_byte_data(AK8963_ADDR, register-1)
    high = bus.read_byte_data(AK8963_ADDR, register)
    # combine higha and low for unsigned bit value
    value = ((high << 8) | low)
    # convert to +- value
    if(value > 32768):
        value -= 65536
    
    return value

def AK8963_conv():
    # raw magnetometer bits
    while 1:
##        if ((bus.read_byte_data(AK8963_ADDR,AK8963_ST1) & 0x01))!=1:
##            return 0,0,0
        mag_x = AK8963_reader(HXH)
        mag_y = AK8963_reader(HYH)
        mag_z = AK8963_reader(HZH)

        # the next line is needed for AK8963
        if (bus.read_byte_data(AK8963_ADDR,AK8963_ST2)) & 0x08!=0x08:
            break
        
    #convert to acceleration in g and gyro dps
##    m_x = AK8963_coeffs[0]*(mag_x/(2.0**15.0))*mag_sens
##    m_y = AK8963_coeffs[1]*(mag_y/(2.0**15.0))*mag_sens
##    m_z = AK8963_coeffs[2]*(mag_z/(2.0**15.0))*mag_sens
    m_x = (mag_x/(2.0**15.0))*mag_sens
    m_y = (mag_y/(2.0**15.0))*mag_sens
    m_z = (mag_z/(2.0**15.0))*mag_sens
    return m_x,m_y,m_z

def calibrate_mag(seconds=60):
    print("Starting calibration")
    bias ={'x': 0, 'y': 0, 'z': 0}
    scale = {'x': 1, 'y': 1, 'z': 1}
    starting = time.time()
    min_values = [1000, 1000, 1000]
    max_values = [-1000, -1000, -1000]
    while time.time() - starting < seconds:
        geomagnetic = AK8963_conv()
        if min_values[0] > geomagnetic[0]:
            min_values[0] = geomagnetic[0]
        if max_values[0] < geomagnetic[0]:
            max_values[0] = geomagnetic[0]
        if min_values[1] > geomagnetic[1]:
            min_values[1] = geomagnetic[1]
        if max_values[1] < geomagnetic[1]:
            max_values[1] = geomagnetic[1]
        if min_values[2] > geomagnetic[2]:
            min_values[2] = geomagnetic[2]
        if max_values[2] < geomagnetic[2]:
            max_values[2] = geomagnetic[2]
    # print("Min:", min_values)
    # print("Max:", max_values)
    
    bias['x'] = (max_values[0] + min_values[0]) / 2
    bias['y'] = (max_values[1] + min_values[1]) / 2
    bias['z'] = (max_values[2] + min_values[2]) / 2

    delta_axis = [(max_values[0] - min_values[0]) / 2,
                (max_values[1] - min_values[1]) / 2,
                (max_values[2] - min_values[2]) / 2]

    delta_avg = sum(delta_axis) / 3

    scale['x'] = delta_avg / delta_axis[0]
    scale['y'] = delta_avg / delta_axis[1]
    scale['z'] = delta_avg / delta_axis[2]
    return bias, scale
    
# MPU6050 Registers
MPU6050_ADDR = 0x68
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_PIN_CFG  = 0x37
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
TEMP_OUT_H   = 0x41
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
#AK8963 registers
AK8963_ADDR   = 0x0C
AK8963_ST1    = 0x02
HXH          = 0x04
HYH          = 0x06
HZH          = 0x08
AK8963_ST1   = 0x02
AK8963_ST2   = 0x09
AK8963_CNTL  = 0x0A
AK8963_ASAX = 0x10

mag_sens = 4800.0 # magnetometer sensitivity: 4800 uT

# start I2C driver

try:
    bus = smbus.SMBus(1) # start comm with i2c bus
except OSError:
    print("OSError")    
time.sleep(1)
AK8963_coeffs = AK8963_start() # instantiate magnetometer
time.sleep(0.5)
# print(calibrate_mag(60))