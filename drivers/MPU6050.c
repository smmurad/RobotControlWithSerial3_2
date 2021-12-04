/************************************************************************/
// File:            MPU6050.h                                           //
// Author:                                                              //
// Purpose:         IMU driver                                          //
//                                                                      //
/************************************************************************/

#include "MPU6050.h"
#include "i2c.h"

/* 
This driver could be expanded to cover all the uses of the MPU however this would be of little use since we lack an interupt line from the imu
TODO: USE 3a INT STATUS TO CHECK IF WE GOT New data-> we have to use polling due to no interupt
*/

static volatile uint8_t IMU_COPY[14]; //storage array for data from imu

uint8_t MPU_read_8bit(uint8_t reg){
    uint8_t ret;
    i2c_recive(MPU_ADDR, reg, &ret, 1);
    return ret;
}

void MPU_read_burst(uint8_t reg, uint8_t* storage, uint8_t len8bit){
    i2c_recive(MPU_ADDR, reg, storage, len8bit);
}

void MPU_write(uint8_t reg, uint8_t value){
    i2c_rend(MPU_ADDR, reg, &value, 1);
}

void MPU_write_bit(uint8_t reg, uint8_t bit, uint8_t data){
     uint8_t b;
     b = MPU_read_8bit(reg);
     b = (data != 0) ? (b | (1 << bit)) : (b & ~(1 << bit));
     MPU_write(reg,b);
}

void MPU_write_bits(uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data){
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    b = MPU_read_8bit(reg);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= data; // combine data with existing byte
    MPU_write(reg, b);
}


void IMU_init(void){
    //set clock souce to xgyro
    MPU_write_bits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);
    //set accel range to 2g
    MPU_write_bits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS_2);
    //set gyro scale to 250 degrees /s
    MPU_write_bits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS_250);
    //enable data ready interupt
    MPU_write_bit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, true);
    //disable sleep
    MPU_write_bit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT,false);
    //set GYRO SAMPLE RATE
    MPU_write(MPU6050_RA_SMPLRT_DIV, 24);// max 1khz actuall samplerate  = 1khz /(rate+1)   1 or 8 depends on dlpf for gyro
    //set low pass filter 
    MPU_write_bits(MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH,3);
    //set High pass filter dhpf not documented in datasheet
   // MPU_write_bits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ACCEL_HPF_BIT, MPU6050_ACONFIG_ACCEL_HPF_LENGTH, bandwidth);
}

int IMU_newData(void){
     uint8_t data = MPU_read_8bit(MPU6050_RA_INT_STATUS);//bit 0 is set on new data, automaticaly cleared when reading register
     return (data<<7);
}


float GyroValueToDegrees(int16_t measure){
      const float scaling = 250.0/32767.0;  // analog signal is at its maximum 16 bits (32767) at 250 degrees max value of signal
      return (scaling*measure);
}
      
float AccValueToG(int16_t measure){
      const float scaling = 2.0/3276.70;
      return scaling*measure;
}      

//due to no interupt line the gyro needs to be used in burst read to get data from same frame

void IMU_read(void){
      MPU_read_burst(MPU6050_RA_ACCEL_XOUT_H ,(uint8_t*) &IMU_COPY,14);
}

int IMU_getTemp(void){
      int16_t result;
      result = ((int16_t)IMU_COPY[6]<<8)|IMU_COPY[7];
      return (int)(((float)result / 340)+ 36.53); //returns temprature in degrees C
}   

float fIMU_readFloatAccelX(void){
    int16_t temp;
    temp = ((int16_t)IMU_COPY[0]<<8) | IMU_COPY[1];
    return AccValueToG(temp);
}
float fIMU_readFloatAccelY(void){
    int16_t temp;
    temp = ((int16_t)IMU_COPY[2]<<8) | IMU_COPY[3];
    return AccValueToG(temp);
}
float fIMU_readFloatAccelZ(void){
    int16_t temp;
    temp = ((int16_t)IMU_COPY[4]<<8) | IMU_COPY[5];
    return AccValueToG(temp);
}

float fIMU_readFloatGyroX(void){
    int16_t temp;
    temp = ((int16_t)IMU_COPY[8]<<8) | IMU_COPY[9];
    return GyroValueToDegrees(temp);
}
float fIMU_readFloatGyroY(void){
    int16_t temp;
    temp = ((int16_t)IMU_COPY[10]<<8) | IMU_COPY[11];
    return GyroValueToDegrees(temp);
}
float fIMU_readFloatGyroZ(void){
    int16_t temp;
    temp = ((int16_t)IMU_COPY[12]<<8) | IMU_COPY[13];
    return GyroValueToDegrees(temp);						
}

IMU_reading_t IMU_getGyro(void){
  IMU_reading_t gyro;
  gyro.x = fIMU_readFloatGyroX();
  gyro.y = fIMU_readFloatGyroY();
  gyro.z = fIMU_readFloatGyroZ();
  return gyro;
}

IMU_reading_t IMU_getAccel(void){
    IMU_reading_t accel;
    accel.x = fIMU_readFloatAccelX();
    accel.y = fIMU_readFloatAccelY();
    accel.z = fIMU_readFloatAccelZ();
    return accel;
}
