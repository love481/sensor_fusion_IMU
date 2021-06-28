/******************
*Author:Love panta
*Edited
****************/
#pragma Once
#ifndef _IMU_H_
#define _IMU_H_
#include<stdio.h>
#include "I2Cdev.h"
#include"IMU_config.h"
int buffersize=1000; 
int acel_deadzone=16384;     
int gyro_deadzone=250;  
int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
static const uint16_t lsb_per_g[4] = { 16384, 8192, 4096, 2048 };
static const float lsb_per_dsp[4] = { 131.0, 65.5, 32.8, 16.4 };
struct mag_data
{
int16_t mx, my, mz;
};
struct accel_data
{
int16_t ax, ay, az;
};
struct gyro_data
{
int16_t gx, gy, gz;
};
struct mpu_data{
     
     accel_data a;
     mag_data m;
     gyro_data g;
     int16_t temp;
};
class IMU
{ private:
     uint8_t hmcAddr;
     uint8_t mpuAddr;
     uint8_t mpuBuffer[14];
     uint8_t hmcBuffer[6];
     uint8_t mode;
     mpu_data data;
     uint16_t offsets[10];
     
  public:
     IMU(){
         hmcAddr = HMC5883L_DEFAULT_ADDRESS;
         mpuAddr=MPU6050_DEFAULT_ADDRESS;
         setAccelXOffsets(0);
         setAccelYOffsets(0);
         setAccelZOffsets(0);
         setGyroXOffsets(0);
         setGyroYOffsets(0);
         setGyroZOffsets(0);
         setMagXOffsets(0);
         setMagYOffsets(0);
         setMagZOffsets(0);
         setTemOffsets(0);
     }
     IMU(uint8_t hmcAddr,uint8_t mpuAddr) {
         this->hmcAddr = HMC5883L_DEFAULT_ADDRESS;
         this->mpuAddr=MPU6050_DEFAULT_ADDRESS;
        }
     void initialize()
     {
        // write CONFIG_A register
        I2Cdev::writeByte(hmcAddr, HMC5883L_RA_CONFIG_A,
        (HMC5883L_AVERAGING_8 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1)) |
        (HMC5883L_RATE_15     << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)) |
        (HMC5883L_BIAS_NORMAL << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1)));

        I2Cdev::writeByte(hmcAddr, HMC5883L_RA_CONFIG_B, HMC5883L_GAIN_1090 << (HMC5883L_CRB_GAIN_BIT - HMC5883L_CRB_GAIN_LENGTH + 1));
        I2Cdev::writeByte(hmcAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
        mode = HMC5883L_MODE_SINGLE; // track to tell if we have to clear bit 7 after a read

        I2Cdev::writeBits(mpuAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH,MPU6050_CLOCK_PLL_XGYRO);

        setFullScaleGyroRange(MPU6050_GYRO_FS_250);
        setFullScaleAccelRange(MPU6050_ACCEL_FS_2); 
        setSleepEnabled(false); 
     }
    uint8_t getFullScaleAccelRange() {
        I2Cdev::readBits(mpuAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, mpuBuffer);
        return mpuBuffer[0];

    }
    void setFullScaleAccelRange(uint8_t range) {
    I2Cdev::writeBits(mpuAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
                    }
    uint8_t getFullScaleGyroRange() {
    I2Cdev::readBits(mpuAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, mpuBuffer);
    return mpuBuffer[0];
    }
    void setFullScaleGyroRange(uint8_t range) {
    I2Cdev::writeBits(mpuAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
    }
    void setSleepEnabled(bool enabled) {
    I2Cdev::writeBit(mpuAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
    }
    bool testConnection() {
    bool hmcState=false;
    if (I2Cdev::readBytes(hmcAddr, HMC5883L_RA_ID_A, 3, hmcBuffer) == 3) {
        hmcState= (hmcBuffer[0] == 'H' && hmcBuffer[1] == '4' && hmcBuffer[2] == '3');
    }
    bool mpuState=(getmpuDeviceID() == 0x34);
    return mpuState & hmcState;
    }
    uint8_t getmpuDeviceID() {
    I2Cdev::readBits(mpuAddr, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH,mpuBuffer );
    return mpuBuffer[0];
    }
    mag_data getRawMag() {
    I2Cdev::readBytes(hmcAddr, HMC5883L_RA_DATAX_H, 6,hmcBuffer);//start reading data from the given address
    if (mode == HMC5883L_MODE_SINGLE) I2Cdev::writeByte(hmcAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
    data.m.mx= (((int16_t)hmcBuffer[0]) << 8) | hmcBuffer[1];
    data.m.my = (((int16_t)hmcBuffer[4]) << 8) | hmcBuffer[5];
    data.m.mz = (((int16_t)hmcBuffer[2]) << 8) | hmcBuffer[3];
    return data.m;
    }
    accel_data getRawAccel(){
       I2Cdev::readBytes(mpuAddr, MPU6050_RA_ACCEL_XOUT_H, 6,mpuBuffer);
       data.a.ax = (((int16_t)mpuBuffer[0]) << 8) | mpuBuffer[1];
       data.a.ay = (((int16_t)mpuBuffer[2]) << 8) | mpuBuffer[3];
       data.a.az = (((int16_t)mpuBuffer[4]) << 8) | mpuBuffer[5];
       return data.a;
    }
    accel_data getNormAccel(){
      struct accel_data d;
      d.ax=data.a.ax/16384;
      d.ay=data.a.ay/16384;
      d.az=data.a.az/16384;
      return d;
    }
    uint16_t getTemp() {
    I2Cdev::readBytes(mpuAddr, MPU6050_RA_TEMP_OUT_H, 2, mpuBuffer);
    data.temp=(((int16_t)mpuBuffer[0]) << 8) | mpuBuffer[1];
    return data.temp;
    }
    gyro_data getRawGyro() {
    I2Cdev::readBytes(mpuAddr, MPU6050_RA_GYRO_XOUT_H, 6, mpuBuffer);
    data.g.gx = (((int16_t)mpuBuffer[0]) << 8) | mpuBuffer[1];
    data.g.gy = (((int16_t)mpuBuffer[2]) << 8) | mpuBuffer[3];
    data.g.gz = (((int16_t)mpuBuffer[4]) << 8) | mpuBuffer[5];
    return data.g;
    }
    gyro_data getNormGyro(){
      struct gyro_data d;
      d.gx=data.g.gx/131.0;
      d.gy=data.g.gy/131.0;
      d.gz=data.g.gz/131.0;
      return d;
    }
    void setAccelXOffsets(uint16_t o){offsets[0]=o;}
    void setAccelYOffsets(uint16_t o){offsets[1]=o;}
    void setAccelZOffsets(uint16_t o){offsets[2]=o;}
    void setGyroXOffsets(uint16_t o){offsets[3]=o;}
    void setGyroYOffsets(uint16_t o){offsets[4]=o;}
    void setGyroZOffsets(uint16_t o){offsets[5]=o;}
    void setMagXOffsets(uint16_t o){offsets[6]=o;}
    void setMagYOffsets(uint16_t o){offsets[7]=o;}
    void setMagZOffsets(uint16_t o){offsets[8]=o;}
    void setTemOffsets(uint16_t o){offsets[9]=o;}
    uint16_t getAccelXOffsets(){return offsets[0];}
    uint16_t getAccelYOffsets(){return offsets[1];}
    uint16_t getAccelZOffsets(){return offsets[2];}
    uint16_t getGyroXOffsets(){return offsets[3];}
    uint16_t getGyroYOffsets(){return offsets[4];}
    uint16_t getGyroZOffsets(){return offsets[5];}
    uint16_t getMagXOffsets(){return offsets[6];}
    uint16_t getMagYOffsets(){return offsets[7];}
    uint16_t getMagZOffsets(){return offsets[8];}
    uint16_t getTemOffsets(){return offsets[9];}
    void meanAccel(){
         long i=0,buff_ax=0,buff_ay=0,buff_az=0;
         accel_data a;
         while (i<(buffersize+101)){
    // read raw accel measurements from device
    a=getRawAccel();
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+a.ax;
      buff_ay=buff_ay+a.ay;
      buff_az=buff_az+a.az;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
    }
    i++;
    delay(2); 
     }
    }
    void meanGyro(){
         long i=0,buff_gx=0,buff_gy=0,buff_gz=0;
         gyro_data g;
         while (i<(buffersize+101)){
    // read raw accel measurements from device
      g=getRawGyro();
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_gx=buff_gx+g.gx;
      buff_gy=buff_gy+g.gy;
      buff_gz=buff_gz+g.gz;
    }
    if (i==(buffersize+100)){
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); 
     }
    }

    void calibrate_accel(int t)
    {
      
       offsets[0]=-mean_ax/8;
       offsets[1]=-mean_ay/8;
       offsets[2]=(16384-mean_az)/8;
     while (1){
      int ready=0;
    setAccelXOffsets( offsets[0]);
    setAccelYOffsets( offsets[1]);
    setAccelZOffsets( offsets[2]);
    meanAccel();
    Serial.println("...");

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else offsets[0]=offsets[0]-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else offsets[1]=offsets[1]-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else offsets[2]=offsets[2]+(16384-mean_az)/acel_deadzone;
    if (ready==t) break;
  }
      
    }
    
    void calibrate_gyro(int t)
    {

     offsets[3]=-mean_gx/4;
     offsets[4]=-mean_gy/16;
     offsets[5]=-mean_gz/4;
     while (1){
    int ready=0;
    setGyroXOffsets( offsets[3]);
    setGyroYOffsets( offsets[4]);
    setGyroZOffsets( offsets[5]);

    meanGyro();
    Serial.println("...");

    if (abs(mean_gx)<=gyro_deadzone) ready++;
    else offsets[3]=offsets[3]-mean_gx/(gyro_deadzone+1);

    if (abs(mean_gy)<=gyro_deadzone) ready++;
    else offsets[4]=offsets[4]-mean_gy/(gyro_deadzone+1);

    if (abs(mean_gz)<=gyro_deadzone) ready++;
    else offsets[5]=offsets[5]-mean_gz/(gyro_deadzone+1);

    if (ready==t) break;
  }
      
    }
     void calibrate_magnetometer(int t)
     {
       mag_data m;
       int Step=0;
       long min_x,min_y,min_z,max_x,max_y,max_z;
       m=getRawMag();
       min_x=max_x=m.mx;
       min_y=max_y=m.my;
       min_z=max_z=m.mz;
       while(Step<t)
       { m=getRawMag();
        min_x = min(min_x, m.mx);
       min_y = min(min_y, m.my);
       min_z = min(min_z, m.mz);

       max_x = max(max_x, m.mx);
       max_y = max(max_y, m.my);
       max_z = max(max_z, m.mz);

       offsets[6] = (max_x + min_x) / 2;
       offsets[7]  = (max_y + min_y) / 2;
       offsets[8]  = (max_z + min_z) / 2;
       Step++;
       }
      
     }

};
#endif // !_IMU_H_
