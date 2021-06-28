/******************
*Author:Love Panta
*gmail:075bei016.love@pcampuse.edu.np
*created:2021/6/10
******************/
#include "Wire.h"
#include "IMU.h"
#include <ros.h>
#include<geometry_msgs/Vector3.h>
ros::NodeHandle  nh;
geometry_msgs::Vector3 accel;
geometry_msgs::Vector3 gyro;
geometry_msgs::Vector3 mag;
IMU  imu_;
long dt=0;
float yaw_g=0;//yaw obtained from gyro,may have drift
float off_y;
struct accel_data accel_data_;
struct mag_data mag_data_;
struct gyro_data gyro_data_;
ros::Publisher accel_pub("IMU/accel", &accel);
ros::Publisher gyro_pub("IMU/gyro", &gyro);
ros::Publisher mag_pub("IMU/mag", &mag);
float normalizeAngle(float dx)
{
  if (dx >= 0)
    dx = fmodf(dx + (float)PI, 2 * (float)PI) - (float)PI;
  else
    dx = fmodf(dx - (float)PI, 2 * (float)PI) + (float)PI;
  return dx;
}
float get_yaw_offsets()
{
  mag_data_=imu_.getRawMag();
  float yaw_offset= atan2(mag_data_.my,mag_data_.mx);
  if(yaw_offset < 0)
     yaw_offset+= 2 * M_PI;
  return yaw_offset;
}
void setup() {
    nh.initNode();
    nh.advertise(accel_pub);
    nh.advertise(gyro_pub);
    nh.advertise(mag_pub);
    Wire.begin();
    Serial.begin(57600);
    //initialize device
    Serial.println("Initializing I2C devices...");
    imu_.initialize();
    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(imu_.testConnection() ? "mpu connection successful" : "mpu connection failed");
    imu_.calibrate_magnetometer(1000);
    //delay(5000);
    imu_.calibrate_gyro(3);
    imu_.calibrate_accel(3);
    //off_y=get_yaw_offsets();
    dt=millis();
}
void get_heading_mag()
{
  float headingm = atan2(mag_data_.my,mag_data_.mx);
  headingm=normalizeAngle(headingm);
  Serial.print("headingm:\t");
  Serial.println(headingm*(180/M_PI));

  
}
void get_heading_gyro()
{
  yaw_g+=((millis()-dt))*(gyro_data_.gz/131000.0);
  dt=millis();
  yaw_g=normalizeAngle(yaw_g);
  Serial.print("headingg:\t");
  Serial.println(PI);
  
  
}
void loop() {
  // put your main code here, to run repeatedly:
  accel_data_=imu_.getRawAccel();
  accel_data_.ax+=imu_.getAccelXOffsets();
  accel_data_.ay+=imu_.getAccelYOffsets();
  accel_data_.az+=imu_.getAccelZOffsets();
  accel.x=accel_data_.ax;
  accel.y=accel_data_.ay;
  accel.z=accel_data_.az;
  Serial.print("accel:\t");
  Serial.print(accel_data_.ax); Serial.print("\t");
  Serial.print(accel_data_.ay); Serial.print("\t");
  Serial.println(accel_data_.az); Serial.println("\t");
  gyro_data_=imu_.getRawGyro();
  gyro_data_.gx+=imu_.getGyroXOffsets();
  gyro_data_.gy+=imu_.getGyroYOffsets();
  gyro_data_.gz+=imu_.getGyroZOffsets();
  gyro.x=gyro_data_.gx;
  gyro.y=gyro_data_.gy;
  gyro.z=gyro_data_.gz;
  Serial.print("gyro:\t");
  Serial.print(gyro_data_.gx); Serial.print("\t");
  Serial.print(gyro_data_.gy); Serial.print("\t");
  Serial.println(gyro_data_.gz); Serial.println("\t");
  mag_data_=imu_.getRawMag();
  /*mag_data_.mx-=imu_.getMagXOffsets();
  mag_data_.my-=imu_.getMagYOffsets();
  mag_data_.mz-=imu_.getMagZOffsets();*/
  mag_data_.mx+=9.3;//offsets based on the calibration values from matlab by plotting ellipsoid
  mag_data_.my-=106;
  //get_heading_mag();
  //Serial.println(sqrt(mag_data_.mx*mag_data_.mx+mag_data_.my*mag_data_.my+mag_data_.mz*mag_data_.mz));
  mag.x=mag_data_.mx;
  mag.y=mag_data_.my;
  mag.z=mag_data_.mz;
  Serial.print("magnetometer:\t");
  Serial.print(mag_data_.mx); Serial.print("\t");
  Serial.print(mag_data_.my); Serial.print("\t");
  Serial.print(mag_data_.mz); Serial.println("\t");
  accel_pub.publish(&accel);
  gyro_pub.publish(&gyro);
  mag_pub.publish(&mag);
  delay(100);
  nh.spinOnce();
  

}
