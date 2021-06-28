#include"filters_algorithms/imu_sensor_fusion.h"
#include<ros/ros.h>
#include"filters_algorithms/linalg.h"
#include<geometry_msgs/Vector3.h>
#include<std_msgs/Float32.h>
kalman imu[3];
float A_roll,A_pitch,M_yaw;//obtained from acceleration value
float G_roll,G_pitch,G_yaw=0;//obtained from gyro value by integrating
float Mx,My;
ros::Time prevTime_kalman;
std::vector<float>state_r(2);
std::vector<float>state_p(2);
std::vector<float>state_y(2);
bool first_entered_mag=false;
bool first_entered_accel=false;
float r_offset=0;
float p_offset=0;
float y_offset=0;
void accel_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{

  //ROS_INFO(" accelerometer: %f  %f  %f\n",msg->x,msg->y,msg->z);
  A_roll=atan2(msg->y,msg->z);
  A_pitch=linalg::atan2(-msg->x,sqrt(msg->y*msg->y+msg->z*msg->z));
  if(first_entered_accel==false)
    {
      state_r[0]=A_roll*(180/M_PI);
      state_r[1]=0.1;
      r_offset=state_r[1];
      state_p[0]=A_pitch*(180/M_PI);
      state_p[1]=0.1;
      p_offset=state_p[1];
      first_entered_accel=true;
    ROS_INFO("first enetered_accel \n");
  }
}
void gyro_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  //ROS_INFO(" Gyro: %f  %f  %f\n",msg->x,msg->y,msg->z);
  G_roll=(msg->x)/131;
  G_pitch=(msg->y)/131;
  G_yaw=(msg->z)/131;
}
void mag_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  //ROS_INFO(" magnetometer: %f  %f  %f\n",msg->x,msg->y,msg->z);
  Mx=msg->x*cos(A_pitch)+msg->z*sin(A_pitch);
  My=msg->x*sin(A_roll)*sin(A_pitch)+msg->y*cos(A_roll)-msg->z*sin(A_roll)*cos(A_pitch);
  M_yaw=atan2(-My, Mx);
  /*if(M_yaw<0)
     M_yaw+=2*M_PI;
   if(M_yaw>0)
     M_yaw-=2*M_PI;*/
  //ROS_INFO("yaw: %f\n",M_yaw*(180/M_PI));
  if(first_entered_mag==false)
    {
      state_y[0]=M_yaw*(180/M_PI);
      y_offset=state_y[0];
      state_y[1]=0.1;
      first_entered_mag=true;
    ROS_INFO("first enetered_mag \n");
    }
}
int main(int argc, char **argv)
{
ros::init(argc, argv, "imu_node");
ros::NodeHandle nh;
ros::Publisher roll_pub=nh.advertise<std_msgs::Float32>("IMU/roll",1000);
ros::Publisher pitch_pub=nh.advertise<std_msgs::Float32>("IMU/pitch",1000);
ros::Publisher yaw_pub=nh.advertise<std_msgs::Float32>("IMU/yaw",1000);
ros::Publisher raw_yaw_pub=nh.advertise<std_msgs::Float32>("IMU/Raw_yaw",1000);
ros::Subscriber accel_sub = nh.subscribe<geometry_msgs::Vector3>("IMU/accel", 1000,&accel_callback);
ros::Subscriber gyro_sub = nh.subscribe<geometry_msgs::Vector3>("IMU/gyro", 1000,&gyro_callback);
ros::Subscriber mag_sub = nh.subscribe<geometry_msgs::Vector3>("IMU/mag", 1000,&mag_callback);
ros::Rate loop_rate(100);
std_msgs::Float32 filtered_roll;
std_msgs::Float32 filtered_pitch;
std_msgs::Float32 filtered_yaw;
std_msgs::Float32 Raw_yaw;
prevTime_kalman=ros::Time::now();
while(ros::ok())
{
imu[0].setState(state_r);
imu[1].setState(state_p);
imu[2].setState(state_y);
imu[0].setInput(G_roll);
imu[1].setInput(G_pitch);
imu[2].setInput(G_yaw);
for(int i=0;i<3;i++)
   imu[i].state_prediction((ros::Time::now()-prevTime_kalman).toSec());
imu[0].measurementFunction(A_roll*(180/M_PI)-r_offset); 
imu[1].measurementFunction(A_pitch*(180/M_PI)-p_offset); 
imu[2].measurementFunction(M_yaw*(180/M_PI));
for(int i=0;i<3;i++)
   imu[i].state_update();
state_r=imu[0].get_state();
state_p=imu[1].get_state();
state_y=imu[2].get_state();
filtered_roll.data=state_r[0];
filtered_pitch.data=state_p[0];
filtered_yaw.data=state_y[0];
ROS_INFO("filtered roll:%f",filtered_roll.data);
ROS_INFO("filtered pitch:%f",filtered_pitch.data);
ROS_INFO("filtered yaw:%f",filtered_yaw.data);
prevTime_kalman=ros::Time::now();
Raw_yaw.data=M_yaw*(180/M_PI);
roll_pub.publish(filtered_roll);
pitch_pub.publish(filtered_pitch);
yaw_pub.publish(filtered_yaw);
raw_yaw_pub.publish(Raw_yaw);
loop_rate.sleep();
ros::spinOnce();
}
}