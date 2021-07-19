/******************
*Author:Love Panta
*gmail:075bei016.love@pcampus.edu.np
*created:2021/6/10
******************/
#pragma Once
#ifndef IMU_SENSOR_FUSION_H
#define IMU_SENSOR_FUSION_H
#include<vector>
#include"filters_algorithms/linalg.h"
#include<math.h>
using namespace linalg;
using namespace linalg::aliases;
struct kalman_param
{ 
  mat<float,2,2>F_x;//jacobian of motion model w.r.t state
  vec<float,2>F_u;//jacobian of motion model w.r.t input
  mat<float,2,2>Q; //system noise covariance
  mat<float,2,2>P;//covariance prediction
  vec<float,2>K_gain;//kalman_gain
  //meas_noise
  float R;
  //jac_meas 
  mat<float,1,2>H;
};
class kalman
{  private:
      kalman_param *param;
      vec<float,2> state;//e.g (roll,bias previous state)
      float input;//(del_roll obtained from gyro)
      float meas;//obtained from magnetometer
   public:
      kalman();
      kalman(kalman &&) = default;
      kalman(const kalman &) = default;
      kalman &operator=(kalman &&) = default;
      kalman &operator=(const kalman &) = default;
      ~kalman(){}
      void set_EKF_param(kalman_param *p){param=p;}
      void setState(std::vector<float>);
      void setInput(float);
      void state_prediction(float);
      void state_update();
      vec<float,2> transitionFunction(vec<float,2>,float,float);
      void measurementFunction(float);
      std::vector<float>get_state();
};

#endif // !IMU_SENSOR_FUSION_H
