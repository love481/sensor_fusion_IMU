#include"filters_algorithms/imu_sensor_fusion.h"
kalman::kalman()
{
 param=new kalman_param();
 state={0,0};
 input=0;
 param->F_x=identity;
 param->F_u={0,0};
 param->Q={{0.001,0},{0,0.003}};
 param->P=float(0.1)*(mat<float,2,2>)identity;
 param->R=0.1;
 param->H={{1},{0}};
 param->K_gain={0,0};

}
void kalman::setState(std::vector<float> state_)
{
  for(int i=0;i<state_.size();i++)
     state[i]=state_[i];
}
void kalman::setInput(float input)
{
  this->input=input;
}

vec<float,2>kalman::transitionFunction(vec<float,2>prev_state,float input,float dt)
{
  //process model function is defines here
  //prev_state(roll_t-1,del_roll_t-1),curr_input(del_roll obtained from gyro)
  param->F_x={{1,0},{-dt,1}};
  param->F_u={dt,0};
  return mul(param->F_x,prev_state)+param->F_u*input;
}
 void kalman::measurementFunction(float yaw_mag)
 {
  meas=yaw_mag;
  param->H={{1},{0}};
 }
void kalman::state_prediction(float dt)
{
 state=transitionFunction(state,input,dt);
 param->Q={{0.1,0},{0,0.3}};
 param->P=mul(param->F_x,mul(param->P,transpose(param->F_x)))+param->Q;
}
void kalman::state_update()
{
    float y_k=meas-mul(param->H,state)[0];
    mat<float,1,1>innov_cov=mul(param->H,mul(param->P,transpose(param->H)))+param->R;
    mat<float,2,1>K=mul(param->P,mul(transpose(param->H),inverse(innov_cov)));
    param->K_gain={K.row(0)[0],K.row(1)[0]};
    state=state+param->K_gain*y_k;
    param->P=param->P-mul(K,mul(param->H,param->P));
}
std::vector<float>kalman::get_state()
{
  std::vector<float>state_(2);
  for(int i=0;i<2;i++)
     state_[i]=state[i];
  return state_;
}