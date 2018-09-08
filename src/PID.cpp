#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;
  K[0] = Kp;
  K[1] = Ki;
  K[2] = Kd;
  p_error = 0.;
  i_error = 0.;
  d_error = 0.;
}

void PID::UpdateError(double cte) {
    double previous_cte = p_error;
    p_error =  cte;
    i_error += cte;
    d_error = (cte - previous_cte);
    
}

double PID::TotalError() {
    return p_error;
}

double PID::UpdateSteerAngle(double cte) {
  UpdateError(cte);
  double steer_angle = -Kp * p_error - Ki * i_error - Kd * d_error;
  return steer_angle;
}


PID_CALIBRATE::PID_CALIBRATE() {}
PID_CALIBRATE::~PID_CALIBRATE() {}

void PID_CALIBRATE::Init(double Kp, double Ki, double Kd) {
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;
  K[0] = Kp;
  K[1] = Ki;
  K[2] = Kd;
  p_error = 0.;
  i_error = 0.;
  d_error = 0.;
  /* Twiddle coefficients */
  dp[0] = Kp * 0.01;
  dp[1] = Ki * 0.1;
  dp[2] = Kd * 0.05;
  
  twiddle_flag[0] = 1;
  twiddle_flag[1] = 1;
  twiddle_flag[2] = 1;
  num_steps = 0;
  num_reset_period = 100;
  twiddle_current_error = 0.;
  twiddle_best_error[0] = 1000.;
  twiddle_best_error[1] = 1000.;
  twiddle_best_error[2] = 1000.;
}

double PID_CALIBRATE::UpdateSteerAngle(double cte) {
  PID::UpdateError(cte);
  num_steps += 1;
  double steer_angle = -Kp * p_error - Ki * i_error - Kd * d_error;
    
  if (num_steps % num_reset_period >= num_reset_period/2) {
      twiddle_current_error += cte * cte /num_reset_period;
  }
  
  /* modified Twiddle algorithm
   There is no tol. Instead the parameter Kp, Ki, Kd gets
   updated one at a time (every num_reset_period) depending on
   the error from previous cycle */
  
  if (num_steps % num_reset_period == 0) {
      //std::cout << num_steps;
    // param_index is one of 0, 1, 2 corresponding Kp, Ki, Kd respectively.
    int param_index = (num_steps % (3 * num_reset_period))/num_reset_period;

    if (twiddle_current_error < twiddle_best_error[param_index]){
        twiddle_best_error[param_index] = twiddle_current_error;
        twiddle_flag[param_index] = true;
        K[param_index] += dp[param_index];
    }
    else {
        if (twiddle_flag[param_index] == true) {
            K[param_index] -= 2 * dp[param_index];
            twiddle_flag[param_index] = false;
        }
        else {
            K[param_index] += dp[param_index];
            dp[param_index] *= 0.9;
            twiddle_flag[param_index] = false;
        }
    }
    // reset twiddle error
    twiddle_current_error = 0.;
  }
    

    return steer_angle;

}
