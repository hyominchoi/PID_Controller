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
  twiddle_best_error = 1000.;
}

void PID::UpdateError(double cte) {
    double previous_cte = p_error;
    p_error =  cte;
    i_error += cte;
    d_error = (cte - previous_cte);
    
}

double PID::TotalError() {
    return twiddle_current_error;
}


double PID::UpdateSteerAngle(double cte) {
    UpdateError(cte);
    num_steps += 1;
    double steer_angle = -Kp * p_error - Ki * i_error - Kd * d_error;
    
    if (num_steps % num_reset_period >= num_reset_period/2) {
        twiddle_current_error += cte * cte /num_reset_period;
    }
    
    if (num_steps % num_reset_period == 0) {
        //std::cout << num_steps;
        int param_index = (num_steps % (3 * num_reset_period))/num_reset_period;

        cout << "param_index " << param_index << endl ;
        if (twiddle_current_error < twiddle_best_error){
            twiddle_best_error = twiddle_current_error;
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
        twiddle_current_error = 0.;
    }
    

    return steer_angle;

}



