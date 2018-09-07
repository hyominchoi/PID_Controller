#ifndef PID_H
#define PID_H

class PID {

public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */
  double K[3];
  double Kp;
  double Ki;
  double Kd;
    
  /* Twiddle coefficients dps */
  double dp[3];
  double twiddle_current_error;
  double twiddle_best_error;
  int num_reset_period;
  int num_steps;
  bool twiddle_flag[3];
    
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
    
  double UpdateSteerAngle(double cte);
    

};




#endif /* PID_H */
