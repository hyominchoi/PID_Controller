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
  double throttle;
  double steer_angle;
  
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
  virtual void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
    
  virtual double UpdateSteerAngle(double cte);
  
  void AdjustThrottle(double s1, double s2);

};


class PID_CALIBRATE : public PID {
public:
  PID_CALIBRATE();
  ~PID_CALIBRATE();
  /* Twiddle coefficients dps */
  double dp[3];
  double twiddle_current_error;
  double twiddle_best_error[3];
  int num_reset_period;
  int num_steps;
  
  void Init(double Kp, double Ki, double Kd);
  /* update steering angle and update parameters */
  double UpdateSteerAngle(double cte);
private:
  bool twiddle_flag[3];
  
};

#endif /* PID_H */
