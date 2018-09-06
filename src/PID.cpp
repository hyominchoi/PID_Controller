#include "PID.h"

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
    p_error = 0.;
    i_error = 0.;
    d_error = 0.;
    /* Twiddle coefficients */
    dp[0] = Kp * 0.1;
    dp[1] = Ki * 0.1;
    dp[2] = Kd * 0.1;
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

double PID::TwiddleUpdateError(double cte) {

    double steer_angle = UpdateSteerAngle(cte);
    if (num_steps%200 >= 100) {
        twiddle_total_error += cte * cte;
    }
    
    if (num_steps%200 == 0) {
        twiddle_total_error = twiddle_total_error/200;
    }
    return steer_angle;
}

/*
void PID::Twiddle(double cte, int num_steps) {
    
    while(dp[0] > 0.001) {
        Kp += dp[0];
        double err =
    }
}

def twiddle(tol=0.2):
 # Don't forget to call `make_robot` before every call of `run`!
 p = [0, 0, 0]
 dp = [1, 1, 1]
 robot = make_robot()
 x_trajectory, y_trajectory, best_err = run(robot, p)
 
 while (sum(dp) > tol):
 for i in range(len(p)):
 p[i] += dp[i]
 robot = make_robot()
 x_trajectory, y_trajectory, err = run(robot, p)
 if err < best_err:
 print(i, 1)
 dp[i] *= 1.1
 best_err = err
 else:
 p[i] -= 2*dp[i]
 robot = make_robot()
 x_trajectory, y_trajectory, err = run(robot, p)
 if err < best_err:
 print(i, 2)
 best_err = err
 dp[i] *= 1.1
 else:
 print(i, 3)
 p[i] += dp[i]
 dp[i] *= 0.9
 return p, best_err
 
*/
 
/*
def run(robot, params, n=100, speed=1.0):
x_trajectory = []
y_trajectory = []
err = 0
prev_cte = robot.y
int_cte = 0
for i in range(2 * n):
cte = robot.y
diff_cte = cte - prev_cte
int_cte += cte
prev_cte = cte
steer = -params[0] * cte - params[1] * diff_cte - params[2] * int_cte
robot.move(steer, speed)
x_trajectory.append(robot.x)
y_trajectory.append(robot.y)
if i >= n:
err += cte ** 2
return x_trajectory, y_trajectory, err / n
 */


// Parameter fitting helper functions
/* Twiddle method */




