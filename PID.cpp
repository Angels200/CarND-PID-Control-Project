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
	PID::p_error = PID::d_error = PID::i_error = 0.0;

	// Twiddling parameters
	PID::twiddle = false;
	dp_ = {0.1*Kp,0.1*Kd,0.1*Ki};
	total_error_ = 0;

}

void PID::UpdateError(double cte) {
  double previous_cte = p_error;

  // Proportional error is just the CTE (Cross Track Error)
  PID::p_error = cte;

  // Integration error is the sum of all CTE so far
  PID::i_error = i_error + cte;

  // Differential error is the rate of change of the CTE but assuming
  // each call is 1 time-step then it simplifies to subtracting the
  // previous CTE from the current CTE.
  PID::d_error = cte - previous_cte;
}

double PID::TotalError() {
  // Multiply out our errors by the coefficients
  return -PID::Kp * PID::p_error - PID::Ki * PID::i_error - PID::Kd * PID::d_error;
}
