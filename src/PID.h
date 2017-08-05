#ifndef PID_H
#define PID_H

#include <uWS/uWS.h>


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
  double Kp;
  double Ki;
  double Kd;

  int step;

  /*
  *  twiddle params
  */
  double dKp_;
  double dKi_;
  double dKd_;

  double current_averaged_error_;

  double best_averaged_error_;

  double num_twiddle_time_steps_;

  int current_tweak_dimension_;

  int current_time_step_;

  bool tried_decreasing_;

  /*
  * Helpers
  */
  double cte_last_time_step_;


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

  /*
  * Send restart message to simulator
  */
  void Restart(uWS::WebSocket<uWS::SERVER> ws);


  /*
  * Check whether twiddle time interval has elapsed
  */
  bool IsTwiddleTimeIntervalComplete();

  /*
  * Run twiddle step
  */
  void DoTwiddleTweak();

  /*
  * Print corrent controller parameters to std::cout
  */
  void PrintCurrentSettings();

  /*
  * Increase controller parameters (Twiddle step)
  */
  void IncreaseParams();

  /*
  * Reset all controller parameter
  */
  void ResetParams();

  /*
  * Decrease controller parameters (Twiddle step)
  */
  void DecreaseParams();

  /*
  * Decrease delta parameters (Twiddle step)
  */
  void DecreaseDeltaParams();


};

#endif /* PID_H */
