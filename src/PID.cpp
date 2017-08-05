#include "PID.h"
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	p_error = d_error = i_error = 0.0;
	step = 1;

	// init twiddle parameters
	dKp_ = Kp*0.1;
	dKi_ = Ki*0.1;
	dKd_ = Kd*0.1;

	current_averaged_error_   = 0.0;
	current_time_step_        = 0;
	best_averaged_error_      = 1e9;
	num_twiddle_time_steps_   = 750;
	current_tweak_dimension_  = 0;
	tried_decreasing_ = false;
}

void PID::UpdateError(double cte) {
	if (step == 1) {
		p_error = cte;
	}
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
	current_averaged_error_ = current_averaged_error_ + sqrt(cte*cte);
	step++;
}

double PID::TotalError() {
	return Kp * p_error		  // proportional
	          + Kd * d_error  // differential part
	          + Ki * i_error; // integral part
}

// taken from https://discussions.udacity.com/t/twiddle-application-in-pid-controller/243427/20
void PID::Restart(uWS::WebSocket<uWS::SERVER> ws)
{
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
  step = 0;
}

bool PID::IsTwiddleTimeIntervalComplete()
{
  if (step>num_twiddle_time_steps_)
    return true;
  else
    return false;
}

void PID::DoTwiddleTweak()
{
  if (current_averaged_error_ < best_averaged_error_ )
  {
    best_averaged_error_ = current_averaged_error_;
    current_tweak_dimension_ = (current_tweak_dimension_+1)%3;

    tried_decreasing_ = false;
    current_time_step_ = 0;
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    cte_last_time_step_ =0.0;
    current_averaged_error_   = 0.0;
    IncreaseParams();
  }
  else if (tried_decreasing_ == false )
  {
    DecreaseParams();
    tried_decreasing_ = true;
    current_time_step_ = 0;
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    cte_last_time_step_ =0.0;
    current_averaged_error_   = 0.0;
  }
  else
  {
    // nothing worked move on
    ResetParams();
    DecreaseDeltaParams();
    current_tweak_dimension_ = (current_tweak_dimension_+1)%3;
    tried_decreasing_ = false;
    IncreaseParams();
    current_time_step_ = 0;
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    cte_last_time_step_ =0.0;
    current_averaged_error_   = 0.0;
  }
}

void PID::PrintCurrentSettings()
{
  std::cout << "current_averaged_error_ " << current_averaged_error_ << " current_tweak_dimension_ " << current_tweak_dimension_;
  std::cout << " Kp_ :" <<  Kp << " Ki_ :" <<  Ki << " Kd_ :" <<  Kd ;
  std::cout << " dKp_ :" <<  dKp_ << " dKi_ :" <<  dKi_ << " Kd_ :" <<  dKd_ << std::endl;
}

void PID::IncreaseParams()
{
  switch (current_tweak_dimension_)
  {
    case 0:
      Kp +=dKp_;
      dKp_ *=1.1;
      break;
    case 1:
      Ki +=dKi_;
      dKi_ *=1.1;
      break;
    case 2:
      dKd_ *=1.1;
      Kd +=dKd_;
      break;
    default:
      break;
  }
}

void PID::ResetParams()
{
  switch (current_tweak_dimension_)
  {
    case 0:
      Kp +=dKp_;
      break;
    case 1:
      Ki +=dKi_;
      break;
    case 2:
      Ki +=dKi_;
      break;
    default:
      break;
  }
}

void PID::DecreaseParams()
{
  switch (current_tweak_dimension_)
  {
    case 0:
      Kp -=2*dKp_;
      break;
    case 1:
      Ki -=2*dKi_;
      break;
    case 2:
      Kd -=2*dKd_;
      break;
    default:
      break;
  }
}

void PID::DecreaseDeltaParams()
{
  switch (current_tweak_dimension_)
  {
    case 0:
      dKp_ *=0.9;
      break;
    case 1:
      dKi_ *=0.9;
      break;
    case 2:
      dKd_ *=0.9;
      break;
    default:
      break;
  }
}




