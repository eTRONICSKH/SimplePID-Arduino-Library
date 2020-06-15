//Need to update by dev 2
#include "simplePID.h"

simplePID::simplePID(bool pid_mode): _pid_mode(pid_mode){
  _intergral = 0.0;
  _last_timer = (unsigned long)millis();
}

void simplePID::SetSampleTime(unsigned long new_sample_time){
  _sample_time_ms = new_sample_time;
}

void simplePID::SetTunings(double kp, double ki, double kd){
  _kp = kp;
  _ki = ki;
  _kd = kd;
}

void simplePID::SetOutputRange(double output_min, double output_max){
  _output_min = output_min;
  _output_max = output_max;
}

void simplePID::SetMotorParameters(double gear_ratio, double pule_per_revolution, double revolution_unit){
  _motor_gear = gear_ratio;
  _motor_ppr = pule_per_revolution;
  _motor_revolution_unit = revolution_unit;
}

double simplePID::MotorComputeLoop(double setpoint, long encoder_input){
  if (_pid_mode != PID_MOTOR_CONTROL) return;

  _elapsed_time = (unsigned long)millis()-_last_timer;
  if (_elapsed_time >= _sample_time_ms){
    _motor_encoder = encoder_input;
    _elapsed_revolution = (double)(_motor_last_encoder-_motor_encoder)/(_motor_gear*_motor_ppr);
    _motor_revolution += _elapsed_revolution;
    _motor_speed  = _motor_revolution_unit *  _elapsed_revolution / _elapsed_time;
    _motor_last_encoder = _motor_encoder;
    _last_timer = (unsigned long)millis();
    _calc_input = _motor_speed;
  }

  return(simplePID::cmd(setpoint, _calc_input, _elapsed_time));
}

double simplePID::InputComputeLoop(double setpoint, double raw_input){
  if (_pid_mode != PID_INPUT_CONTROL) return;
  _elapsed_time = (unsigned long)millis()-_last_timer;
  if (_elapsed_time >= _sample_time_ms) {
    _calc_input = raw_input;
    _last_timer = (unsigned long)millis();
  }

  return(simplePID::cmd(setpoint, _calc_input, _elapsed_time));
}

double simplePID::cmd(double setpoint, double calc_input, long elapsed_time){
  _error = setpoint - calc_input;
  _intergral += _error * (double)elapsed_time;
  _derivative = (_error - _last_error)/(double)elapsed_time;
  _cmd = _kp*_error + _ki*_intergral + _kd*_derivative;
  _last_input = calc_input;
  _last_error = _error;
  return round(constrain(_cmd, _output_min, _output_max));
}

double simplePID::MotorSpeed(){
  return _motor_speed;
}

double simplePID::MotorRevolution(){
  return _motor_revolution;
}


ramp::ramp(float a_max, float v_max, float tolerance):_a_max(a_max), _v_max(v_max), _tolerance(tolerance){
  _ex_v = 0.0;
  _error = 0.0;
}

float ramp::cmd(float desire, float current, int dt){
  _error = desire - current;
  if(abs(_error) <= _tolerance) _v = 0;
  else if(_error>0) _v = min(_v_max, sqrt(0.9*2*_a_max*(_error-_tolerance)));
  else _v = max(-_v_max, -sqrt(-0.9*2*_a_max*(_error+_tolerance)));

  if (_ex_v > _v) _v = max(_v, _ex_v - 0.001*_a_max*dt);
  else _v = min(_v, _ex_v + 0.001*_a_max*dt);

  _ex_v = _v;
  
  return _v;
}
