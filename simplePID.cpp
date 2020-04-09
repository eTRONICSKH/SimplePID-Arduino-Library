#include "simplePID.h"

simplePID::simplePID(float kp, float ki): _kp(kp), _ki(ki), _limit(1.0){
  _intergral = 0.0;
}


float simplePID::cmd(float desire, float current, long dt){
  _error = desire - current;
  _intergral += _error * dt/1000.0;
  _intergral = max(-_limit, min(_limit, _intergral));
  _cmd = _kp*_error + _ki*_intergral;
  return round(max(-255, min(255, _cmd)));
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
