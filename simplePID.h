#ifndef simplePID_master
#define simplePID_master

#include "Arduino.h"

class simplePID{
  public:
    simplePID(float kp, float ki);
    float cmd(float desire, float current, long dt);
  
  private:
    const float _kp;
    const float _ki;
    const float _limit;
    float _error;
    float _intergral;
    float _cmd;

};

class ramp{
  public:
    ramp(float a_max, float v_max, float tolerance);
    float cmd(float desire, float current, int dt);

  private:
    float _e;
    float _error;
    float _v;
    float _ex_v;
    float _v_max;
    float _a_max;
    float _tolerance;
};



#endif
