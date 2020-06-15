#ifndef SIMPLE_PID_MASTER
#define SIMPLE_PID_MASTER

#include "Arduino.h"

class simplePID{
  public:
    #define MOTOR_RPS 1000.0
    #define MOTOR_RPM 60000.0
    #define PID_INPUT_CONTROL false
    #define PID_MOTOR_CONTROL true

    simplePID(bool pid_mode);
    void SetSampleTime(unsigned long sample_time);
    void SetTunings(double kp, double ki, double kd);
    void SetOutputRange(double output_min, double output_max);
    void SetMotorParameters(double gear_ratio, double pule_per_revolution, double revolution_unit);

    double MotorComputeLoop(double setpoint, long encoder_input);
    double MotorSpeed();
    double MotorRevolution();

    double InputComputeLoop(double setpoint, double raw_input);

    double cmd(double setpoint, double calc_input, long elapsed_time);
  
  private:
    bool _pid_mode;

    double _kp = 0.0;
    double _ki = 0.0;
    double _kd = 0.0;

    unsigned long _sample_time_ms = 10;

    //motor-encoder-parameters
    double _motor_gear= 1.0;
    double _motor_ppr = 1.0;
    double _motor_revolution_unit = MOTOR_RPS; //round per second

    long _motor_encoder;
    long _motor_last_encoder;
    double _elapsed_revolution;
    double _motor_revolution = 0.0;
    double _motor_speed = 0.0;

    //pid-computing-parameters
    double _calc_input;
    double _error;
    double _last_input;
    double _last_error;
    double _intergral;
    double _derivative;
    double _cmd;

    //output-limits
    double _output_min = -255;
    double _output_max = 255;

    //computing-timer
    unsigned long _last_timer;
    unsigned long _elapsed_time;

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
