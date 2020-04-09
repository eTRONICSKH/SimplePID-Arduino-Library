#include <Encoder.h>
#include <simplePID.h>
#include <BTS7960.h>

#define GEAR 51.0
#define PPR 6.0
#define VALUE_PER_PULSE 4.0

#define ENCODER_A 2
#define ENCODER_B 3

#define LPWM 5
#define RPWM 6

float current_speed, desire_speed=0;
float pid_cmd;

float speedMonitor[10];
int Ism=0;

long t, xt, dt;
long current_encoder, last_encoder=0;

Encoder Enc(ENCODER_A, ENCODER_B);
simplePID pid(250.0, 120.0);  //Kp, Ki
BTS7960 motor(LPWM, RPWM);

void setup() {
  Serial.begin(9600);
  motor.init();
}

void loop() {
  readSerial();
  
  t=millis();
  dt = t - xt;

  if(dt >= 10){
    xt = t;
    current_encoder = Enc.read();
    current_speed = (1000.0*(current_encoder-last_encoder))/(dt*GEAR*PPR*VALUE_PER_PULSE);
    last_encoder = current_encoder;
    
    speedMonitor[Ism] = current_speed; //Smoothing display speed
    Ism++;
    if(Ism>=10){
      Ism=0;
    }
  }
  
  float ms=0;
  for(int i=0;i<10;i++){
    ms += speedMonitor[i];
  }
  
  pid_cmd = pid.cmd(desire_speed, current_speed, dt);
  motor.setPWM(pid_cmd);
  Serial.print("Current speed: ");
  Serial.println(ms/10);
  
}

void readSerial() {
  if(Serial.available()) {
    String inString = Serial.readString();
    desire_speed = inString.toFloat();
    Serial.print("target = ");
    Serial.println(desire_speed);
  }
}
