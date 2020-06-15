#include <simplePID.h>

#define PLOTTER true        // Enable / Disable graph plotter

#define LED_PIN 3           // LED pwm pin control
#define LDR_PIN A1          // LDR analog pin read value

uint8_t pwm_write = 0;      // PWM control LED bright 0-255
double value_read = 0;    // Analog from LDR read 0-1023
double setpoint = 600;        // Set desire value

String inString = "";

simplePID ldr_control(PID_INPUT_CONTROL); //Set mode as general INPUT mode

void setup() {
  Serial.begin(115200);
  pinMode(LDR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  //Setup PID config
  ldr_control.SetSampleTime(1);                 // Smaller simple time make system smoother
  ldr_control.SetTunings(0.060, 0.020, 0.20);   // Should start with smaller value
  ldr_control.SetOutputRange(0, 255);           // Set as PWM range
}

void loop() {
  serialValue();
  
  /* Value read/write mush be logic to input/output
   * Ex. value_read must be higher when LED is brighter.
   * Do any calculation.*/
  value_read = 1023-analogRead(LDR_PIN); 
  
  pwm_write = ldr_control.InputComputeLoop(setpoint, value_read); // Compute function need to run without any delay before and after.
  analogWrite(LED_PIN, pwm_write);
  
  if(PLOTTER){
    Serial.print(setpoint);
    Serial.print(" ");
    Serial.print(value_read);
    Serial.print(" ");
    Serial.println(pwm_write);
  }
  
}

void serialValue(){
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string, then the string's value:
    if (inChar == '\n') {
      setpoint = inString.toInt();
      inString = "";
    }
  }
  
}

