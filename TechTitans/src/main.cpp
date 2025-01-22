#include <Arduino.h>

#define MOTOR_L_IN1       11
#define MOTOR_L_IN2       10
#define MOTOR_R_IN3       9
#define MOTOR_R_IN4       6  
#define speedControlerR   5 
#define speedControlerL   3


int speedValue = 200;

void setup() {


  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN3, OUTPUT);
  pinMode(MOTOR_R_IN4, OUTPUT);
  pinMode(speedControlerR, OUTPUT);
  pinMode(speedControlerL, OUTPUT);





}

void loop() {

digitalWrite(MOTOR_L_IN1, HIGH);
digitalWrite(MOTOR_L_IN2, LOW);
digitalWrite(MOTOR_R_IN3, HIGH);
digitalWrite(MOTOR_R_IN4, LOW);


speedValue = constrain(speedValue, 0, 255);

analogWrite(speedControlerR, speedValue);
analogWrite(speedControlerL, speedValue);

}

