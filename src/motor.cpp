// Source file for motor, PID and line following. 

#include <Arduino.h>
#include "motor.h"

// Variables for motor
const int frequency = 30000; // max frequency is 100kHz
int baseSpeed = 100;
int memory = 0; // forward-0, left-1, right-2, backwards-3

// Variable for PID
long threshold = 1000; // Placeholder 
int sensor[6] = {IR0, IR1, IR2, IR3, IR4, IR5};
long sensorValue[6];
float Kp = 1.0;
float Ki = 1.0;
float Kd = 1.0;
int error, position, speed, lastError;
int target = 2500;

LineFollower::LineFollower()
{

}

void LineFollower::begin(){
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    
    pinMode(IR1, INPUT);
    pinMode(IR2, INPUT);
    pinMode(IR3, INPUT);
    pinMode(IR4, INPUT);
    pinMode(IR5, INPUT);
    pinMode(IR0, INPUT);
}

// Motor function
void LineFollower::moveMotors(int leftSpeed, int rightSpeed){
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  // PWM range from 0(OFF) to 255(MAX)
  analogWriteFrequency(frequency);
  analogWrite(PWMA, leftSpeed); 
  analogWrite(PWMB, rightSpeed);
}

void LineFollower::readSensors() {
  for (int i = 0; i < 6; i++) {
    sensorValues[i] = analogRead(sensor[i]);
  }
}

int PID(){
  // weighted average: 2500 is perfect middle of line 
  position = (0*sensorValue[0] + 1000*sensorValue[1] + 2000*sensorValue[2] + 3000*sensorValue[3] + 4000*sensorValue[4] + 5000*sensorValue[5]) / (sensorValue[0] + sensorValue[1] + sensorValue[2] + sensorValue[3] + sensorValue[4] + sensorValue[5]);
  error = target - position;

  // if is over limit of position
  if (error < 0)
  {
    error = 0;
  }
  if (error > 6000)
  {
    error = 6000;
  }

  speed = Kp * error + Kd * (error - lastError);
  lastError = error;

  // Clamp motor speed 
  int leftSpeed = baseSpeed - speed;
  if (leftSpeed > 255)
  {
    leftSpeed = 255;
    }
  else if (leftSpeed < 0)
  {
    leftSpeed = 0;
  }
  int rightSpeed = baseSpeed + speed;
  if (rightSpeed > 255)
  {
    rightSpeed = 255;
  }
  else if (rightSpeed < 0)
  {
    rightSpeed = 0;
  }
  return leftSpeed, rightSpeed;
}

// Direction function
void direction(int *memory)
{
  // detect what situation the car is facing 
  // Forward: a = b, Left: a < b, Right: a > b, Backwards: -a = -b
  // forward case: call PID function
  if ((sensorValue[0] < threshold) && (sensorValue[5] < threshold) && (sensorValue[2] > threshold) && (sensorValue[3] > threshold)){
    //int leftSpeed, rightSpeed = PID();
    move(baseSpeed, baseSpeed); // change to PID speed later 
    *memory = 0;
  }

  // leftmost sensor detects black, turn spot right
  else if ((sensorValue[0] > threshold) && (sensorValue[2] > threshold) && (sensorValue[3] > threshold) && (sensorValue[5] < threshold)){
    move(-baseSpeed, baseSpeed);
    *memory = 2;
  }
  // rightmost sensor detects black, turn spot left
  else if ((sensorValue[0] < threshold) && (sensorValue[2]  > threshold) && (sensorValue[3] > threshold) && (sensorValue[5] > threshold)){
    move(baseSpeed, -baseSpeed);
    *memory = 1;
  }
  // all sensors detect black, stop
  else if ((sensorValue[0] > threshold) && (sensorValue[1] > threshold) && (sensorValue[2] > threshold) && (sensorValue[3] > threshold) && (sensorValue[4] > threshold) && (sensorValue[5] > threshold)){
    move(0, 0);
  }
}

