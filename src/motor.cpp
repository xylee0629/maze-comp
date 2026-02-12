// Source file for motor, PID and line following. 

#include <Arduino.h>
#include "motor.h"

LineFollower::LineFollower(int sensor[6])
{

    // Default values
    baseSpeed = 100;
    maxSpeed = 255;
    threshold = 1000;
    targetPosition = 2500;
    lastError = 0;
    memory = 0; // forward-0, left-1, right-2, backwards-3
  
    // Default PID
    Kp = 1.0; 
    Ki = 0.0; 
    Kd = 0.0;
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

void LineFollower::setBaseSpeed(int speed)
{
    baseSpeed = speed;
}

void LineFollower::setPID(float p, float i, float d)
{
    Kp = p;
    Ki = i;
    Kd = d;
}

// Motor function
void LineFollower::moveMotors(int leftSpeed, int rightSpeed){
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  // PWM range from 0(OFF) to 255(MAX)
  analogWrite(PWMA, leftSpeed); 
  analogWrite(PWMB, rightSpeed);
}

void LineFollower::readSensors() {
  for (int i = 0; i < 6; i++) {
    sensorValue[i] = analogRead(sensor[i]);
  }
}

int LineFollower::PID(){
  // weighted average: 2500 is perfect middle of line 
  position = (0*sensorValue[0] + 1000*sensorValue[1] + 2000*sensorValue[2] + 3000*sensorValue[3] + 4000*sensorValue[4] + 5000*sensorValue[5]) / (sensorValue[0] + sensorValue[1] + sensorValue[2] + sensorValue[3] + sensorValue[4] + sensorValue[5]);
  error = targetPosition - position;

  // if is over limit of position
  if (error < 0)
  {
    error = 0;
  }
  if (error > 6000)
  {
    error = 6000;
  }

  adjustSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;
  return adjustSpeed;
}

void LineFollower::update()
{
    readSensors();

    if (sensorValue[0] > threshold && sensorValue[5] < threshold) {
     moveMotors(-baseSpeed, baseSpeed); // Spot turn Left
     return;
    }
  
    // Example: Extreme Right (Sensor 5 high, others low)
    if (sensorValue[5] > threshold && sensorValue[0] < threshold) {
     moveMotors(baseSpeed, -baseSpeed); // Spot turn Right
     return;
    }

    // Example: All sensors black (Stop)
    bool allBlack = true;
    for(int i=0; i<6; i++) {
    if(sensorValue[i] < threshold) allBlack = false;
    }
    if (allBlack) {
        moveMotors(0, 0);
        return;
    }

  // --- Normal Line Following (PID) ---
  int adjustSpeed = PID();

  int leftSpeed = baseSpeed - adjustSpeed;
  int rightSpeed = baseSpeed + adjustSpeed;

  moveMotors(leftSpeed, rightSpeed);

}


