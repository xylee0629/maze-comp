/* Main code for maze competiton using a modified floodfill algorithm. 
// apparently bluetooth uses ADC2 pins which has conflict with motor pins :( 
-> added encoder, distance measurement need to calibrate
-> added uTurn function
-> Junction is handled by left hand rule.  currently does not support loop. memory system for turns can help this
-> check if PID works as normal
-> deleted the motor.cpp and motor.h files. Will try to make the code seperate next week
*/

#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
//#include <BluetoothSerial.h>

/*#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;*/

// TB6612FNG Motor Driver Pins
#define AIN1 27
#define AIN2 26
#define PWMA 25
#define BIN1 21
#define BIN2 22
#define PWMB 23
#define STBY 4

// Encoder Pins 
#define ENC1 16
#define ENC1_DIRECT 17
#define ENC2 18
#define ENC2_DIRECT 19

// IR Sensor Pins 
/*int IR0 = 33; // Left33
int IR1 = 32; // Left32
int IR2 = 35; // Centre35
int IR3 = 34; // Centre34
int IR4 = 39; // Right39
int IR5 = 36; // Right 36*/
int SENSOR_PINS[6] = {33, 32, 35, 34, 39, 36};

// Control Variables 
const int BASE_SPEED = 120;
const int TURN_SPEED = 150;    
const int ALIGN_TIME = 250; // Time to drive forward BEFORE turning (Center the wheels)
const int BLIND_TURN_TIME = 500; // Time to spin blind (ignore sensors) to clear the old line
const int CLEARANCE_TIME = 200; // Time to drive forward AFTER turning (Escape the junction)
int threshold = 1000;

int intersectionDelay = 150; 
int blindTurnDelay = 200;

int sensorValues[6];
int lastError = 0;
const int targetPosition = 2500;
float Kp = 0.08;
float Kd = 0.5;
int lastTurn = 0; // 0 - forward, 1 - left, 2 - right, 3 - backwards

// Encoder variables 
volatile long ENC1_TICKS = 0;
volatile long ENC2_TICKS = 0;
const int ENC_SLOTS = 20;

void readEncoder1();
void readEncoder2();
void readSensors();
void setMotorSpeed(int left, int right);
int calculatePID();
void sharpTurn(bool turnLeft);
void uTurn();


void setup() 
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  /*SerialBT.begin("ESP32"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");*/

  // set pinMode 
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  for (int i = 0; i < 6; i++)
  {
    pinMode(SENSOR_PINS[i], INPUT);
  }
  delay(2000);

  // Encoder pinMode
  pinMode(ENC1, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1), readEncoder1, RISING); // Check for interrupt pins on ESP32, check if encoder is LOW or HIGH when blocked.
  pinMode(ENC2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC2), readEncoder2, RISING);

}

void loop() 
{
  // Read all sensors
  readSensors();

  // Create easy-to-read boolean variables for our sensor states
  bool leftSensed = (sensorValues[0] > threshold);
  bool rightSensed = (sensorValues[5] > threshold);
  bool centerSensed = (sensorValues[2] > threshold || sensorValues[3] > threshold);

  // ==========================================
  // PRIORITY 1: LEFT TURN (Take immediately)
  // ==========================================
  if (leftSensed) {
    // 1. Move forward slightly to align wheels with the intersection center
    setMotorSpeed(110, 175);
    delay(150); // <-- TUNE THIS: Time to roll forward
    
    // 2. Start a hard left turn
    setMotorSpeed(0, 175); // (Or use -110, 175 for a tighter pivot)
    lastTurn = 2;
    delay(200); // <-- TUNE THIS: Blind turn time to escape the current line

    // 3. Keep turning left until the center sensors lock onto the new line
    readSensors();
    while (!(sensorValues[2] > threshold || sensorValues[3] > threshold)) {
      setMotorSpeed(0, 175);
      readSensors();
    }
  }

  // ==========================================
  // PRIORITY 2: GO STRAIGHT (Line Following)
  // ==========================================
  // If no left turn exists, but the center is on the line, follow it.
  // This automatically ignores right turns if a straight path exists!
  else if (centerSensed) {
    
    // Micro-adjustments to stay centered on the straight path
    if (sensorValues[2] > threshold && sensorValues[3] > threshold) {
      setMotorSpeed(110, 175); // Perfectly centered
    }
    else if (sensorValues[2] > threshold && sensorValues[3] < threshold) {
      setMotorSpeed(0, 175); // Drifting off, adjust left
      lastTurn = 2;
    }
    else if (sensorValues[2] < threshold && sensorValues[3] > threshold) {
      setMotorSpeed(110, 0); // Drifting off, adjust right
      lastTurn = 1;
    }
  }

  // ==========================================
  // PRIORITY 3: RIGHT TURN (Only if no Left & no Straight)
  // ==========================================
  else if (rightSensed && !leftSensed && !centerSensed) {
    // 1. Move forward slightly to align wheels
    setMotorSpeed(110, 175);
    delay(150); // <-- TUNE THIS: Time to roll forward

    // 2. Start a hard right turn
    setMotorSpeed(110, 0);
    lastTurn = 1;
    delay(200); // <-- TUNE THIS: Blind turn time

    // 3. Keep turning right until center sensors find the line
    readSensors();
    while (!(sensorValues[2] > threshold || sensorValues[3] > threshold)) {
      setMotorSpeed(110, 0);
      readSensors();
    }
  }

  // ==========================================
  // PRIORITY 4: DEAD END (U-TURN)
  // ==========================================
  else if (!leftSensed && !centerSensed && !rightSensed) {
    // Pivot around in place
    if (lastTurn == 1) {
      setMotorSpeed(110, -175); // Pivot right
    } else {
      setMotorSpeed(-110, 175); // Pivot left
    }
    
    // You may want to add a small delay here so it clears the dead end 
    // before the loop restarts and it checks the sensors again.
    delay(50);
  }

  float AVG_TICKS = (ENC1_TICKS + ENC2_TICKS) / 2.0;
  float CM_PER_TICK = PI * 3.4 / ENC_SLOTS;
  float DISTANCE = AVG_TICKS * CM_PER_TICK;

  Serial.println(ENC1_TICKS);
  Serial.println(ENC2_TICKS);
  Serial.println(DISTANCE);
  Serial.println(" ");
}


void readEncoder1()
{
  if (digitalRead(ENC1) ==  digitalRead(ENC1_DIRECT)){
    ENC1_TICKS++;
  }
  else{
    ENC1_TICKS--;
  }
}

void readEncoder2()
{
  if (digitalRead(ENC2) ==  digitalRead(ENC2_DIRECT)){
    ENC2_TICKS++;
  }
  else{
    ENC2_TICKS--;
  }
}

void readSensors()
{
  for (int i = 0; i < 6; i++)
  {
    sensorValues[i] = analogRead(SENSOR_PINS[i]);
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  Serial.println("");
}

void setMotorSpeed(int left, int right)
{
  // if left speed is positive, proceed as normal
  // However if left speed is negative, reverse digital pins 
  // need to check if pwm support negative pwm
  if (left >= 0)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else 
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
  analogWrite(PWMA, left);

  // if right speed is positive, proceed as normal
  if (right >= 0)
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  else 
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
  analogWrite(PWMB, right);
}

int calculatePID()
{
  long weightedSum = 0;
  long totalSum = 0;

  for (int i = 0; i < 6; i++)
  {
    weightedSum += (long)sensorValues[i] *(i * 1000);
    totalSum += (long)sensorValues[i];
  }

  if (totalSum == 0) return lastError;
  int position = weightedSum/totalSum;
  int error = targetPosition - position;
  int output = (Kp * error) + (Kd * (error - lastError));
  lastError = error;
  return output;
}

void sharpTurn(bool leftTurn)
{
  setMotorSpeed(0,0);
  delay(100);

  setMotorSpeed(BASE_SPEED, BASE_SPEED);
  delay(ALIGN_TIME);

  if(leftTurn) 
  {
    setMotorSpeed(-TURN_SPEED, TURN_SPEED);
  } 
  else 
  {
    setMotorSpeed(TURN_SPEED, -TURN_SPEED);
  }
  delay(BLIND_TURN_TIME); 

  long START_TIME = millis();
  while(true)
  {
    readSensors();
    long timeSpinning = millis() - START_TIME;
    if (timeSpinning < 200)
    {
      continue;
    }
    if (sensorValues[2] > 600 || sensorValues[3] > 600)
    {
      break;
    }
    if (timeSpinning > 2000) break;
  }

  setMotorSpeed(BASE_SPEED, BASE_SPEED);
  delay(CLEARANCE_TIME);
}

void uTurn()
{
  setMotorSpeed(0, 0);
  delay(200);

  setMotorSpeed(TURN_SPEED, -TURN_SPEED);

  long START_TIME = millis();
  while(true)
  {
    readSensors();
    long timeSpinning = millis() - START_TIME;

    if (timeSpinning < 400)
    {
      continue;
    }
    if (sensorValues[2] > 600 || sensorValues[3] > 600)
    {
      break;
    }
    if (timeSpinning > 3000) break;

    setMotorSpeed(0, 0);
    delay(100);
  }
}