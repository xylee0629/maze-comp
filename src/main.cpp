#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// --- PINS ---
#define AIN1 27
#define AIN2 26
#define PWMA 25
#define BIN1 21
#define BIN2 22
#define PWMB 23
#define STBY 4

const int SENSOR_PINS[6] = {33, 32, 35, 34, 39, 36}; 

// --- CALIBRATED SPEEDS (The "Golden" Numbers) ---
// STRAIGHT: The numbers that make it go straight
const int L_BASE = 110;  
const int R_BASE = 175; 

// NUDGE: The speeds to use when slightly off-center
// We keep the outer motor fast, and slow the inner motor way down to turn.
const int L_NUDGE = 50;  // Slow down left to turn left
const int R_NUDGE = 90;  // Slow down right to turn right

// TURNING: For 90 degree pivots
const int L_PIVOT = 140;
const int R_PIVOT = 210;

// --- SETTINGS ---
const int THRESHOLD = 1000;      
const int ALIGN_TIME = 150;      
const int BLIND_TURN_TIME = 250; 
const int CLEARANCE_TIME = 150;  

int sensorValues[6];

// --- FUNCTIONS ---
void readSensors();
void setMotorSpeed(int left, int right);
void sharpTurn(bool leftTurn);

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  for (int i = 0; i < 6; i++) pinMode(SENSOR_PINS[i], INPUT);
  delay(2000); 
}

void loop() {
  readSensors();

  // --- 1. JUNCTION CHECK (Highest Priority) ---
  bool leftJunction = (sensorValues[0] > THRESHOLD) && (sensorValues[1] > THRESHOLD);
  bool rightJunction = (sensorValues[4] > THRESHOLD) && (sensorValues[5] > THRESHOLD);

  if (leftJunction) {
    sharpTurn(true); // Turn Left
    return;
  }
  else if (rightJunction) {
    sharpTurn(false); // Turn Right
    return;
  }

  // --- 2. STEPPED LINE FOLLOWING (No PID!) ---
  
  // CASE A: Perfect Center (Sensors 2 or 3 are black)
  if (sensorValues[2] > THRESHOLD || sensorValues[3] > THRESHOLD) {
    // Drive Straight with calibrated speeds
    setMotorSpeed(L_BASE, R_BASE);
  }
  
  // CASE B: Drifting Right (Sensor 1 sees black) -> Needs to Turn Left
  else if (sensorValues[1] > THRESHOLD) {
    // Keep Right motor fast, cut Left motor speed
    setMotorSpeed(L_NUDGE, R_BASE);
  }

  // CASE C: Drifting Left (Sensor 4 sees black) -> Needs to Turn Right
  else if (sensorValues[4] > THRESHOLD) {
    // Keep Left motor fast, cut Right motor speed
    setMotorSpeed(L_BASE, R_NUDGE);
  }

  // CASE D: Far Off / Tight Curve (Sensors 0 or 5 see black but not a junction)
  else if (sensorValues[0] > THRESHOLD) {
     // Emergency Left Turn (Pivot)
     setMotorSpeed(0, R_BASE);
  }
  else if (sensorValues[5] > THRESHOLD) {
     // Emergency Right Turn (Pivot)
     setMotorSpeed(L_BASE, 0);
  }
  
  // CASE E: Lost Line? Keep doing whatever you did last.
}

// ---------------- FUNCTIONS ----------------

void readSensors() {
  for (int i = 0; i < 6; i++) {
    sensorValues[i] = analogRead(SENSOR_PINS[i]);
  }
}

void setMotorSpeed(int left, int right) {
  // Hard Safety: No negative numbers allowed
  left = max(0, left);
  right = max(0, right);

  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  analogWrite(PWMA, left);

  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  analogWrite(PWMB, right);
}

void sharpTurn(bool leftTurn) {
  setMotorSpeed(0, 0);
  delay(100);

  setMotorSpeed(L_BASE, R_BASE); // Align with intersection
  delay(ALIGN_TIME); 

  if (leftTurn) {
    setMotorSpeed(0, R_PIVOT); // Brake Left, Swing Right
  } else {
    setMotorSpeed(L_PIVOT, 0); // Brake Right, Swing Left
  }
  
  delay(BLIND_TURN_TIME); 

  long START_TIME = millis();
  while(true) {
    readSensors();
    if (sensorValues[2] > THRESHOLD || sensorValues[3] > THRESHOLD) break;
    if (millis() - START_TIME > 1500) break; 
  }

  setMotorSpeed(L_BASE, R_BASE); // Clear junction
  delay(CLEARANCE_TIME);
}