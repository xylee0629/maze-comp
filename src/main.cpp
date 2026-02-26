#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// --- PINS ---
#define AIN1 26
#define AIN2 25
#define PWMA 27
#define BIN1 21
#define BIN2 22
#define PWMB 23
#define STBY 4

const int SENSOR_PINS[6] = {33, 32, 35, 34, 39, 36}; 

// --- CALIBRATED SPEEDS (The "Golden" Numbers) ---
const int L_BASE = 115;  
const int R_BASE = 120; 

const int L_NUDGE = 75;  
const int R_NUDGE = 80;  

const int L_PIVOT = 215;
const int R_PIVOT = 220;

// --- SETTINGS ---
const int THRESHOLD = 900;      
const int ALIGN_TIME = 50;      
const int BLIND_TURN_TIME = 200; 
const int CLEARANCE_TIME = 150;  

// --- DEAD END SETTINGS ---
const int DEAD_END_LIMIT = 50;  // How many consecutive loops of "all white" to trigger a U-turn
int whiteCount = 0;

int sensorValues[6];

// --- MEMORY MATRIX SETTINGS ---
#define MAX_PATH 100          // Maximum number of turns the robot can remember
char pathMemory[MAX_PATH];    // The array storing the turns ('L', 'R')
int pathLength = 0;           // Keeps track of how many turns we've made

// --- MAZE STATE ---
int blackBoxCount = 0;       // Counts how many black boxes we've hit
bool currentlyOnBox = false; // Prevents the robot from counting the same box twice

// --- FUNCTIONS ---
void readSensors();
void setMotorSpeed(int left, int right);
void sharpTurn(bool leftTurn);
void recordTurn(char turn); // New function to log turns
void executeUTurn();
void simplifyPath();

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
  /*setMotorSpeed(L_BASE, R_BASE);
  delay(2000);
  setMotorSpeed(L_BASE, 0);
  delay(2000);
  setMotorSpeed(L_BASE, -R_BASE);
  delay(2000);
  setMotorSpeed(0, R_BASE);
  delay(2000);
  setMotorSpeed(-L_BASE, R_BASE);
  delay(2000);
  setMotorSpeed(0, 0);
  delay(2000);*/

readSensors();
// =========================================================
  // PRIORITY 0: START / FINISH BOX CHECK (Counter Method)
  // =========================================================
  bool allBlack = true;
  for (int i = 0; i < 6; i++) {
    if (sensorValues[i] < THRESHOLD) { 
      allBlack = false; 
      break; 
    }
  }

  if (allBlack) {
    if (!currentlyOnBox) {
      // We just transitioned from the white floor onto a black box!
      currentlyOnBox = true;
      blackBoxCount++;
    }

    if (blackBoxCount >= 2) {
      // This is the SECOND black box we've seen. Maze complete!
      setMotorSpeed(0, 0); // Hard Stop
      Serial.println("===========================");
      Serial.println("        MAZE SOLVED!       ");
      Serial.print("Final Path Memory: ");
      for(int i = 0; i < pathLength; i++) {
        Serial.print(pathMemory[i]);
      }
      Serial.println("\n===========================");
      while(true) delay(1000); // Freeze forever
    } else {
      // This is the FIRST black box (Start Box). Push forward to get off it.
      setMotorSpeed(L_BASE, R_BASE);
      return; 
    }
  } else {
    // Outer sensors see white, so we are definitely not on a black box right now
    currentlyOnBox = false; 
  }

  // --- 0. DEAD END CHECK (Move Forward + Counter) ---
  bool allWhite = true;
  for (int i = 0; i < 6; i++) {
    if (sensorValues[i] > THRESHOLD) {
      allWhite = false; 
      break; 
    }
  }

  // If all sensors read white...
  if (allWhite) {
    whiteCount++;
    
    // Force the car to drive straight forward to check for the line
    setMotorSpeed(L_BASE, R_BASE); 
    
    // If it drives forward long enough and STILL sees no line, it's a dead end
    if (whiteCount >= DEAD_END_LIMIT) {
      recordTurn('U'); 
      executeUTurn();
      whiteCount = 0; // Reset the counter after the turn
      return;
    }
    
    // Skip the rest of the loop so it doesn't trigger emergency turns
    return; 
    
  } else {
    // If ANY sensor sees the line, instantly reset the counter to 0
    whiteCount = 0; 
  }

 bool leftJunction = (sensorValues[0] > THRESHOLD) && (sensorValues[1] > THRESHOLD);
  bool rightJunction = (sensorValues[4] > THRESHOLD) && (sensorValues[5] > THRESHOLD);

  if (leftJunction || rightJunction) {
    setMotorSpeed(0, 0); delay(50); // Brake

    // Push past the horizontal line to read the paths ahead
    setMotorSpeed(L_BASE, R_BASE);
    delay(ALIGN_TIME); 

    setMotorSpeed(0, 0);
    readSensors();

    bool canGoLeft = leftJunction; 
    bool canGoRight = rightJunction;
    bool canGoStraight = (sensorValues[2] > THRESHOLD || sensorValues[3] > THRESHOLD);

    int availablePaths = 0;
    if (canGoLeft) availablePaths++;
    if (canGoRight) availablePaths++;
    if (canGoStraight) availablePaths++;

    bool isDecisionPoint = (availablePaths > 1);

    // Left-Hand Rule Execution
    if (canGoLeft) {
      if (isDecisionPoint) recordTurn('L');
      sharpTurn(true); 
      return;
    }
    else if (canGoStraight) {
      if (isDecisionPoint) recordTurn('S');
      return; // Resume normal driving
    }
    else if (canGoRight) {
      if (isDecisionPoint) recordTurn('R');
      sharpTurn(false);
      return;
    }
  }

  // --- 2. STEPPED LINE FOLLOWING (No PID!) ---
  
  // CASE A: Perfect Center (Sensors 2 or 3 are black)
  if (sensorValues[2] > THRESHOLD || sensorValues[3] > THRESHOLD) {
    setMotorSpeed(L_BASE, R_BASE);
  }
  // CASE B: Drifting Right (Sensor 1 sees black) -> Needs to Turn Left
  else if (sensorValues[1] > THRESHOLD) {
    setMotorSpeed(L_NUDGE, R_BASE);
  }
  // CASE C: Drifting Left (Sensor 4 sees black) -> Needs to Turn Right
  else if (sensorValues[4] > THRESHOLD) {
    setMotorSpeed(L_BASE, R_NUDGE);
  }
  // CASE D: Far Off / Tight Curve (Sensors 0 or 5 see black but not a junction)
  else if (sensorValues[0] > THRESHOLD && sensorValues[2] < THRESHOLD) {
     setMotorSpeed(0, R_BASE);
  }
  else if (sensorValues[5] > THRESHOLD && sensorValues[3] < THRESHOLD) {
     setMotorSpeed(L_BASE, 0);
  }

  
}

// ---------------- FUNCTIONS ----------------

void readSensors() {
  for (int i = 0; i < 6; i++) {
    sensorValues[i] = analogRead(SENSOR_PINS[i]);
  }
}

void setMotorSpeed(int left, int right) {
  if (left >= 0)
  {
      digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
      analogWrite(PWMA, abs(left));
  }
  else 
  {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, abs(left));
  }

  if (right >=0)
  {
      digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
      analogWrite(PWMB, abs(right));
  }
  else 
  {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, abs(right));
  }
}

void sharpTurn(bool leftTurn) {
  // Swing
  if (leftTurn) setMotorSpeed(0, R_PIVOT);
  else setMotorSpeed(L_PIVOT, 0);
  
  delay(BLIND_TURN_TIME); 

  // Sweep to center
  long START_TIME = millis();
  while(true) {
    readSensors();
    if (leftTurn && sensorValues[3] > THRESHOLD) break; // Sweeping left
    else if (!leftTurn && sensorValues[2] > THRESHOLD) break; // Sweeping right
    if (millis() - START_TIME > 1500) break; 
  }

  // Active Brake
  setMotorSpeed(0, 0);
  delay(50);

  // Micro-Adjust "Wiggle"
  long SETTLE_TIME = millis();
  while (millis() - SETTLE_TIME < 300) { 
    readSensors();
    bool s2 = sensorValues[2] > THRESHOLD;
    bool s3 = sensorValues[3] > THRESHOLD;

    if (s2 && !s3) setMotorSpeed(L_NUDGE, 0); 
    else if (s3 && !s2) setMotorSpeed(0, R_NUDGE);
    else { setMotorSpeed(0, 0); break; }
  }
}


void executeUTurn() {
  // 1. Stop momentarily to prevent voltage spikes
  setMotorSpeed(0, 0);
  delay(100);

  // 2. Execute a tight 180-degree spin (Left Forward, Right Reverse)
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); 
  analogWrite(PWMA, L_PIVOT); // Left motor forward

  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); 
  analogWrite(PWMB, R_PIVOT); // Right motor backward

  // 3. Blind turn time (longer than a normal 90-degree turn to clear the old line)
  delay(BLIND_TURN_TIME * 1.5); 

  // 4. Wait until the center sensors find the line again
  long START_TIME = millis();
  while(true) {
    readSensors();
    // If center sensors see the line, stop spinning
    if (sensorValues[2] > THRESHOLD || sensorValues[3] > THRESHOLD) break;
    
    // Safety timeout just in case it completely loses the line
    if (millis() - START_TIME > 2000) break; 
  }

  // 5. Stop and resume normal driving
  setMotorSpeed(0, 0); 
  delay(100);
  setMotorSpeed(L_BASE, R_BASE); 
}

void recordTurn(char turn) {
  if (pathLength < MAX_PATH) {
    pathMemory[pathLength] = turn;
    pathLength++;
    simplifyPath(); // Immediately try to compress dead ends
  } else {
    Serial.println("WARNING: Memory Matrix Full!");
  }
}

void simplifyPath() {
  if (pathLength < 3 || pathMemory[pathLength - 2] != 'U') return; 

  int totalAngle = 0;
  for (int i = 1; i <= 3; i++) {
    char dir = pathMemory[pathLength - i];
    if (dir == 'L') totalAngle -= 90;
    else if (dir == 'R') totalAngle += 90;
    else if (dir == 'U') totalAngle += 180;
    else if (dir == 'S') totalAngle += 0;
  }

  totalAngle = totalAngle % 360;
  if (totalAngle < 0) totalAngle += 360; 

  char newTurn;
  if (totalAngle == 0) newTurn = 'S';
  else if (totalAngle == 90) newTurn = 'R';
  else if (totalAngle == 180) newTurn = 'U';
  else if (totalAngle == 270) newTurn = 'L';

  pathLength -= 3;
  pathMemory[pathLength] = newTurn;
  pathLength++;

  Serial.print("Path Optimized! New Memory: [ ");
  for(int i = 0; i < pathLength; i++) {
    Serial.print(pathMemory[i]);
    Serial.print(" ");
  }
  Serial.println("]");
}