#include <Arduino.h>

// ==========================================
// 1. HARDWARE PINS & CONSTANTS
// ==========================================
// TB6612FNG Motor Driver Pins
#define AIN1 26
#define AIN2 25
#define PWMA 27
#define BIN1 21
#define BIN2 22
#define PWMB 23
#define STBY 4

// Encoder Pins 
#define ENC1 18
#define ENC1_DIRECT 19
#define ENC2 16
#define ENC2_DIRECT 17

// IR Sensor Pins 
int IR0 = 33; // Left33
int IR1 = 32; // Left32
int IR2 = 35; // Centre35
int IR3 = 34; // Centre34
int IR4 = 39; // Right39
int IR5 = 36; // Right 36
int SENSOR_PINS[6] = {IR0, IR1, IR2, IR3, IR4, IR5};


// Movement & Tuning Constants
const int BASE_SPEED = 90;
const int TURN_SPEED = 100;    
const int ALIGN_TIME = 250; 
const int BLIND_TURN_TIME = 300; 
const int CLEARANCE_TIME = 200; 

// PID Variables
int sensorValues[6];
int lastError = 0;
const int targetPosition = 2500;
float Kp = 0.08;
float Kd = 0.5;

// ==========================================
// 2. ENCODER VARIABLES & MATH
// ==========================================
volatile long ENC1_TICKS = 0;
volatile long ENC2_TICKS = 0;
const int ENC_SLOTS = 20;

const float GRID_SIZE_CM = 25.0; 
const float WHEEL_DIAMETER_CM = 3.4; 
const float CM_PER_TICK = (PI * WHEEL_DIAMETER_CM) / ENC_SLOTS;

// ==========================================
// 3. FLOODFILL MAZE VARIABLES
// ==========================================
#define MAZE_SIZE 9

// Headings: 0=North, 1=East, 2=South, 3=West
int heading = 0; 
int rx = 0; 
int ry = 0; 
int targetX = 8; // Goal coordinates (Adjust as needed based on the maze)
int targetY = 8;

// Bitmasks: 1=North, 2=East, 4=South, 8=West
byte paths[MAZE_SIZE][MAZE_SIZE]; 
byte visited[MAZE_SIZE][MAZE_SIZE];
int distances[MAZE_SIZE][MAZE_SIZE];

// ==========================================
// 4. FUNCTION PROTOTYPES
// ==========================================
void readEncoder1();
void readEncoder2();
float getDistance();
void resetEncoders();
void readSensors();
void setMotorSpeed(int left, int right);
int calculatePID();
void sharpTurn(bool turnLeft);
void uTurn();
void updateMap(bool L, bool S, bool R);
void calculateFloodFill();
int getNextMove();
void executeMove(int targetHeading, bool wasVirtual);

// ==========================================
// 5. SETUP
// ==========================================
void setup() {
  Serial.begin(115200);

  // Motor Setup
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

  // Sensor Setup
  for (int i = 0; i < 6; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  // Encoder Setup
  pinMode(ENC1, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1), readEncoder1, RISING); 
  pinMode(ENC2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC2), readEncoder2, RISING);

  // Initialize maze arrays
  for(int x=0; x<MAZE_SIZE; x++) {
    for(int y=0; y<MAZE_SIZE; y++) {
      paths[x][y] = 15; // 15 in binary is 1111 (Assume all directions exist initially)
      visited[x][y] = 0;
    }
  }
  calculateFloodFill(); // Initial distance calculation

  delay(2000);
}

// ==========================================
// 6. MAIN LOOP
// ==========================================
void loop() {
  readSensors();

  // Determine what type of intersection we are seeing physically
  bool leftJunction = (sensorValues[0] > 600) && (sensorValues[1] > 600);
  bool rightJunction = (sensorValues[4] > 600) && (sensorValues[5] > 600);
  bool straightPath = (sensorValues[2] > 600) || (sensorValues[3] > 600);
  
  bool isPhysicalJunction = leftJunction || rightJunction;
  bool deadEnd = !leftJunction && !rightJunction && !straightPath;
  
  // Check virtual distance
  float currentDistance = getDistance();
  bool isVirtualNode = (currentDistance >= 24.5); // Slightly under 25cm to prevent overshoot

  // If we hit any node (physical or virtual)
  if (isPhysicalJunction || deadEnd || isVirtualNode) {
    
    // Stop only for physical events
    if (isPhysicalJunction || deadEnd) {
      setMotorSpeed(0, 0);
      delay(100);
    }

    // Update coordinates and maps
    visited[rx][ry] = 1;

    if (isVirtualNode && !isPhysicalJunction) {
      updateMap(false, true, false); // Virtual node means straight path only
    } else {
      updateMap(leftJunction, straightPath, rightJunction);
    }

    // Floodfill Brain: Think and decide
    calculateFloodFill();
    int nextHeading = getNextMove();

    // Act
    executeMove(nextHeading, isVirtualNode);
    
    // Reset encoders for the next 25cm tile
    resetEncoders();
    
    return; // Loop restarts to follow the line on the new path
  }

  // --- PID LINE FOLLOWING ---
  int correction = calculatePID();
  int speedLeft = constrain(BASE_SPEED + correction, 0, 180);
  int speedRight = constrain(BASE_SPEED - correction, 0, 180);
  setMotorSpeed(speedLeft, speedRight);
}

// ==========================================
// 7. SENSOR & HARDWARE FUNCTIONS
// ==========================================
void readEncoder1() {
  if (digitalRead(ENC1) == digitalRead(ENC1_DIRECT)) ENC1_TICKS++;
  else ENC1_TICKS--;
}

void readEncoder2() {
  if (digitalRead(ENC2) == digitalRead(ENC2_DIRECT)) ENC2_TICKS++;
  else ENC2_TICKS--;
}

float getDistance() {
  noInterrupts();
  long ticks1 = ENC1_TICKS;
  long ticks2 = ENC2_TICKS;
  interrupts();
  
  float avgTicks = (abs(ticks1) + abs(ticks2)) / 2.0; 
  return avgTicks * CM_PER_TICK;
}

void resetEncoders() {
  noInterrupts();
  ENC1_TICKS = 0;
  ENC2_TICKS = 0;
  interrupts();
}

void readSensors() {
  for (int i = 0; i < 6; i++) {
    sensorValues[i] = analogRead(SENSOR_PINS[i]);
  }
}

void setMotorSpeed(int left, int right) {
  if (left >= 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  }
  analogWrite(PWMA, abs(left)); // Added abs() for safety

  if (right >= 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
  }
  analogWrite(PWMB, abs(right)); // Added abs() for safety
}

int calculatePID() {
  long weightedSum = 0;
  long totalSum = 0;

  for (int i = 0; i < 6; i++) {
    weightedSum += (long)sensorValues[i] * (i * 1000);
    totalSum += (long)sensorValues[i];
  }

  if (totalSum == 0) return lastError;
  int position = weightedSum/totalSum;
  int error = targetPosition - position;
  int output = (Kp * error) + (Kd * (error - lastError));
  lastError = error;
  return output;
}

// ==========================================
// 8. MOVEMENT FUNCTIONS
// ==========================================
void sharpTurn(bool leftTurn) {
  setMotorSpeed(0,0);
  delay(100);

  setMotorSpeed(BASE_SPEED, BASE_SPEED);
  delay(ALIGN_TIME); // Center wheels on junction

  if(leftTurn) {
    setMotorSpeed(-TURN_SPEED, TURN_SPEED);
  } else {
    setMotorSpeed(TURN_SPEED, -TURN_SPEED);
  }
  delay(BLIND_TURN_TIME); // Turn blind to get off the line

  long START_TIME = millis();
  while(true) {
    readSensors();
    long timeSpinning = millis() - START_TIME;
    if (timeSpinning < 200) continue;
    
    // Stop turning when middle sensors see the line
    if (sensorValues[2] > 600 || sensorValues[3] > 600) break;
    if (timeSpinning > 2000) break; // Timeout fallback
  }

  setMotorSpeed(BASE_SPEED, BASE_SPEED);
  delay(CLEARANCE_TIME); // Clear the junction area
}

void uTurn() {
  setMotorSpeed(0, 0);
  delay(200);

  setMotorSpeed(TURN_SPEED, -TURN_SPEED);

  long START_TIME = millis();
  while(true) {
    readSensors();
    long timeSpinning = millis() - START_TIME;

    if (timeSpinning < 400) continue;
    
    if (sensorValues[2] > 600 || sensorValues[3] > 600) break;
    if (timeSpinning > 3000) break; 
  }
}

// ==========================================
// 9. FLOODFILL ALGORITHM FUNCTIONS
// ==========================================
void updateMap(bool L, bool S, bool R) {
  byte actualPaths = 0;

  // Reverse heading to know where we came from
  int back = (heading + 2) % 4; 
  actualPaths |= (1 << back); 

  if (L) actualPaths |= (1 << ((heading + 3) % 4));
  if (S) actualPaths |= (1 << heading);
  if (R) actualPaths |= (1 << ((heading + 1) % 4));

  paths[rx][ry] = actualPaths;
}

void calculateFloodFill() {
  for(int x=0; x<MAZE_SIZE; x++) {
    for(int y=0; y<MAZE_SIZE; y++) {
      distances[x][y] = 255; 
    }
  }
  
  distances[targetX][targetY] = 0;
  
  bool changed = true;
  while(changed) {
    changed = false;
    for(int x=0; x<MAZE_SIZE; x++) {
      for(int y=0; y<MAZE_SIZE; y++) {
        if(distances[x][y] == 0) continue; 
        
        int minNeighbor = 255;
        if((paths[x][y] & 1) && y < MAZE_SIZE-1) minNeighbor = min(minNeighbor, distances[x][y+1]); // North
        if((paths[x][y] & 2) && x < MAZE_SIZE-1) minNeighbor = min(minNeighbor, distances[x+1][y]); // East
        if((paths[x][y] & 4) && y > 0) minNeighbor = min(minNeighbor, distances[x][y-1]); // South
        if((paths[x][y] & 8) && x > 0) minNeighbor = min(minNeighbor, distances[x-1][y]); // West

        if(minNeighbor != 255 && distances[x][y] != minNeighbor + 1) {
          distances[x][y] = minNeighbor + 1;
          changed = true;
        }
      }
    }
  }
}

int getNextMove() {
  int bestHeading = heading;
  int minDist = 255;

  if ((paths[rx][ry] & 1) && ry < MAZE_SIZE-1 && distances[rx][ry+1] < minDist) { minDist = distances[rx][ry+1]; bestHeading = 0; } // North
  if ((paths[rx][ry] & 2) && rx < MAZE_SIZE-1 && distances[rx+1][ry] < minDist) { minDist = distances[rx+1][ry]; bestHeading = 1; } // East
  if ((paths[rx][ry] & 4) && ry > 0 && distances[rx][ry-1] < minDist) { minDist = distances[rx][ry-1]; bestHeading = 2; } // South
  if ((paths[rx][ry] & 8) && rx > 0 && distances[rx-1][ry] < minDist) { minDist = distances[rx-1][ry]; bestHeading = 3; } // West

  return bestHeading;
}

void executeMove(int targetHeading, bool wasVirtual) {
  int turnDiff = (targetHeading - heading);
  if (turnDiff < 0) turnDiff += 4; 

  if (turnDiff == 1) {
    sharpTurn(false); // Right Turn
  } else if (turnDiff == 3) {
    sharpTurn(true);  // Left Turn
  } else if (turnDiff == 2) {
    uTurn();          // U-Turn
  } else if (turnDiff == 0 && !wasVirtual) {
    setMotorSpeed(BASE_SPEED, BASE_SPEED);
    delay(CLEARANCE_TIME); // Only clear if it was a physical junction
  }

  // Update facing direction
  heading = targetHeading;

  // Update physical coordinates
  if (heading == 0) ry++;
  else if (heading == 1) rx++;
  else if (heading == 2) ry--;
  else if (heading == 3) rx--;
}