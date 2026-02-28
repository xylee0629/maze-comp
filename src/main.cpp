// Floodfill algorithm version

#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ============================================================
// TWEAKABLE VARIABLES
// ============================================================
#define MAZE_SIZE 6 // 5x5 grid (Coordinates 0 to 4)
// --- MAZE COORDINATES ---
const int START_X = 0; // Robot's starting X coordinate
const int START_Y = 0; // Robot's starting Y coordinate
const int GOAL_X  = MAZE_SIZE - 1; // Target X coordinate for the finish box
const int GOAL_Y  = MAZE_SIZE - 1; // Target Y coordinate for the finish box

// --- GRID & JUNCTION HANDLING ---
const float GRID_CELL_CM = 25.0;   // Distance of ONE full maze square (center to center)
const float JUNCTION_PUSH_CM = 14; // Distance from IR sensors to wheel axle
const float TURN_90_CM = 9;        // (Wheelbase_in_cm * 3.1415) / 4

// --- CALIBRATED SPEEDS ---
const int L_BASE  = 120;  // Safe speed for exploration and PID mapping
const int R_BASE  = 120; 
const int L_DASH  = 150;  // Maximum speed for Dash Mode (Speedrun)
const int R_DASH  = 150;
const int L_PIVOT = 200;
const int R_PIVOT = 200;
const int SEEK_SPEED = 120; 

// --- PID CONTROL SETTINGS ---
float Kp = 0.08;  
float Kd = 0.5;   
int lastError = 0;
const int WHITE_VALUE = 300; 

// --- IR SENSOR DETECTION THRESHOLD ---
const int THRESHOLD_LINE     = 750;  // Sensors 1-4: line following
const int THRESHOLD_JUNCTION = 700;  // Sensors 0 & 5: junction detection

// --- TIME SETTINGS ---
const int JUNCTION_PUSH_TIMEOUT = 3000; 
const int PIVOT_TURN_TIMEOUT = 3000;    
const int U_TURN_SPIN_TIMEOUT = 4000;   
const int U_TURN_SEEK_TIMEOUT = 2000;   


// ============================================================
// PIN CONNECTIONS & CONSTANTS
// ============================================================

// Motor Driver Pins
#define AIN1 26
#define AIN2 25
#define PWMA 27
#define BIN1 21
#define BIN2 22
#define PWMB 23
#define STBY 4
const int SENSOR_PINS[6] = {33, 32, 35, 34, 39, 36}; 

// Encoder Pins 
#define ENC_L_A 16  
#define ENC_R_A 18  

// Button & LED Pins 
#define BTN_STRATEGY 12 
#define BTN_MODE     13 
#define BTN_START    14 
#define LED_STRATEGY 5  
#define LED_MODE     15 
#define LED_START    2

// --- ENCODER CONSTANTS ---
const float TICKS_PER_CM_L = 49.5;
const float TICKS_PER_CM_R = 45.4;
volatile long leftTicks = 0;
volatile long rightTicks = 0;
volatile unsigned long lastLeftTick  = 0;
volatile unsigned long lastRightTick = 0;

// --- STATE MACHINE VARIABLES ---
int runMode = 0;             // 0 = Explore, 1 = Dash
bool isRunning = false;      // false = Idle, true = Driving
int sensorValues[6];

// ============================================================
// FLOODFILL MEMORY & GRID SETTINGS
// ============================================================
// ============================================================
// FLOODFILL MEMORY & GRID SETTINGS
// ============================================================

// --- NEW BFS DIRECTIONAL ARRAYS ---
// Compass: 0=North, 1=East, 2=South, 3=West
const byte DIR_WALL[4] = {1, 2, 4, 8}; 
const int  DIR_DX[4]   = {0, 1, 0, -1};
const int  DIR_DY[4]   = {1, 0, -1, 0};

byte walls[MAZE_SIZE][MAZE_SIZE];     
int distances[MAZE_SIZE][MAZE_SIZE]; 
// ...

int posX = START_X; // Robot's physical X coordinate
int posY = START_Y; // Robot's physical Y coordinate

// Compass: 0 = North (+Y), 1 = East (+X), 2 = South (-Y), 3 = West (-X)
int heading = 0; 

// Target tracking for multiple runs
int targetX = GOAL_X; 
int targetY = GOAL_Y; 

// --- MAZE STATE ---
int  blackBoxCount    = 0;       
bool currentlyOnBox   = false; 

// --- FUNCTION DECLARATIONS ---
void readSensors();
void setMotorSpeed(int left, int right);
void handleJunction(bool leftDetected, bool rightDetected);
void encoderPivot(bool leftTurn);
void executeUTurn();
void initMaze();
void updateWalls(bool leftOpen, bool straightOpen, bool rightOpen);
void floodFill();
void printMaze();


void IRAM_ATTR countLeft() { 
  unsigned long now = micros();
  if (now - lastLeftTick > 300) { 
    leftTicks++; lastLeftTick = now; 
  }
}

void IRAM_ATTR countRight() { 
  unsigned long now = micros();
  if (now - lastRightTick > 300) { 
    rightTicks++; lastRightTick = now; 
  }
}

// ============================================================
void setup() {
// ============================================================
  Serial.begin(500000);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH); 

  for (int i = 0; i < 6; i++) pinMode(SENSOR_PINS[i], INPUT);

  pinMode(BTN_STRATEGY, INPUT_PULLUP);
  pinMode(BTN_MODE,     INPUT_PULLUP);
  pinMode(BTN_START,    INPUT_PULLUP);
  pinMode(LED_STRATEGY, OUTPUT);
  pinMode(LED_MODE,     OUTPUT);
  digitalWrite(LED_STRATEGY, LOW); 
  digitalWrite(LED_MODE,     LOW);  

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), countLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), countRight, RISING);

  initMaze();
  printMaze();
  Serial.println("System Ready. Waiting for physical button press...");
}

// ============================================================
void loop() {
// ============================================================
  if (digitalRead(BTN_MODE) == LOW) {
    runMode = (runMode == 0) ? 1 : 0;
    digitalWrite(LED_MODE, (runMode == 1) ? HIGH : LOW);
    Serial.print("Mode: "); Serial.println(runMode == 0 ? "EXPLORE" : "DASH");
    delay(300); 
  }

  if (digitalRead(BTN_START) == LOW) {
    isRunning = !isRunning;
    if (isRunning) {
      // --- FULL STATE RESET ---
      portDISABLE_INTERRUPTS();
      leftTicks = 0; rightTicks = 0; 
      portENABLE_INTERRUPTS();
      
      blackBoxCount = 0; currentlyOnBox = false;
      posX = START_X; posY = START_Y; heading = 0;
      
      if (runMode == 0) {
        Serial.println("--- EXPLORATION STARTED ---");
        targetX = GOAL_X; targetY = GOAL_Y;
        initMaze(); 
      } else {
        Serial.println("--- DASH STARTED ---");
        floodFill(); 
      }
    } else {
      setMotorSpeed(0, 0);
      Serial.println("--- RUN STOPPED ---");
    }
    delay(300); 
  }

  if (!isRunning) { setMotorSpeed(0, 0); return; }

  readSensors();
  
  // --- PRIORITY 0: START / FINISH BOX CHECK ---
  bool allBlack = true;
  for (int i = 0; i < 6; i++) {
    if (sensorValues[i] < THRESHOLD_LINE) { allBlack = false; break; }
  }

  if (allBlack) {
    if (!currentlyOnBox) {
      currentlyOnBox = true;
      blackBoxCount++;
      
      // THE BLIND PUSH: Clear the Start Box
      if (blackBoxCount == 1) {
        Serial.println("Pushing off Start Box blind...");
        int spdL = (runMode == 1) ? L_DASH : L_BASE;
        int spdR = (runMode == 1) ? R_DASH : R_BASE;
        setMotorSpeed(spdL, spdR);
        
        delay(400); // Shorter delay to not overshoot the 18cm mark!
        
        // RESET TICKS so we measure perfectly to the next grid cell!
        portDISABLE_INTERRUPTS();
        leftTicks = 0; rightTicks = 0;
        portENABLE_INTERRUPTS();
        return;
      }
    }
    
    if (blackBoxCount >= 2) {
      setMotorSpeed(0, 0); isRunning = false; 
      Serial.println("MAZE SOLVED! (Loop Check)");
      for (int f = 0; f < 6; f++) { digitalWrite(LED_MODE, f % 2); delay(200); }
      digitalWrite(LED_MODE, (runMode == 1) ? HIGH : LOW);
      return; 
    } else {
      int spdL = (runMode == 1) ? L_DASH : L_BASE;
      int spdR = (runMode == 1) ? R_DASH : R_BASE;
      setMotorSpeed(spdL, spdR);
      return; 
    }
  } else {
    currentlyOnBox = false; 
  }

  // --- PRIORITY 2: PHYSICAL JUNCTION CHECK ---
  bool leftJunction  = (sensorValues[0] > THRESHOLD_JUNCTION);
  bool rightJunction = (sensorValues[5] > THRESHOLD_JUNCTION);
  bool onLine = (sensorValues[2] > THRESHOLD_LINE) || (sensorValues[3] > THRESHOLD_LINE)
             || (sensorValues[1] > THRESHOLD_LINE) || (sensorValues[4] > THRESHOLD_LINE);

  if (onLine && (leftJunction || rightJunction)) {
    handleJunction(leftJunction, rightJunction);
    return; 
  }

  // --- PRIORITY 2.5: PHANTOM JUNCTION (ENCODER GRID TRACKER) ---
  long targetPhantomL = (long)(GRID_CELL_CM * TICKS_PER_CM_L);
  long targetPhantomR = (long)(GRID_CELL_CM * TICKS_PER_CM_R);

  portDISABLE_INTERRUPTS();
  long curL = leftTicks; long curR = rightTicks;
  portENABLE_INTERRUPTS();

  // If we traveled a full cell distance without seeing a physical cross
  if (curL >= targetPhantomL || curR >= targetPhantomR) {
    Serial.println("\n>>> PHANTOM JUNCTION (Straight Line Node) <<<");

    // 1. Arrive at node: Update Coordinate (Uncapped!)
    if (heading == 0) posY++;
    if (heading == 1) posX++;
    if (heading == 2) posY--;
    if (heading == 3) posX--;

    // --- EMERGENCY BOUNDARY STOP ---
    if (posX < 0 || posX >= MAZE_SIZE || posY < 0 || posY >= MAZE_SIZE) {
      setMotorSpeed(0, 0); isRunning = false;   
      Serial.println("\n====================================");
      Serial.printf("[CRITICAL ERROR] PHANTOM DROVE OFF MAP!\n");
      Serial.printf("Invalid Coordinate: (%d, %d)\n", posX, posY);
      Serial.println("====================================");
      while (true) { digitalWrite(LED_MODE, HIGH); delay(100); digitalWrite(LED_MODE, LOW); delay(100); }
    }

    // 2. Check if we hit the goal via a phantom straight!
    if (posX == targetX && posY == targetY) {
      Serial.println("MAZE SOLVED! (On Phantom Junction)");
      setMotorSpeed(0,0); isRunning = false;
      for (int f = 0; f < 6; f++) { digitalWrite(LED_MODE, f % 2); delay(200); }
      digitalWrite(LED_MODE, (runMode == 1) ? HIGH : LOW);

      if (targetX == GOAL_X && targetY == GOAL_Y) { 
        targetX = START_X; targetY = START_Y; 
      } else { 
        targetX = GOAL_X; targetY = GOAL_Y; 
      }
      floodFill(); printMaze();
      return;
    }

    // 3. Map Update (Left/Right are walls, Straight is open)
    if (runMode == 0) {
      updateWalls(false, true, false);
      floodFill();
    }

    // 4. Reset Ticks to measure distance to the next cell perfectly
    portDISABLE_INTERRUPTS();
    leftTicks = 0; rightTicks = 0;
    portENABLE_INTERRUPTS();
    
    // Do NOT return here! Let Priority 3 keep driving the motors!
  }

  // --- PRIORITY 3: PID LINE FOLLOWING ---
  bool isLineLost = true;
  for (int i = 0; i < 6; i++) {
    if (sensorValues[i] > THRESHOLD_LINE) { isLineLost = false; break; }
  }
  
  if (isLineLost) {
    handleJunction(false, false); 
    return;
  }

  int weights[6] = {-2500, -1500, -500, 500, 1500, 2500};
  long weightedSum = 0; long sum = 0;

  for (int i = 0; i < 6; i++) {
    int val = sensorValues[i] - WHITE_VALUE; 
    if (val < 0) val = 0; 
    weightedSum += (long)val * weights[i];
    sum += val;
  }

  int error = (sum > 50) ? (weightedSum / sum) : lastError;
  int motorCorrection = (Kp * error) + (Kd * (error - lastError));
  lastError = error;

  int currentBaseL = (runMode == 1) ? L_DASH : L_BASE;
  int currentBaseR = (runMode == 1) ? R_DASH : R_BASE;

  int currentThrottleL = currentBaseL - abs(error * 0.05); 
  int currentThrottleR = currentBaseR - abs(error * 0.05);
  
  if (currentThrottleL < 100) currentThrottleL = 100;
  if (currentThrottleR < 100) currentThrottleR = 100;

  int leftSpeed = constrain(currentThrottleL + motorCorrection, -255, 255);
  int rightSpeed = constrain(currentThrottleR - motorCorrection, -255, 255);

  setMotorSpeed(leftSpeed, rightSpeed);
} // End of loop()


// ============================================================
// FLOODFILL FUNCTIONS
// ============================================================

void initMaze() {
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      walls[x][y] = 0; 
      
      if (y == MAZE_SIZE - 1) walls[x][y] |= 1; // North
      if (x == MAZE_SIZE - 1) walls[x][y] |= 2; // East
      if (y == 0)             walls[x][y] |= 4; // South
      if (x == 0)             walls[x][y] |= 8; // West
      
      distances[x][y] = abs(x - targetX) + abs(y - targetY);
    }
  }
  
  // Use dynamically mapped Starting Coordinates
  walls[START_X][START_Y] |= 8; // Start Box Left wall
  walls[START_X][START_Y] |= 2; // Start Box Right wall
}

void updateWalls(bool leftOpen, bool straightOpen, bool rightOpen) {
  bool pathN = false, pathE = false, pathS = false, pathW = false;

  if (heading == 0) { pathN = straightOpen; pathE = rightOpen;    pathW = leftOpen;     pathS = true; }
  else if (heading == 1) { pathE = straightOpen; pathS = rightOpen;    pathN = leftOpen;     pathW = true; }
  else if (heading == 2) { pathS = straightOpen; pathW = rightOpen;    pathE = leftOpen;     pathN = true; }
  else if (heading == 3) { pathW = straightOpen; pathN = rightOpen;    pathS = leftOpen;     pathE = true; }

  // Symmetrical Wall Writing
  if (!pathN) { walls[posX][posY] |= 1; if (posY < MAZE_SIZE - 1) walls[posX][posY+1] |= 4; }
  if (!pathE) { walls[posX][posY] |= 2; if (posX < MAZE_SIZE - 1) walls[posX+1][posY] |= 8; }
  if (!pathS) { walls[posX][posY] |= 4; if (posY > 0)         walls[posX][posY-1] |= 1; }
  if (!pathW) { walls[posX][posY] |= 8; if (posX > 0)         walls[posX-1][posY] |= 2; }
}

void floodFill() {
  // 1. Reset all distances to 255 (Unreachable)
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      distances[x][y] = 255;
    }
  }

  // 2. Simple array-based queue (safe on ESP32 SRAM)
  int qx[MAZE_SIZE * MAZE_SIZE];
  int qy[MAZE_SIZE * MAZE_SIZE];
  int head = 0, tail = 0;

  // 3. Seed the dynamic target (Goal or Start Box)
  distances[targetX][targetY] = 0;
  qx[tail] = targetX;
  qy[tail] = targetY;
  tail++;

  // 4. Run the Breadth-First Search
  while (head < tail) {
    int cx = qx[head];
    int cy = qy[head];
    head++;

    // Check all 4 compass directions
    for (int d = 0; d < 4; d++) {
      // Skip if a wall blocks this specific direction
      if (walls[cx][cy] & DIR_WALL[d]) continue;

      int nx = cx + DIR_DX[d];
      int ny = cy + DIR_DY[d];

      // Skip if the path leads out of bounds
      if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;

      // Only update and queue if we found a strictly shorter path
      if (distances[nx][ny] > distances[cx][cy] + 1) {
        distances[nx][ny] = distances[cx][cy] + 1;
        qx[tail] = nx;
        qy[tail] = ny;
        tail++;
      }
    }
  }
}

void printMaze() {
  Serial.println("\n==== FLOODFILL MAP ====");
  for (int y = MAZE_SIZE - 1; y >= 0; y--) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      if (x == posX && y == posY) {
        if (heading == 0) Serial.print("[^]"); 
        if (heading == 1) Serial.print("[>]"); 
        if (heading == 2) Serial.print("[v]"); 
        if (heading == 3) Serial.print("[<]"); 
      } 
      else if (x == targetX && y == targetY) Serial.print("[G]"); 
      else Serial.printf("%3d", distances[x][y]); 
    }
    Serial.println(); 
  }
  Serial.println("=======================\n");
}


// ============================================================
// HARDWARE FUNCTIONS
// ============================================================

void readSensors() {
  for (int i = 0; i < 6; i++) sensorValues[i] = analogRead(SENSOR_PINS[i]);
}

void setMotorSpeed(int left, int right) {
  digitalWrite(STBY, HIGH);
  if (left >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); } 
  else { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); }
  analogWrite(PWMA, abs(left));

  if (right >= 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); } 
  else { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); }
  analogWrite(PWMB, abs(right));
}

void encoderPivot(bool leftTurn) {
  Serial.println("--- PIVOT STARTED ---");
  portDISABLE_INTERRUPTS();
  leftTicks  = 0; rightTicks = 0;
  portENABLE_INTERRUPTS();

  long targetL = (long)(TURN_90_CM * TICKS_PER_CM_L);
  long targetR = (long)(TURN_90_CM * TICKS_PER_CM_R);
  unsigned long pivotStart = millis();

  int lastLSpeed = 0; int lastRSpeed = 0;

  // ==========================================
  // PHASE 1: Fast Encoder Spin 
  // ==========================================
  Serial.println("Phase 1: Encoder Spin...");
  
  while (true) {
    portDISABLE_INTERRUPTS();
    long curL = leftTicks; long curR = rightTicks;
    portENABLE_INTERRUPTS();

    if (curL >= targetL && curR >= targetR) {
      Serial.println("Phase 1 Success: Target reached.");
      break;
    }
    if (millis() - pivotStart > PIVOT_TURN_TIMEOUT) {
      Serial.println("ERROR: Phase 1 Timed Out! (Are the encoders ticking?)");
      break;
    }

    int currentLSpeed = 0;
    int currentRSpeed = 0;
    
    // Restored exact original logic
    if (leftTurn) {
      currentLSpeed = (curL < targetL) ? -L_PIVOT : 0;
      currentRSpeed = (curR < targetR) ?  R_PIVOT : 0;
    } else {
      currentLSpeed = (curL < targetL) ?  L_PIVOT : 0;
      currentRSpeed = (curR < targetR) ? -R_PIVOT : 0;
    }

    if (currentLSpeed != lastLSpeed || currentRSpeed != lastRSpeed) {
      setMotorSpeed(currentLSpeed, currentRSpeed);
      lastLSpeed = currentLSpeed; lastRSpeed = currentRSpeed;
    }
    delay(1); 
  }

  setMotorSpeed(0, 0); delay(50); 
  
  // PHASE 2: Slow seek — creep in the same direction until centre sensors find the line
  unsigned long seekStart = millis();

  while (true) {
    readSensors();

    // Centre sensors confirm we're on the line — stop immediately
    if (sensorValues[2] > THRESHOLD_LINE || sensorValues[3] > THRESHOLD_LINE) {
      Serial.println("--- LINE FOUND ---");
      break;
    }

    // Safety timeout — if line not found within timeout
    if (millis() - seekStart > PIVOT_TURN_TIMEOUT) {
      Serial.println("WARNING: Seek timed out");
      break;
    }

    // Keep rotating slowly in the same direction as the original turn
    if (leftTurn) {
      setMotorSpeed(-SEEK_SPEED, SEEK_SPEED);
    } else {
      setMotorSpeed(SEEK_SPEED, -SEEK_SPEED);
    }
    delay(1);
  }
  setMotorSpeed(0, 0); delay(30);
  Serial.println("--- PIVOT FINISHED ---");
}

void executeUTurn() {
  Serial.println("--- U-TURN STARTED ---");
  portDISABLE_INTERRUPTS();
  leftTicks  = 0; rightTicks = 0;
  portENABLE_INTERRUPTS();

  long targetSpinL = (long)(TURN_90_CM * 2.0f * TICKS_PER_CM_L);
  long targetSpinR = (long)(TURN_90_CM * 2.0f * TICKS_PER_CM_R);
  unsigned long spinStart = millis();

  int lastSpinL = 0; int lastSpinR = 0;

  while (true) {
    portDISABLE_INTERRUPTS();
    long curL = leftTicks; long curR = rightTicks;
    portENABLE_INTERRUPTS();

    if (curL >= targetSpinL && curR >= targetSpinR) break;
    if (millis() - spinStart > U_TURN_SPIN_TIMEOUT) break;

    int currentLSpeed = (curL < targetSpinL) ?  L_PIVOT : 0;   
    int currentRSpeed = (curR < targetSpinR) ? -R_PIVOT : 0;   

    if (currentLSpeed != lastSpinL || currentRSpeed != lastSpinR) {
      setMotorSpeed(currentLSpeed, currentRSpeed);
      lastSpinL = currentLSpeed; lastSpinR = currentRSpeed;
    }
    delay(1);
  }

  setMotorSpeed(0, 0); delay(50); 
  unsigned long seekStart = millis();
  setMotorSpeed(SEEK_SPEED, -SEEK_SPEED); 

  while (true) {
    readSensors();
    if (sensorValues[2] > THRESHOLD_LINE || sensorValues[3] > THRESHOLD_LINE) break;
    if (millis() - seekStart > U_TURN_SEEK_TIMEOUT) break;
    delay(1);
  }

  setMotorSpeed(0, 0); delay(30);
}


// ------------------------------------------------------------
// THE UNIVERSAL JUNCTION BRAIN (Floodfill Edition)
// ------------------------------------------------------------
void handleJunction(bool leftDetected, bool rightDetected) {
  Serial.println("\n>>> PHYSICAL JUNCTION DETECTED <<<");
  
  // 1. Stop and let the physical chassis settle
  setMotorSpeed(0, 0); delay(50); 

  // 2. Read the sensors one final time while perfectly still
  // This catches the crossbar even if the robot hit it diagonally
  readSensors();
  bool canGoLeft     = leftDetected || (sensorValues[0] > THRESHOLD_JUNCTION);
  bool canGoRight    = rightDetected || (sensorValues[5] > THRESHOLD_JUNCTION);
  bool canGoStraight = false;

  portDISABLE_INTERRUPTS();
  leftTicks  = 0; rightTicks = 0;
  portENABLE_INTERRUPTS();

  long targetPushL = (long)(JUNCTION_PUSH_CM * TICKS_PER_CM_L);
  long targetPushR = (long)(JUNCTION_PUSH_CM * TICKS_PER_CM_R);

  Serial.println("Pushing forward to axle...");
  unsigned long pushStart = millis();
  int lastPushL = 0; int lastPushR = 0;

  // 3. The 14cm push
  while (true) {
    portDISABLE_INTERRUPTS();
    long curL = leftTicks; long curR = rightTicks;
    portENABLE_INTERRUPTS();

    if (curL >= targetPushL && curR >= targetPushR) break;
    if (millis() - pushStart > JUNCTION_PUSH_TIMEOUT) break; 

    int lSpeed = (curL < targetPushL) ? L_BASE : 0;
    int rSpeed = (curR < targetPushR) ? R_BASE : 0;

    if (lSpeed != lastPushL || rSpeed != lastPushR) {
      setMotorSpeed(lSpeed, rSpeed);
      lastPushL = lSpeed; lastPushR = rSpeed;
    }

    // *** DELETED THE CONTINUOUS SENSOR SCANNING FROM HERE! ***
    // The robot will no longer hallucinate paths while pushing.
    
    delay(1);
  } 

  setMotorSpeed(0, 0); delay(50);
  readSensors();
  
// ... (after the push while-loop and stopping the motors)
  setMotorSpeed(0, 0); delay(50);
  readSensors();

  // 1. ARRIVE AT NODE: Update Coordinate FIRST!
  if (heading == 0) posY++;
  if (heading == 1) posX++;
  if (heading == 2) posY--;
  if (heading == 3) posX--;

  if (posX < 0 || posX >= MAZE_SIZE || posY < 0 || posY >= MAZE_SIZE) {
    setMotorSpeed(0, 0); isRunning = false;   
    Serial.printf("\n[CRITICAL ERROR] Invalid Coordinate: (%d, %d)\n", posX, posY);
    while (true) { digitalWrite(LED_MODE, HIGH); delay(100); digitalWrite(LED_MODE, LOW); delay(100); }
  }

  // 2. NOW check for the Finish Box
  bool allBlackBox = true;
  for (int i = 0; i < 6; i++) {
    if (sensorValues[i] < THRESHOLD_LINE) { allBlackBox = false; break; }
  }

  if (allBlackBox) {
    Serial.println("MAZE SOLVED!");
// ... (rest of the finish box logic stays the same)
    isRunning = false; 
    
    for (int f = 0; f < 6; f++) { digitalWrite(LED_MODE, f % 2); delay(200); }
    digitalWrite(LED_MODE, (runMode == 1) ? HIGH : LOW);
    
    if (targetX == GOAL_X && targetY == GOAL_Y) { 
        targetX = START_X; targetY = START_Y; 
    } else { 
        targetX = GOAL_X; targetY = GOAL_Y; 
    }
    
    floodFill(); printMaze();
    return; 
  }

  // --- EMERGENCY BOUNDARY STOP ---
  if (posX < 0 || posX >= MAZE_SIZE || posY < 0 || posY >= MAZE_SIZE) {
    setMotorSpeed(0, 0); isRunning = false;   
    Serial.println("\n====================================");
    Serial.printf("[CRITICAL ERROR] DROVE OFF THE MAP!\n");
    Serial.printf("Invalid Coordinate Reached: (%d, %d)\n", posX, posY);
    Serial.println("====================================");
    while (true) { digitalWrite(LED_MODE, HIGH); delay(100); digitalWrite(LED_MODE, LOW); delay(100); }
  }

  canGoStraight = (sensorValues[2] > THRESHOLD_LINE || sensorValues[3] > THRESHOLD_LINE);

  // ==========================================
  // SPLIT: EXPLORE MODE VS DASH MODE
  // ==========================================
  if (runMode == 0) {
    // 2. We are Mapping: Update the walls physically seen for this new square
    updateWalls(canGoLeft, canGoStraight, canGoRight);
    
    // 3. Run the heavy math calculation
    floodFill();
    
    // 4. Print the brain to the screen
    printMaze();
  } 

  // ==========================================
  // NAVIGATION: Follow the lowest distance
  // ==========================================
  int nextHeading = heading;
  int minVal = 999;

  // Check North (0)
  if (!(walls[posX][posY] & 1) && distances[posX][posY+1] < minVal) {
    minVal = distances[posX][posY+1]; nextHeading = 0;
  }
  // Check East (1)
  if (!(walls[posX][posY] & 2) && distances[posX+1][posY] < minVal) {
    minVal = distances[posX+1][posY]; nextHeading = 1;
  }
  // Check South (2)
  if (!(walls[posX][posY] & 4) && distances[posX][posY-1] < minVal) {
    minVal = distances[posX][posY-1]; nextHeading = 2;
  }
  // Check West (3)
  if (!(walls[posX][posY] & 8) && distances[posX-1][posY] < minVal) {
    minVal = distances[posX-1][posY]; nextHeading = 3;
  }

  // Determine physical turn direction
  int turnDiff = (nextHeading - heading + 4) % 4;

  if (turnDiff == 1) {
    Serial.println("Decision: EAST (Right)");
    encoderPivot(false);
  } else if (turnDiff == 3) {
    Serial.println("Decision: WEST (Left)");
    encoderPivot(true);
  } else if (turnDiff == 2) {
    Serial.println("Decision: SOUTH (U-Turn)");
    executeUTurn();
  } else {
    Serial.println("Decision: NORTH (Straight)");
    int spdL = (runMode == 1) ? L_DASH : L_BASE;
    int spdR = (runMode == 1) ? R_DASH : R_BASE;
    setMotorSpeed(spdL, spdR); 
    delay(150); 
  }

  // ======================================================
  // 5. DEPARTURE: Commit to New Heading and Reset Distance
  // ======================================================
  heading = nextHeading;

  // --- NEW: Departure Kickstart ---
  // Force the motors ON to physically clear the intersection crosshair
  int kickSpd = (runMode == 1) ? L_DASH : L_BASE;
  setMotorSpeed(kickSpd, kickSpd);
  delay(10); // Drive blindly for 0.1s to escape the junction tape

  // Wipe the encoders using safe Arduino core interrupts
  noInterrupts();
  leftTicks = 0; rightTicks = 0;
  interrupts();
} // End of handleJunction