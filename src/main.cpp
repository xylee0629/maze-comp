// PID with left hand and right hand seperate memory, non-stable

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

// --- ENCODER PINS ---
#define ENC_L_A 16  
#define ENC_R_A 18  

// --- BUTTON PINS ---
#define BTN_STRATEGY 12 
#define BTN_MODE     13 
#define BTN_START    14 

// --- LED INDICATOR PINS ---
#define LED_STRATEGY 5  // ON = Left-Hand Rule, OFF = Right-Hand Rule
#define LED_MODE     15 // OFF = Explore Mode, ON = Dash Mode

// --- ENCODER CONSTANTS ---
const float TICKS_PER_CM_L = 49.5;
const float TICKS_PER_CM_R = 45.4;

// !! IMPORTANT !! MEASURE YOUR ROBOT AND CHANGE THESE TWO NUMBERS:
const float JUNCTION_PUSH_CM = 14; // Distance from IR sensors to wheel axle in cm
const float TURN_90_CM = 9;       // (Wheelbase_in_cm * 3.1415) / 4

// --- ENCODER COUNTERS ---
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// --- STATE MACHINE VARIABLES ---
bool useLeftHandRule = true; // true = Left bias, false = Right bias
int runMode = 0;             // 0 = Explore, 1 = Dash
bool isRunning = false;      // false = Idle, true = Driving

// --- CALIBRATED SPEEDS ---
const int L_BASE  = 180;  // Matched from working line-follower test
const int R_BASE  = 180; 
const int L_NUDGE = 80;   // Sharper correction from working line-follower test
const int R_NUDGE = 80;  
const int L_PIVOT = 200;
const int R_PIVOT = 200;

// --- PID CONTROL SETTINGS ---
float Kp = 0.08;  // Proportional: How hard to steer based on current error
float Kd = 0.5;   // Derivative: How hard to resist sudden changes (dampening)
int lastError = 0;

// --- SETTINGS ---
// Split threshold: line-follow needs to be sensitive (750),
// junction outer sensors can afford to be stricter (800) to avoid false triggers.
const int THRESHOLD_LINE     = 750;  // Sensors 1-4: line following
const int THRESHOLD_JUNCTION = 700;  // Sensors 0 & 5: junction detection
const int BLIND_TURN_TIME = 200; 
const int DEAD_END_LIMIT  = 50;  
int whiteCount = 0;
int sensorValues[6];

// --- MEMORY MATRIX SETTINGS ---
#define MAX_PATH 100          

// Left-Hand Memory Bank
char leftPathMemory[MAX_PATH];    
int leftPathLength = 0;    
int leftDashIndex = 0;       

// Right-Hand Memory Bank
char rightPathMemory[MAX_PATH];    
int rightPathLength = 0;    
int rightDashIndex = 0; 

// --- MAZE STATE ---
int  blackBoxCount    = 0;       
bool currentlyOnBox   = false; 

// --- FUNCTION DECLARATIONS ---
void readSensors();
void setMotorSpeed(int left, int right);
void handleJunction(bool leftDetected, bool rightDetected);
void encoderPivot(bool leftTurn);
void recordTurn(char turn);
void executeUTurn();
void simplifyPath();

// --- INTERRUPT SERVICE ROUTINES (ISRs) ---
volatile unsigned long lastLeftTick  = 0;
volatile unsigned long lastRightTick = 0;

void IRAM_ATTR countLeft() { 
  unsigned long now = micros();
  if (now - lastLeftTick > 300) { 
    leftTicks++; 
    lastLeftTick = now; 
  }
}

void IRAM_ATTR countRight() { 
  unsigned long now = micros();
  if (now - lastRightTick > 300) { 
    rightTicks++; 
    lastRightTick = now; 
  }
}

// ============================================================
void setup() {
// ============================================================
  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);

  // Motor Setup
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); // Motors enabled from the start

  // Sensor Setup
  for (int i = 0; i < 6; i++) pinMode(SENSOR_PINS[i], INPUT);

  // Button Setup
  pinMode(BTN_STRATEGY, INPUT_PULLUP);
  pinMode(BTN_MODE,     INPUT_PULLUP);
  pinMode(BTN_START,    INPUT_PULLUP);

  // LED Setup
  pinMode(LED_STRATEGY, OUTPUT);
  pinMode(LED_MODE,     OUTPUT);
  digitalWrite(LED_STRATEGY, HIGH); // Default: Left-Hand Rule
  digitalWrite(LED_MODE,     LOW);  // Default: Explore Mode

  // Encoder Setup
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), countLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), countRight, RISING);

}

// ============================================================
void loop() {
// ============================================================
  // ----------------------------------------------------------
  // 1. BUTTON READING & STATE MACHINE
  // ----------------------------------------------------------
  if (digitalRead(BTN_STRATEGY) == LOW) {
    useLeftHandRule = !useLeftHandRule;
    digitalWrite(LED_STRATEGY, useLeftHandRule ? HIGH : LOW);
    Serial.print("Strategy: "); Serial.println(useLeftHandRule ? "LEFT-HAND" : "RIGHT-HAND");
    delay(300); 
  }

  if (digitalRead(BTN_MODE) == LOW) {
    runMode = (runMode == 0) ? 1 : 0;
    digitalWrite(LED_MODE, (runMode == 1) ? HIGH : LOW);
    Serial.print("Mode: "); Serial.println(runMode == 0 ? "EXPLORE" : "DASH");
    delay(300); 
  }

if (digitalRead(BTN_START) == LOW) {
    isRunning = !isRunning;
    if (isRunning) {
      leftTicks = 0; rightTicks = 0; whiteCount = 0; blackBoxCount = 0;       // <--- THE FIX: Forget the old boxes!
      currentlyOnBox = false;
      
      // --- NEW: Reset variables based on mode & strategy ---
      if (runMode == 0) {
        // Clear memory for a new exploration on the active strategy
        if (useLeftHandRule) leftPathLength = 0;
        else rightPathLength = 0;
        Serial.println("--- EXPLORATION STARTED ---");
      } else {
        // Start reading from the beginning of the saved path for the active strategy
        if (useLeftHandRule) leftDashIndex = 0;
        else rightDashIndex = 0;
        Serial.println("--- DASH STARTED ---");
      }
      
    } else {
      setMotorSpeed(0, 0);
      Serial.println("--- RUN STOPPED ---");
    }
    delay(300); 
  }

  // ----------------------------------------------------------
  // 2. IDLE MODE
  // ----------------------------------------------------------
  if (!isRunning) {
    setMotorSpeed(0, 0); 
    return; 
  }

  // ----------------------------------------------------------
  // 3. DRIVING LOGIC
  // ----------------------------------------------------------
  readSensors();
  
  Serial.printf("Sensors: %d %d %d %d %d %d\n",
    sensorValues[0], sensorValues[1], sensorValues[2],
    sensorValues[3], sensorValues[4], sensorValues[5]);
  
// --- PRIORITY 0: START / FINISH BOX CHECK ---
  bool allBlack = true;
  for (int i = 0; i < 6; i++) {
    if (sensorValues[i] < THRESHOLD_LINE) { allBlack = false; break; }
  }

  if (allBlack) {
    if (!currentlyOnBox) {
      currentlyOnBox = true;
      blackBoxCount++;
      Serial.print("Black box count: "); Serial.println(blackBoxCount);
      
      // ==========================================
      // THE BLIND PUSH: Clear the Start Box
      // ==========================================
      if (blackBoxCount == 1) {
        Serial.println("Pushing off Start Box blind...");
        setMotorSpeed(L_BASE, R_BASE);
        
        // Disable sensors and drive straight for X milliseconds. 
        // Increase this number if it still catches the edge!
        delay(600); 
        return;
      }
    }
    
    if (blackBoxCount >= 2) {
      setMotorSpeed(0, 0); 
      isRunning = false; 
      
      Serial.print("MAZE SOLVED! ");
      if (useLeftHandRule) {
        Serial.print("Left Path: ");
        for (int i = 0; i < leftPathLength; i++) Serial.print(leftPathMemory[i]);
      } else {
        Serial.print("Right Path: ");
        for (int i = 0; i < rightPathLength; i++) Serial.print(rightPathMemory[i]);
      }
      Serial.println();
      
      // Victory LED Flash
      for (int f = 0; f < 6; f++) {
        digitalWrite(LED_STRATEGY, f % 2);
        digitalWrite(LED_MODE,     f % 2);
        delay(200);
      }
      
      // Restore LEDs to their actual mode states
      digitalWrite(LED_STRATEGY, useLeftHandRule ? HIGH : LOW);
      digitalWrite(LED_MODE, (runMode == 1) ? HIGH : LOW);
      
      return;
    } else {
      // If we are STILL somehow on the box, keep driving straight
      setMotorSpeed(L_BASE, R_BASE);
      return; 
    }
  } else {
    currentlyOnBox = false; 
  }

  // --- PRIORITY 2: JUNCTION CHECK ---
  // STRICT FIX: Only the far outer sensors can trigger a junction. 
  // Sensors 1 and 4 are strictly for steering only!
  bool leftJunction  = (sensorValues[0] > THRESHOLD_JUNCTION);
  bool rightJunction = (sensorValues[5] > THRESHOLD_JUNCTION);

  // Only treat as junction if centre sensors confirm we're on the main line,
  // not just riding the edge of a curve.
  bool onLine = (sensorValues[2] > THRESHOLD_LINE) || (sensorValues[3] > THRESHOLD_LINE)
             || (sensorValues[1] > THRESHOLD_LINE) || (sensorValues[4] > THRESHOLD_LINE);

  if (onLine && (leftJunction || rightJunction)) {
    handleJunction(leftJunction, rightJunction);
    return; 
  }

// --- PRIORITY 3: PID LINE FOLLOWING ---
  
  // Step 1: Check if the line is completely lost (Dead End fallback)
  bool isLineLost = true;
  for (int i = 0; i < 6; i++) {
    if (sensorValues[i] > THRESHOLD_LINE) { isLineLost = false; break; }
  }
  
  if (isLineLost) {
    handleJunction(false, false); 
    return;
  }

  // Step 2: Calculate the Weighted Average (Position Error)
  // Assign physical weights to sensors from left to right
  int weights[6] = {-2500, -1500, -500, 500, 1500, 2500};
  long weightedSum = 0;
  long sum = 0;

  for (int i = 0; i < 6; i++) {
    // Subtract a baseline (e.g., 300 for white floor) to clean up sensor noise
    int val = sensorValues[i] - 300; 
    if (val < 0) val = 0; 

    weightedSum += (long)val * weights[i];
    sum += val;
  }

  int error = 0;
  if (sum > 50) { // Prevent division by zero
    error = weightedSum / sum;
  } else {
    error = lastError; // If reading is weak, hold the last known error
  }

  // Step 3: Calculate PID Correction
  int motorCorrection = (Kp * error) + (Kd * (error - lastError));
  lastError = error;

  // --- NEW: Dynamic Auto-Braking ---
  // If the error is high, subtract speed from the base throttle
  // You can tune the '0.05' multiplier to make it brake harder on curves
  int currentThrottleL = L_BASE - abs(error * 0.05); 
  int currentThrottleR = R_BASE - abs(error * 0.05);
  
  // Prevent the throttle from dropping too low
  if (currentThrottleL < 100) currentThrottleL = 100;
  if (currentThrottleR < 100) currentThrottleR = 100;

  // Step 4: Apply to Motors
  int leftSpeed = currentThrottleL + motorCorrection;
  int rightSpeed = currentThrottleR - motorCorrection;

  // Ensure speeds do not exceed hardware limits
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  setMotorSpeed(leftSpeed, rightSpeed);
} // End of loop()

// ============================================================
// FUNCTIONS
// ============================================================

void readSensors() {
  for (int i = 0; i < 6; i++) sensorValues[i] = analogRead(SENSOR_PINS[i]);
}

void setMotorSpeed(int left, int right) {
  // Ensure motors are never disabled by a power hiccup
  digitalWrite(STBY, HIGH);

  if (left >= 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
  }
  analogWrite(PWMA, abs(left));

  if (right >= 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
  }
  analogWrite(PWMB, abs(right));
}

// ------------------------------------------------------------
// encoderPivot
//
// Phase 1: Fast encoder-based turn gets the robot close to 90°.
//          Uses TURN_90_CM to determine distance — tune this to
//          slightly undershoot so Phase 2 always seeks forward.
//
// Phase 2: Slow creep continues in the same turn direction until
//          centre sensors [2] or [3] land on the line.
//          This eliminates over/undershoot regardless of surface
//          friction or battery voltage variation.
// ------------------------------------------------------------
void encoderPivot(bool leftTurn) {
  Serial.println("--- PIVOT STARTED ---");

  // PHASE 1: Fast rough turn
  portDISABLE_INTERRUPTS();
  leftTicks  = 0;
  rightTicks = 0;
  portENABLE_INTERRUPTS();

  long targetL = (long)(TURN_90_CM * TICKS_PER_CM_L);
  long targetR = (long)(TURN_90_CM * TICKS_PER_CM_R);
  unsigned long pivotStart = millis();

  // Inside encoderPivot()...
  int lastLSpeed = 0; // Track the last speed sent to the motors
  int lastRSpeed = 0;

  while (true) {
    portDISABLE_INTERRUPTS();
    long curL = leftTicks;
    long curR = rightTicks;
    portENABLE_INTERRUPTS();

    if (curL >= targetL && curR >= targetR) break;
    if (millis() - pivotStart > 3000) break;

    // Calculate what the speed SHOULD be
    int currentLSpeed = 0;
    int currentRSpeed = 0;

    if (leftTurn) {
      currentLSpeed = (curL < targetL) ? -L_PIVOT : 0;
      currentRSpeed = (curR < targetR) ?  R_PIVOT : 0;
    } else {
      currentLSpeed = (curL < targetL) ?  L_PIVOT : 0;
      currentRSpeed = (curR < targetR) ? -R_PIVOT : 0;
    }

    // ONLY update the hardware if the speed has actually changed!
    if (currentLSpeed != lastLSpeed || currentRSpeed != lastRSpeed) {
      setMotorSpeed(currentLSpeed, currentRSpeed);
      lastLSpeed = currentLSpeed;
      lastRSpeed = currentRSpeed;
    }
    
    delay(1);
  }

  setMotorSpeed(0, 0);
  delay(50); // Let robot physically settle before sensing

  Serial.println("--- PHASE 1 DONE: Seeking line ---");

  // PHASE 2: Slow seek — creep in the same direction until centre sensors find the line
  const int SEEK_SPEED = 80;
  unsigned long seekStart = millis();

  while (true) {
    readSensors();

    // Centre sensors confirm we're on the line — stop immediately
    if (sensorValues[2] > THRESHOLD_LINE || sensorValues[3] > THRESHOLD_LINE) {
      Serial.println("--- LINE FOUND ---");
      break;
    }

    // Safety timeout — if line not found within 1.5s something is wrong
    if (millis() - seekStart > 3000) {
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

  setMotorSpeed(0, 0);
  delay(30);

  Serial.println("--- PIVOT FINISHED ---");
}

// ------------------------------------------------------------
// handleJunction — centres the robot then decides where to go
// ------------------------------------------------------------
void handleJunction(bool leftDetected, bool rightDetected) {
  Serial.println("\n>>> JUNCTION DETECTED <<<");
  setMotorSpeed(0, 0);
  delay(50);

  portDISABLE_INTERRUPTS();
  leftTicks  = 0;
  rightTicks = 0;
  portENABLE_INTERRUPTS();

  long targetPushL = (long)(JUNCTION_PUSH_CM * TICKS_PER_CM_L);
  long targetPushR = (long)(JUNCTION_PUSH_CM * TICKS_PER_CM_R);

  bool canGoLeft     = leftDetected;
  bool canGoRight    = rightDetected;
  bool canGoStraight = false;

  Serial.println("Pushing forward to axle...");
  unsigned long pushStart = millis();

  int lastPushL = 0;
  int lastPushR = 0;

while (true) {
    portDISABLE_INTERRUPTS();
    long curL = leftTicks;
    long curR = rightTicks;
    portENABLE_INTERRUPTS();

    if (curL >= targetPushL && curR >= targetPushR) break;
    if (millis() - pushStart > 3000) break;

    int lSpeed = (curL < targetPushL) ? L_BASE : 0;
    int rSpeed = (curR < targetPushR) ? R_BASE : 0;

    // ONLY update hardware if the speed changed
    if (lSpeed != lastPushL || rSpeed != lastPushR) {
      setMotorSpeed(lSpeed, rSpeed);
      lastPushL = lSpeed;
      lastPushR = rSpeed;
    }

    // Flawlessly scan for left/right paths the entire time
    readSensors();
    if (sensorValues[0] > THRESHOLD_JUNCTION) canGoLeft  = true;
    if (sensorValues[5] > THRESHOLD_JUNCTION) canGoRight = true;

    delay(1);
  } 

  // Robot is now centred at the intersection
  setMotorSpeed(0, 0); delay(50);
  readSensors();

  // ======================================================
  // THE ELEGANT FINISH BOX CHECK
  // ======================================================
  // If we pushed 14cm and all sensors are STILL black, 
  // it is physically impossible to be a normal junction tape.
  bool allBlackBox = true;
  for (int i = 0; i < 6; i++) {
    if (sensorValues[i] < THRESHOLD_LINE) { allBlackBox = false; break; }
  }

if (allBlackBox) {
    Serial.print("MAZE SOLVED! (Post-push) ");
    if (useLeftHandRule) {
      Serial.print("Left Path: ");
      for (int i = 0; i < leftPathLength; i++) Serial.print(leftPathMemory[i]);
    } else {
      Serial.print("Right Path: ");
      for (int i = 0; i < rightPathLength; i++) Serial.print(rightPathMemory[i]);
    }
    Serial.println();
    
    isRunning = false; 
    
    // Victory LED Flash
    for (int f = 0; f < 6; f++) {
      digitalWrite(LED_STRATEGY, f % 2);
      digitalWrite(LED_MODE,     f % 2);
      delay(200);
    }
    
    // Restore LEDs to their actual mode states
    digitalWrite(LED_STRATEGY, useLeftHandRule ? HIGH : LOW);
    digitalWrite(LED_MODE, (runMode == 1) ? HIGH : LOW);
    
    return; // Instantly abort the rest of the function!
  }

  // Robot is now centred at the intersection — safe to check straight
  setMotorSpeed(0, 0); delay(50);
  readSensors();
  canGoStraight = (sensorValues[2] > THRESHOLD_LINE || sensorValues[3] > THRESHOLD_LINE);

  int availablePaths = (canGoLeft ? 1 : 0) + (canGoRight ? 1 : 0) + (canGoStraight ? 1 : 0);
  bool isDecisionPoint = (availablePaths > 1);

  Serial.printf("Paths — L:%d  S:%d  R:%d\n", canGoLeft, canGoStraight, canGoRight);

// ==========================================
  // DASH MODE (Speedrun)
  // ==========================================
  if (runMode == 1) {
    if (isDecisionPoint) {
      // Dynamically point to the active memory bank based on the switch!
      char* activePath = useLeftHandRule ? leftPathMemory : rightPathMemory;
      int activeLen = useLeftHandRule ? leftPathLength : rightPathLength;
      int& activeIndex = useLeftHandRule ? leftDashIndex : rightDashIndex;

      // Ask memory what to do
      if (activeIndex < activeLen) {
        char nextMove = activePath[activeIndex];
        activeIndex++; 
        
        Serial.print("Dash Memory Read: "); Serial.println(nextMove);

        if (nextMove == 'L') encoderPivot(true);
        else if (nextMove == 'R') encoderPivot(false);
        else if (nextMove == 'S') {
          setMotorSpeed(L_BASE, R_BASE); delay(150);
        }
      } else {
        setMotorSpeed(0, 0);
        isRunning = false;
        Serial.println("ERROR: End of Memory Reached Early!");
      }
    } else {
      // Forced Corner: Take it automatically, memory doesn't track these
      // ... (Keep your forced corner logic here exactly as it is) ...
      // Forced Corner: Take it automatically, memory doesn't track these
      if (canGoLeft) {
        Serial.println("Dash: Forced Left");
        encoderPivot(true);
      } else if (canGoRight) {
        Serial.println("Dash: Forced Right");
        encoderPivot(false);
      } else if (canGoStraight) {
        Serial.println("Dash: Forced Straight");
        setMotorSpeed(L_BASE, R_BASE); delay(150);
      }
    }
    
    Serial.println(">>> EXITING DASH JUNCTION LOGIC <<<");
    return; // Exit here so it doesn't accidentally run the Explore logic!
  }

  // ==========================================
  // EXPLORE MODE (Mapping)
  // ==========================================
if (useLeftHandRule) {
    if (canGoLeft) {
      Serial.println("Decision: TURN LEFT");
      if (isDecisionPoint) recordTurn('L');
      encoderPivot(true); 
    } else if (canGoStraight) {
      Serial.println("Decision: STRAIGHT");
      if (isDecisionPoint) recordTurn('S'); 
      setMotorSpeed(L_BASE, R_BASE); delay(150);
    } else if (canGoRight) {
      Serial.println("Decision: TURN RIGHT");
      if (isDecisionPoint) recordTurn('R');
      encoderPivot(false); 
    } else {
      // --- LAST OPTION: DEAD END CHECK ---
      bool allWhite = true;
      for (int i = 0; i < 6; i++) {
        if (sensorValues[i] > THRESHOLD_LINE) { allWhite = false; break; }
      }
      if (allWhite) {
        Serial.println("Decision: DEAD END (All White)");
        recordTurn('U');
        executeUTurn();
      }
    }
  } else {
    // Right-Hand Rule
    if (canGoRight) {
      Serial.println("Decision: TURN RIGHT");
      if (isDecisionPoint) recordTurn('R');
      encoderPivot(false); 
    } else if (canGoStraight) {
      Serial.println("Decision: STRAIGHT");
      if (isDecisionPoint) recordTurn('S'); 
      setMotorSpeed(L_BASE, R_BASE); delay(150);
    } else if (canGoLeft) {
      Serial.println("Decision: TURN LEFT");
      if (isDecisionPoint) recordTurn('L');
      encoderPivot(true); 
    } else {
      // --- LAST OPTION: DEAD END CHECK ---
      bool allWhite = true;
      for (int i = 0; i < 6; i++) {
        if (sensorValues[i] > THRESHOLD_LINE) { allWhite = false; break; }
      }
      if (allWhite) {
        Serial.println("Decision: DEAD END (All White)");
        recordTurn('U');
        executeUTurn();
      }
    }
  }

  Serial.println(">>> EXITING JUNCTION LOGIC <<<");
}

// ------------------------------------------------------------
// executeUTurn 
// Phase 1: Push forward to align axle with the tip of the dead end
// Phase 2: Blind encoder spin for ~180°
// Phase 3: Slow sensor-guided seek to re-acquire the line
// ------------------------------------------------------------
void executeUTurn() {
  Serial.println("--- U-TURN STARTED ---");
  portDISABLE_INTERRUPTS();
  leftTicks  = 0;
  rightTicks = 0;
  portENABLE_INTERRUPTS();

  long targetSpinL = (long)(TURN_90_CM * 2.0f * TICKS_PER_CM_L);
  long targetSpinR = (long)(TURN_90_CM * 2.0f * TICKS_PER_CM_R);
  unsigned long spinStart = millis();

  int lastSpinL = 0; // State tracking variables
  int lastSpinR = 0;

  while (true) {
    portDISABLE_INTERRUPTS();
    long curL = leftTicks;
    long curR = rightTicks;
    portENABLE_INTERRUPTS();

    if (curL >= targetSpinL && curR >= targetSpinR) break;
    if (millis() - spinStart > 4000) {
      Serial.println("ERROR: U-TURN SPIN TIMED OUT");
      break;
    }

    int currentLSpeed = (curL < targetSpinL) ?  L_PIVOT : 0;   // Left wheel forward
    int currentRSpeed = (curR < targetSpinR) ? -R_PIVOT : 0;   // Right wheel backward

    // ONLY update the hardware if the speed has actually changed
    if (currentLSpeed != lastSpinL || currentRSpeed != lastSpinR) {
      setMotorSpeed(currentLSpeed, currentRSpeed);
      lastSpinL = currentLSpeed;
      lastSpinR = currentRSpeed;
    }
    
    delay(1);
  }

  setMotorSpeed(0, 0);
  delay(50); 

  // ==========================================================
  // PHASE 3: Slow seek to lock back onto the line
  // ==========================================================
  Serial.println("U-Turn Phase 3: Seeking line...");
  const int SEEK_SPEED = 80;
  unsigned long seekStart = millis();

  // Set the speed ONCE outside the loop. The ESP32 will maintain it automatically!
  setMotorSpeed(SEEK_SPEED, -SEEK_SPEED); 

  while (true) {
    readSensors();

    if (sensorValues[2] > THRESHOLD_LINE || sensorValues[3] > THRESHOLD_LINE) {
      Serial.println("--- U-TURN LINE FOUND ---");
      break;
    }

    if (millis() - seekStart > 2000) {
      Serial.println("WARNING: U-Turn seek timed out");
      break;
    }

    delay(1);
  }

  setMotorSpeed(0, 0);
  delay(30);
  setMotorSpeed(L_BASE, R_BASE); // Re-enter line
  Serial.println("--- U-TURN FINISHED ---");
}

// ------------------------------------------------------------
// recordTurn + simplifyPath (Dual-Bank Memory)
// ------------------------------------------------------------
void recordTurn(char turn) {
  if (useLeftHandRule) {
    if (leftPathLength < MAX_PATH) {
      leftPathMemory[leftPathLength++] = turn;
      simplifyPath(); 
    }
  } else {
    if (rightPathLength < MAX_PATH) {
      rightPathMemory[rightPathLength++] = turn;
      simplifyPath(); 
    }
  }
}

void simplifyPath() {
  // Create reference pointers to the active arrays so the math works for both!
  char* path = useLeftHandRule ? leftPathMemory : rightPathMemory;
  int& len = useLeftHandRule ? leftPathLength : rightPathLength;

  if (len < 3) return;
  if (path[len - 2] != 'U') return; // Middle entry must be a U-turn

  auto toAngle = [](char c) -> int {
    if (c == 'L') return -90;
    if (c == 'R') return  90;
    if (c == 'U') return  180;
    return 0; // 'S'
  };

  char before = path[len - 3];
  char after  = path[len - 1];

  int total = ((toAngle(before) + 180 + toAngle(after)) % 360 + 360) % 360;

  char replacement;
  if      (total ==   0) replacement = 'S';
  else if (total ==  90) replacement = 'R';
  else if (total == 270) replacement = 'L';
  else                   replacement = 'U'; // 180

  len -= 3;
  path[len++] = replacement;

  Serial.print(useLeftHandRule ? "Left Path: [ " : "Right Path: [ ");
  for (int i = 0; i < len; i++) { Serial.print(path[i]); Serial.print(' '); }
  Serial.println("]");
}