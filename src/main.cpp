/* Main code for maze competiton using a modified floodfill algorithm. 
  To Do: 
  -> Confirm pin connection
  -> Add Line follower code
  -> Add PID: https://forum.arduino.cc/t/hall-effects-sensor-with-dc-motor-used-as-encoder/532648/16
  -> Add encoder calculation
  -> Add cell track (at which tick does the robot travel to the next grid)
  -> Add floodfill algorithm
  -> Mode switch with buttons code 
  -> Seperate movement, maze solve into two source code files: https://www.geeksforgeeks.org/cpp/build-a-cpp-program-that-have-multiple-source-code-files/
  -> walk into the sea once all this is over

  Done:
  -> Motor control
  -> Encoder ticks tracking
  -> Add git source control

  Progress: 
  -> Code compiles... i guess
*/

#include <Arduino.h>

// TB6612FNG Motor Driver Pins (Placeholder)
#define AIN1 32
#define AIN2 33
#define PWMA 25
#define BIN1 19
#define BIN2 18
#define PWMB 5 
#define STBY 17 // Pulled High to disable standby mode

// Encoder Pins (Placeholder)
#define ENC1 12
#define ENC1_DIRECT 11
#define ENC2 13
#define ENC2_DIRECT 14

// IR Sensor Pins (Placeholder)
int IR1 = 1; // Left
int IR2 = 2; // Left
int IR3 = 3; // Centre
int IR4 = 4; // Centre
int IR5 = 5; // Right
int IR6 = 6; // Right

// Variables for motor
const int frequency = 30000; // max frequency is 100kHz
int baseSpeed = 200;

// Variables for encoder
volatile long ENC1_TICKS = 0;
volatile long ENC2_TICKS = 0;
const int ENC_SLOTS = 20; // Placeholder

// Variable for PID
long threshold = 2000; // Placeholder 
int *sensor[6] = {&IR1, &IR2, &IR3, &IR4, &IR5, &IR6};
long sensorValue[6];
float Kp = 0;
float Ki = 0;
float Kd = 0;
int error, position, speed, lastError;
int target = 2500;

// IR Sensor read 
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

  int leftSpeed = baseSpeed - speed;
  int rightSpeed = baseSpeed + speed;
  return leftSpeed, rightSpeed;
}

// Motor Function
void motorSpeed(int a, int b){
  analogWriteFrequency(frequency);
  analogWrite(PWMA, a); // PWM range from 0(OFF) to 255(MAX)
  analogWrite(PWMB, b);
}

void move(int a, int b){
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  motorSpeed(a, b);
}

void readEncoder1(){
  if (digitalRead(ENC1) ==  digitalRead(ENC1_DIRECT)){
    ENC1_TICKS++;
  }
  else{
    ENC1_TICKS--;
  }
}

void readEncoder2(){
  if (digitalRead(ENC2) ==  digitalRead(ENC2_DIRECT)){
    ENC2_TICKS++;
  }
  else{
    ENC2_TICKS--;
  }
}

void setup() {

  Serial.begin(115200);

  // Motor driver pinMode
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, INPUT_PULLUP); // Pulled HIGH by default

  // Encoder pinMode
  pinMode(ENC1, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1), readEncoder1, RISING); // Check for interrupt pins on ESP32, check if encoder is LOW or HIGH when blocked.
  pinMode(ENC2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC2), readEncoder2, RISING);

  // IR sensor pinMode
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  pinMode(IR6, INPUT);

}

void loop() {

  // Add calibration for black sensor 
  for (int i = 0; i < 6; i++){
    sensorValue[i] = analogRead(*sensor[i]);
  }

  // detect what situation the car is facing 
  // forward case: call PID function
  if ((sensorValue[0] && sensorValue[5]) < threshold && (sensorValue[2] && sensorValue[3]) > threshold){
    int leftSpeed, rightSpeed = PID();
    move(leftSpeed, rightSpeed);
  }

  // leftmost sensor detects black, turn spot right
  else if (sensorValue[0] > threshold && (sensorValue[2] && sensorValue[3]) > threshold && sensorValue[5] < threshold){
    move(-baseSpeed, baseSpeed);
  }
  // rightmost sensor detects black, turn spot left
  else if (sensorValue[0] < threshold && (sensorValue[2] && sensorValue[3]) > threshold && sensorValue[5] > threshold){
    move(baseSpeed, -baseSpeed);
  }
  // all sensors detect black, stop
  else if ((sensorValue[0] && sensorValue[1] && sensorValue[2] && sensorValue[3] && sensorValue[4] + sensorValue[5]) > threshold){
    move(0, 0);
  }

  // Forward: a = b, Left: a < b, Right: a > b, Backwards: -a = -b

  Serial.println(ENC1_TICKS);
  Serial.println(ENC1_TICKS);
  Serial.println(" ");

}