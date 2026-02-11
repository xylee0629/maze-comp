/* Main code for maze competiton using a modified floodfill algorithm. 
  To Do: 
  -> Confirm pin connection
  -> Turn memory
  -> all sensors white scenario
  -> Consider turning IR1 to IR0 for consistency with IR sensor value array 
  -> Bluetooth to check remotely without wire
  -> Add cell track (at which tick does the robot travel to the next grid)
  -> Add floodfill algorithm
  -> Mode switch with buttons code 
  -> Seperate movement, maze solve into two source code files: https://www.geeksforgeeks.org/cpp/build-a-cpp-program-that-have-multiple-source-code-files/
  -> walk into the sea once all this is over

  Done:
  -> Motor control
  -> Encoder ticks tracking
  -> Add git source control
  -> Add Line follower code
  -> Add PID
  -> Add encoder calculation

  Progress: 
  -> Code compiles... i guess
*/

#include <Arduino.h>

// TB6612FNG Motor Driver Pins (Placeholder)
#define AIN1 25
#define AIN2 26
#define PWMA 27
#define BIN1 14
#define BIN2 12
#define PWMB 13 

// Encoder Pins (Placeholder)
#define ENC1 18
#define ENC1_DIRECT 19
#define ENC2 16
#define ENC2_DIRECT 17

// IR Sensor Pins (Placeholder)
int IR1 = 36; // Left
int IR2 = 39; // Left
int IR3 = 34; // Centre
int IR4 = 35; // Centre
int IR5 = 32; // Right
int IR6 = 33; // Right

// LED Pins (6 pins)
#define RED 
#define YLW
#define GRN


// Variables for motor
const int frequency = 30000; // max frequency is 100kHz
int baseSpeed = 200;

// Variables for encoder
volatile long ENC1_TICKS = 0;
volatile long ENC2_TICKS = 0;
const int ENC_SLOTS = 20; // Placeholder

// Variable for PID
long threshold = 1000; // Placeholder 
int *sensor[6] = {&IR1, &IR2, &IR3, &IR4, &IR5, &IR6};
long sensorValue[6];
float Kp = 0;
float Ki = 0;
float Kd = 0;
int error, position, speed, lastError;
int target = 2500;

// Map Variables
int wall[8][8] = {0};
int xAxis, yAxis = 0;

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

// Direction function
void direction()
{
  // detect what situation the car is facing 
  // Forward: a = b, Left: a < b, Right: a > b, Backwards: -a = -b
  // forward case: call PID function
  if ((sensorValue[0] && sensorValue[5]) < threshold && (sensorValue[2] && sensorValue[3]) > threshold){
    int leftSpeed, rightSpeed = PID();
    move(leftSpeed, rightSpeed);
  }

  // leftmost sensor detects black, turn spot right
  else if (sensorValue[0] > threshold && (sensorValue[2] && sensorValue[3]) > threshold && sensorValue[5] < threshold){
    move(-baseSpeed, baseSpeed);
    yAxis++;
    ENC1_TICKS, ENC2_TICKS = 0;
  }
  // rightmost sensor detects black, turn spot left
  else if (sensorValue[0] < threshold && (sensorValue[2] && sensorValue[3]) > threshold && sensorValue[5] > threshold){
    move(baseSpeed, -baseSpeed);
    yAxis--;
    ENC1_TICKS, ENC2_TICKS = 0;
  }
  // all sensors detect black, stop
  else if ((sensorValue[0] && sensorValue[1] && sensorValue[2] && sensorValue[3] && sensorValue[4] + sensorValue[5]) > threshold){
    move(0, 0);
  }
}
// Consider what happens when the sensor array detects white 

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

  // Constantly reading values from sensor (Add calibration phase)
  for (int i = 0; i < 6; i++){
    sensorValue[i] = analogRead(*sensor[i]);
  }

  direction();
  // Calculate the encoder distance and track grid 
  // wheel diameter = 34mm = 3.4cm
  float AVG_TICKS = (ENC1_TICKS + ENC2_TICKS) / 2.0;
  float CM_PER_TICK = PI * 3.4 / ENC_SLOTS;
  float DISTANCE = AVG_TICKS * CM_PER_TICK;

  // grid check
  if (DISTANCE >= 25.0){
    xAxis++;
    ENC1_TICKS, ENC2_TICKS = 0;
  }


  Serial.println(ENC1_TICKS);
  Serial.println(ENC1_TICKS);
  Serial.println(" ");

}