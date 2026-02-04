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
  -> Add git source control (Research)
  -> walk into the sea once all this is over

  Done:
  -> Motor control
  -> Encoder ticks tracking

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

// Variables for motor
const int frequency = 30000; // max frequency is 100kHz

// Variables for encoder
volatile long ENC1_TICKS = 0;
volatile long ENC2_TICKS = 0;


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

}

void loop() {

  // Forward: a = b, Left: a < b, Right: a > b, Backwards: -a = -b
  move(200, 200);
  delay(1000);
  Serial.println(ENC1_TICKS);
  Serial.println(ENC1_TICKS);
  Serial.println(" ");

}