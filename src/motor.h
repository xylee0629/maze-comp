#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <Arduino.h>

// Motor pins define 
#define AIN1 25
#define AIN2 26
#define PWMA 27
#define BIN1 14
#define BIN2 12
#define PWMB 13 

#define IR0 6 
#define IR1 39
#define IR2 34
#define IR3 35
#define IR4 32
#define IR5 33


class LineFollower
{
    private:
        // Variables 
        long sensorValues[6];
        long baseSpeed;
        
        void moveMotors(int leftSpeed, int rightSpeed);
        void readSensors();
        void PID();

    public:
        LineFollower();

        void begin();

        void setPID(float p, float i, float d);
        void setBaseSpeed(int baseSpeed);

        void update();

};

#endif