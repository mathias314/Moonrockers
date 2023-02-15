#ifndef __PWM_MOTOR_H__
#define __PWM_MOTOR_H__

#include <Arduino.h>

class PwmMotor {
   public:
    PwmMotor(unsigned speedPin, int dirPin=-1);
    void init();
    void setInverted(bool invert = true);

    void run(float power);
    void runTarget();
    float getPower();
    void setTarget(float target);
    float getTarget();
    void stop();

   private:
    bool inverted = false;
    float target = 0.0;
    float power = 0.0;
    unsigned speedPin;      // Pin number for the output 
    int dirPin = -1;        // Pin number for direction control 
};

#endif