#ifndef __PWM_MTOR_H__
#define __PWM_MTOR_H__

#include <Arduino.h>

class PwmMotor {
   public:
    PwmMotor(unsigned pin, unsigned channel);
    void init();
    void setInverted(bool invert = true);

    void setPower(float newPower);
    float getPower();

   private:
    bool inverted = false;
    float power = 0.0;
    unsigned pin;      // Pin number for the output 
    unsigned channel;  // PWM channel for the output.


    static unsigned channelCount;  // Count of the number of channels used.
};

#endif