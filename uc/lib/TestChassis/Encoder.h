/* 
 * Header for the Encoder class.
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "Arduino.h"
#include <SimpleKalmanFilter.h>  // https://github.com/denyssene/SimpleKalmanFilter

#define STEER_TACH_RATE 504
#define DRIVE_TACH_RATE 324

class Encoder {
public:
    Encoder(int trigPin, float ticksPerRotation);
  
    void init();
    float estimateSpeed();
    float estimateSpeed(bool direction);
    float getSpeed();
    float getFilteredSpeed();
    float getPos();
    void resetPos();
    
private:
    //Static stuff for interrupt handling
    static void (*isrs[])();  // Array of static ISRs for all the instances
    static Encoder **instances;
    static unsigned instance_count;
    void tick();

    // Pins and pin registers
    int trigPin;
    
    volatile int ticks = 0;  // Number of ticks since last speed estimate
    volatile unsigned int totTicks = 0;  // Number of ticks since last speed estimate

    volatile float speed; //current speed of the motor in rpm
    float filteredSpeed; //current filtered speed of the motor in rpm

    volatile unsigned long lastTickTime; //time in microseconds of the last encoder tick
    volatile unsigned long lastEstTime; //time in microseconds of the tick preceding the last estimate

    // Conversion from (ticks / us) -> (rot / min)
    float ticksPerRotation = 360;
    float tickConversion = 1000000ul * 60 / ticksPerRotation;

    bool invertDir = false;

    /*
     SimpleKalmanFilter(e_mea, e_est, q);
     e_mea: Measurement Uncertainty 
     e_est: Estimation Uncertainty 
     q: Process Noise
     */
    SimpleKalmanFilter speedFilter;

};

#endif
