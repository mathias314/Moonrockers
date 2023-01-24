/*
 * Header for the Encoder class.
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "Arduino.h"
#include <SimpleKalmanFilter.h> // https://github.com/denyssene/SimpleKalmanFilter

class Encoder
{
public:
    Encoder(int aPin, int bPin, int ticksPerRotation);
    Encoder(int aPin, int ticksPerRotation);

    void init();
    float estimateSpeed();
    float estimateSpeed(bool direction);
    float getSpeed();
    float getFilteredSpeed();
    void invertDirection(bool invertDir);

    long absTot = 0;

private:
    void tick(bool trigA);

    // Static stuff for interrupt handling
    static void isrA();
    static void isrB();
    static Encoder *instance;

    // Pins and pin registers
    int aPin, bPin = -1;
    volatile uint8_t *aPinRegister;
    volatile uint8_t *bPinRegister;
    uint8_t aPinBit, bPinBit;

    volatile int ticks = 0; // Number of ticks since last speed estimate

    volatile float speed; // current speed of the motor in rpm
    float filteredSpeed;  // current filtered speed of the motor in rpm

    bool invertDir = false;             // Track whether speed estimation should be negated
    volatile float lastFilteredSpeed = 0.0; // last calculated speed for checking speed deltas

    volatile unsigned long lastTickTime; // time in microseconds of the last encoder tick
    volatile unsigned long lastEstTime;  // time in microseconds of the tick preceding the last estimate

    // Conversion from (ticks / us) -> (rot / min)
    int ticksPerRotation = 360;
    unsigned long tickConversion = 1000000ul * 60 / ticksPerRotation;

    /*
     SimpleKalmanFilter(e_mea, e_est, q);
     e_mea: Measurement Uncertainty
     e_est: Estimation Uncertainty
     q: Process Noise
     */
    SimpleKalmanFilter speedFilter;
};

#endif
