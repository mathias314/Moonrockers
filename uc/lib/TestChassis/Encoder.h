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

    bool init();
    float estimateSpeed();
    float estimateSpeed(bool direction);
    float getSpeed();
    float getFilteredSpeed();
    void invertDirection(bool invertDir);

    long absTot = 0;

private:
    void tick(bool trigA);

    // Static stuff for interrupt handling
    static void isr0A();
    static void isr1A();
    static void isr2A();
    static void isr3A();
    static void isr4A();
    static void isr5A();
    static void isr6A();
    static void isr7A();
    static void isr8A();
    static void isr9A();

    static void isr0B();
    static void isr1B();
    static void isr2B();
    static void isr3B();
    static void isr4B();
    static void isr5B();
    static void isr6B();
    static void isr7B();
    static void isr8B();
    static void isr9B();

    static int numInstances;
    
    static Encoder *instance0;
    static Encoder *instance1;
    static Encoder *instance2;
    static Encoder *instance3;
    static Encoder *instance4;
    static Encoder *instance5;
    static Encoder *instance6;
    static Encoder *instance7;
    static Encoder *instance8;
    static Encoder *instance9;

    // private methods for binding interrupts
    void bindInterruptA(int aPin);
    void bindInterruptB(int bPin);
    bool bindInstance();

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
