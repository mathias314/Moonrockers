/**
 * Header for the potentiometer class.
 */

#ifndef _POTENTIOMETER_H_
#define _POTENTIOMETER_H_

#include "Arduino.h"
#include <SimpleKalmanFilter.h>  // https://github.com/denyssene/SimpleKalmanFilter

class Potentiometer {
public:
    Potentiometer(unsigned pin);
    Potentiometer(unsigned pin, const unsigned* calPts);
  
    void init();
    void calibrate(const unsigned* calPts);
    void update();
    float getAngle();
    float getRawAngle();
    float getVelocity();
    float getRawVelocity();
    
private:
    unsigned pin = 0;
    unsigned calPts[3] = {0};  // Calibration points corresponding to {-90, 0, 90}.
    float calAngles[3] = {-PI/2, 0, PI/2};  // Corresponding calibration angles

    float rawAngle;
    float rawVelocity;
    float filteredAngle;
    float filteredVelocity;
    unsigned long estimateTime;

    /*
     SimpleKalmanFilter(e_mea, e_est, q);
     e_mea: Measurement Uncertainty 
     e_est: Estimation Uncertainty 
     q: Process Noise
     */
    SimpleKalmanFilter angleFilter;
    SimpleKalmanFilter velocityFilter;

    Potentiometer();
    float interpAngle(unsigned val);

    template <class valType>
    void swapVals(valType* a, valType* b);
};

#endif
