/*
 * Class for the potentiometer.
 */
#include "Potentiometer.h"

const float RAD_MS_TO_RPM = 1000 * 60 / (2*PI);

Potentiometer::Potentiometer() : angleFilter(5, 512, 0.25), velocityFilter(5, 0, 0.25) {
}

Potentiometer::Potentiometer(unsigned pin) : Potentiometer() {
    this->pin = pin;

    const unsigned ROT_ANGLE = 260;  // Kind of a guess.
    const unsigned ANALOG_MAX = 1023;
    unsigned angle90Diff = ANALOG_MAX / 2 * (90.0 / (ROT_ANGLE / 2));
    unsigned pts[] = {
        ANALOG_MAX / 2 - angle90Diff,
        ANALOG_MAX / 2,
        ANALOG_MAX / 2 + angle90Diff};

    calibrate(pts);
}

Potentiometer::Potentiometer(unsigned pin, const unsigned* calPts) : Potentiometer() {
    this->pin = pin;
    calibrate(calPts);
}

/**
 * @brief Initialize the class.
 */
void Potentiometer::init() {
    pinMode(pin, INPUT);
#ifdef ESP32
    analogReadResolution(10);
#endif
}

/**
 * @brief Calibrate the potentiometer.
 *
 * @param calPts - Analog values corresponding to {-90, 0, 90}.
 */
void Potentiometer::calibrate(const unsigned* calPts) {
    memcpy(this->calPts, calPts, sizeof(this->calPts));

    // Sort the calibration points into increasing order
    if (this->calPts[0] > this->calPts[2]) {
        // Reverse order
        swapVals(this->calPts, this->calPts+2);
        
        calAngles[0] = PI/2;
        calAngles[2] = -PI/2;
    } else {
        // Regular order
        calAngles[0] = -PI/2;
        calAngles[2] = PI/2;
    }
}

float Potentiometer::interpAngle(unsigned val) {
    unsigned calIdx = 0;

    if (val > calPts[1]) {
        calIdx = 1;
    }

    return ((int)val - (int)calPts[calIdx]) / float((int)calPts[calIdx + 1] - (int)calPts[calIdx]) * (calAngles[calIdx + 1] - calAngles[calIdx]) + calAngles[calIdx];
}

/**
 * Update the current potentiometer value. This should be called at a regular interval.
 */
void Potentiometer::update() {
    // Get current values
    unsigned newEstimateTime = millis();
    float newRawAngle = interpAngle(analogRead(pin));

    // Estimate velocity
    rawVelocity = (newRawAngle - rawAngle) / (newEstimateTime - estimateTime) * RAD_MS_TO_RPM; 

    // Update the values for next iteration
    rawAngle = newRawAngle;
    estimateTime = newEstimateTime;
    
    // Filter
    filteredVelocity = velocityFilter.updateEstimate(rawVelocity);
    filteredAngle = angleFilter.updateEstimate(rawAngle);
}

/**
 * @brief Get the filtered angle value.
 *
 * @return The angle in radians. 0 is straight.
 */
float Potentiometer::getAngle() {
    return filteredAngle;
}
/**
 * @brief Get the unfiltered angle value.
 *
 * @return The angle in radians. 0 is straight.
 */
float Potentiometer::getRawAngle() {
    return rawAngle;
}

/**
 * @brief Get the filtered velocity estimate.
 *
 * @return The angle in RPMs.
 */
float Potentiometer::getVelocity() {
    return filteredVelocity;
}

/**
 * @brief Get the filtered velocity estimate.
 *
 * @return The angle in RPMs.
 */
float Potentiometer::getRawVelocity() {
    return rawVelocity;
}

template <class valType>
void Potentiometer::swapVals(valType* a, valType* b) {
    valType tmp = *a;
    *a = *b;
    *b = tmp;
}