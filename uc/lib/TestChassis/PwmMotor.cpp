/**
 * @file PwmMotor.cpp
 * @brief Class to interface with a PWM motor controller.
 */
#include "PwmMotor.h"

/**
 * Constructor for the class.
 * 
 * @param speedPin - Pin to use for the output. Must be PWM.
 * @param dirPin - Pin for controlling direction.
 */
PwmMotor::PwmMotor(unsigned speedPin, int dirPin) : speedPin(speedPin), dirPin(dirPin) {
}

/**
 * Initialize the PWM output to the controller.
 */
void PwmMotor::init() {
    pinMode(speedPin, OUTPUT);

    if (dirPin != -1)
    {
        pinMode(dirPin, OUTPUT);
    }
}

/**
 * Set whether the motor should run in inverted mode.
 * 
 * @param invert - (optional) Whether the motor direction should be inverted. Default to true. 
 */
void PwmMotor::setInverted(bool invert) {
    this->inverted = invert;
    run(power);
}

/**
 * Set the output value.
 * 
 * @param power - percent output. Float value in [-1.0, 1.0].
 */
void PwmMotor::run(float power) {
    this->power = constrain(power, -1.0, 1.0);
    if (inverted) {
        power = -power;
    }
    
    if (dirPin == -1) {
        analogWrite(speedPin, 128+127*power);
    } else {
        analogWrite(speedPin, 255 * abs(power));
        digitalWrite(dirPin, (power < 0));
    }
}

void PwmMotor::runTarget() {
    this->run(target);
}

/**
 * Get the currently set power value. 
 * 
 * @return the current set power. 
 */
float PwmMotor::getPower() {
    return power;
}

void PwmMotor::setTarget(float target) {
    this->target = target;
}

float PwmMotor::getTarget() {
    return target;
}

/**
 * Stop the motor.
 */
void PwmMotor::stop() {
    this->run(0);
}