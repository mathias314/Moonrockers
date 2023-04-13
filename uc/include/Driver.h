/**
 * Header for the PID class.
 */
#ifndef _DRIVER_H_
#define _DRIVER_H_

#include <stdint.h>

#include "Encoder.h"
#include "PID.h"
#include "Potentiometer.h"
#include "PwmMotor.h"
#include "globals.h"

class Driver
{
public:
    enum Joint
    {
        BACK_RIGHT,
        FRONT_RIGHT,
        BACK_LEFT,
        FRONT_LEFT,
        MIDDLE_RIGHT,
        MIDDLE_LEFT
    };

    Driver(float updateInterval);

    void init();
    void update();
    void stop();

    void drive(float velLeft, float velRight);
    void setMotorPower(Joint motor, float power);
    void setPivotAngle(Joint pivot, float angle);

    void setPivotPIDConstants(Joint pivot, float kp, float ki, float kd, float N);

    // Motor controllers
    static const unsigned NUM_DRIVE_MOTORS = 6;
    static const unsigned NUM_PIVOTS = 4;

    Potentiometer angleSensors[NUM_PIVOTS];

private:
    bool botStopped = true; // Whether we are actively running motors

    // Motor controllers
    PwmMotor driveMotors[NUM_DRIVE_MOTORS];
    PwmMotor pivotMotors[NUM_PIVOTS];
    static const unsigned NUM_MOTORS = NUM_DRIVE_MOTORS + NUM_PIVOTS;
    PwmMotor *motors[NUM_MOTORS];

    // Velocity Encoders
    Encoder driveEncoders[NUM_DRIVE_MOTORS];
    Encoder pivotEncoders[NUM_PIVOTS];
    Encoder *encoders[NUM_MOTORS];

    // Position feedback sensors and PID
    PID drivePids[NUM_DRIVE_MOTORS];
    PID pivotPositionPids[NUM_PIVOTS];
    PID pivotVelocityPids[NUM_PIVOTS];
    float pivotTargets[NUM_PIVOTS] = {0};

    // Some timing stuff
    const unsigned KEEP_ALIVE_INTERVAL = 40;
    float updateInterval = 0;

    unsigned long lastKeepAliveTime = 0;
    unsigned long lastUpdateTime = 0;

    // Some drivetrain dimensions
    const float OUTER_WHEEL_DY = 15.25;
    const float OUTER_WHEEL_DX = 9;
    const float INNER_WHEEL_DX = 9;

    // Pivoting angle limits
    const float PIVOT_LIM_OUT = PI / 2;
    const float PIVOT_LIM_IN = PI / 2;

    inline float dist(float x1, float x2);
};

#endif
