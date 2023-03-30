#ifndef DRIVER_H
#define DRIVER_H

#include <Arduino.h>
#include <math.h>
#include "PwmMotor.h"
#include "globals.h"
#include "Encoder.h"
#include "PID.h"
#include "Potentiometer.h"

class Driver
{
public:
    enum MotorLocation
    {
        BACK_RIGHT_DRIVE,
        FRONT_RIGHT_DRIVE,
        BACK_LEFT_DRIVE,
        FRONT_LEFT_DRIVE,
        MID_RIGHT_DRIVE,
        MID_LEFT_DRIVE,
        BACK_RIGHT_STEER,
        FRONT_RIGHT_STEER,
        BACK_LEFT_STEER,
        FRONT_LEFT_STEER
    };

    enum DriveLocation
    {
        BACK_RIGHT,
        FRONT_RIGHT,
        BACK_LEFT,
        FRONT_LEFT,
        MID_RIGHT,
        MID_LEFT
    };

    enum PivotLocation
    {
        BACK_RIGHT,
        FRONT_RIGHT,
        BACK_LEFT,
        FRONT_LEFT
    };

    Driver(float updateInterval);

    void init();
    // void addCanSender(void (*canSender)(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *dat));
    // void addCanReceiver(unsigned pinNum, unsigned (*canReceiver)(uint32_t *id, uint8_t *dat));

    void update();
    void stop();

    void drive(float velLeft, float velRight);
    void setMotorPower(DriveLocation motor, float power);
    void setPivotAngle(PivotLocation pivot, float angle);

    void setPivotPIDConstants(PivotLocation pivot, float kp, float ki, float kd, float N);

    // Motor controllers
    static const unsigned NUM_DRIVE_MOTORS = 6;
    static const unsigned NUM_PIVOTS = 4;

    Potentiometer angleSensors[NUM_PIVOTS];

private:
    bool botStopped = true; // Whether we are actively running motors

    // Motor controllers
    PwmMotor driveMotors[NUM_DRIVE_MOTORS];
    PwmMotor pivotMotors[NUM_PIVOTS];
    // SparkMax driveMotors[NUM_DRIVE_MOTORS];
    // TalonSrx pivotMotors[NUM_PIVOTS];

    static const unsigned NUM_MOTORS = NUM_DRIVE_MOTORS + NUM_PIVOTS;
    //FrcMotorController *motors[NUM_MOTORS];

    // Position feedback sensors and PID
    PID pivotPids[NUM_PIVOTS];
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