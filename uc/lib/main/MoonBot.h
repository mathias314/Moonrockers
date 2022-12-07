/**
 * Header for the PID class.
 */
#ifndef _MOON_BOT_H_
#define _MOON_BOT_H_

#include <Adafruit_NeoPixel.h>
#include <stdint.h>

#include "PID.h"
#include "Potentiometer.h"
#include "TalonSrx.h"
#include "SparkMax.h"
#include "globals.h"

class MoonBot {
   public:
    enum Joint { FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT, CENTER_LEFT, CENTER_RIGHT };

    MoonBot(float updateInterval);

    void init();
    void addCanSender(void (*canSender)(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *dat));
    void addCanReceiver(unsigned pinNum, unsigned (*canReceiver)(uint32_t *id, uint8_t *dat));

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
    SparkMax driveMotors[NUM_DRIVE_MOTORS];
    TalonSrx pivotMotors[NUM_PIVOTS];

    static const unsigned NUM_MOTORS = NUM_DRIVE_MOTORS + NUM_PIVOTS;
    FrcMotorController* motors[NUM_MOTORS];

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

    Adafruit_NeoPixel leftLeds;
    Adafruit_NeoPixel rightLeds;

    inline float dist(float x1, float x2);
    uint32_t driveDirToColor(float vel);
};

#endif
