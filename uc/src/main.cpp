#include <Arduino.h>
#define USE_USBCON // fix for ROS not communicating on the Arduino Nano 33 IOT
#include <ros.h>
#include "std_msgs/Float32.h"

#include <SPI.h>
#include "globals.h"
#include "Encoder.h"
#include "PID.h"
#include "PwmMotor.h"


// Motor objects
PwmMotor motors[6] = {
    {BR_DRIVE_PWM_PIN, BR_DRIVE_DIR_PIN},   // BRD
    {FR_DRIVE_PWM_PIN, FR_DRIVE_DIR_PIN},   // FRD
    {BL_DRIVE_PWM_PIN, BL_DRIVE_DIR_PIN},   // BLD
    {FL_DRIVE_PWM_PIN, FL_DRIVE_DIR_PIN},   // FLD
    {MR_DRIVE_PWM_PIN, MR_DRIVE_DIR_PIN},   // MRD
    {ML_DRIVE_PWM_PIN, ML_DRIVE_DIR_PIN}    // MLD
};

// Tachometers from motors
Encoder encoders[6] = {
    {BR_DRIVE_ENC_PIN, DRIVE_TACH_RATE}, // BRD
    {FR_DRIVE_ENC_PIN, DRIVE_TACH_RATE}, // FRD
    {BL_DRIVE_ENC_PIN, DRIVE_TACH_RATE}, // BLD
    {FL_DRIVE_ENC_PIN, DRIVE_TACH_RATE}, // FLD
    {MR_DRIVE_ENC_PIN, DRIVE_TACH_RATE}, // MRD
    {ML_DRIVE_ENC_PIN, DRIVE_TACH_RATE}  // MLD
};

// PID controllers for motors
PID drivePids[6] = {
    {DRIVE_PID_PARAMS[BRD][KP], DRIVE_PID_PARAMS[BRD][KI], DRIVE_PID_PARAMS[BRD][KD], DRIVE_PID_PARAMS[BRD][N], SAMPLE_TIME},
    {DRIVE_PID_PARAMS[FRD][KP], DRIVE_PID_PARAMS[FRD][KI], DRIVE_PID_PARAMS[FRD][KD], DRIVE_PID_PARAMS[FRD][N], SAMPLE_TIME},
    {DRIVE_PID_PARAMS[BLD][KP], DRIVE_PID_PARAMS[BLD][KI], DRIVE_PID_PARAMS[BLD][KD], DRIVE_PID_PARAMS[BLD][N], SAMPLE_TIME},
    {DRIVE_PID_PARAMS[FLD][KP], DRIVE_PID_PARAMS[FLD][KI], DRIVE_PID_PARAMS[FLD][KD], DRIVE_PID_PARAMS[FLD][N], SAMPLE_TIME},
    {DRIVE_PID_PARAMS[MRD][KP], DRIVE_PID_PARAMS[MRD][KI], DRIVE_PID_PARAMS[MRD][KD], DRIVE_PID_PARAMS[MRD][N], SAMPLE_TIME},
    {DRIVE_PID_PARAMS[MLD][KP], DRIVE_PID_PARAMS[MLD][KI], DRIVE_PID_PARAMS[MLD][KD], DRIVE_PID_PARAMS[MLD][N], SAMPLE_TIME}};

// this should include ALL motors on the robot
uint8_t numMotors = sizeof(motors) / sizeof(PwmMotor);

uint32_t lastUpdateTime = 0;
int updateTimeout = 1000;

// --------ROS stuff-----------------
ros::NodeHandle node_handle;

void leftCallback(const std_msgs::Float32 &powerMsg)
{
    /*
    mtrFrontLeft.setVelocity(powerMsg.data);
    mtrBackLeft.setVelocity(powerMsg.data);
    */
    analogWrite(A0, 255 * abs(powerMsg.data));
    lastUpdateTime = millis();
}

void rightCallback(const std_msgs::Float32 &powerMsg)
{
    // these powers are negated since the motors are facing opposite the left motors
    /*
    mtrFrontRight.setVelocity(-powerMsg.data);
    mtrBackRight.setVelocity(-powerMsg.data);
    */
    analogWrite(A7, 255 * abs(powerMsg.data));
    lastUpdateTime = millis();
}

ros::Subscriber<std_msgs::Float32> subLeft("left_power", &leftCallback);
ros::Subscriber<std_msgs::Float32> subRight("right_power", &rightCallback);

void setup()
{
    // --- Motor control startup ---
    // Initialize motors, PIDs, and encoders
    for (int i = 0; i < numMotors; i++)
    {
        drivePids[i].setTargetLimits(-20, 20);
        drivePids[i].setLimits(-1.0, 1.0);
        drivePids[i].setTarget(0);

        motors[i].init();
        motors[i].run(0);
        encoders[i].init();
    }

    // Set motors on right side to inverted so they spin in the same direction as the left side
    motors[FRD].setInverted(true);
    motors[MRD].setInverted(true);
    motors[BRD].setInverted(true);

    // get the motors ready to go
    for (int i = 0; i < numMotors; i++)
    {
        motors[i].run(0);
    }

    // --- ROS Startup ---
    node_handle.initNode();
    node_handle.subscribe(subLeft);
    node_handle.subscribe(subRight);
}

void loop()
{
    // cut power to all motors if we haven't received anything in the past 100ms
    if (millis() - lastUpdateTime > updateTimeout)
    {
        for (int i = 0; i < numMotors; i++)
        {
            drivePids[i].setTarget(0);
            motors[i].run(0);
        }
    }
    node_handle.spinOnce();
}
