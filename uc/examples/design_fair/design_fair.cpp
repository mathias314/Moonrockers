#include <Arduino.h>
#include <math.h>
#include "PwmMotor.h"
#include "globals.h"
#include "Encoder.h"
#include "PID.h"
#include "Potentiometer.h"

#define USE_USBCON // fix for ROS not communicating on the Arduino Nano 33 IOT
#include <ros.h>
#include "std_msgs/Float32.h"

/*
 * To Run:
 * pio run -t upload -c examples.ini -e design_fair
 */

#define isPos(x) ((x) > 0 ? true : false)
#define NUM_MOTORS 10
#define NUM_DRIVE_MOTORS 6
#define NUM_STEER_MOTORS 4

char readVal = '\0';
float speed = 0;
bool inverted = false;

PwmMotor motors[10] = {
    {BR_DRIVE_PWM_PIN, BR_DRIVE_DIR_PIN},
    {FR_DRIVE_PWM_PIN, FR_DRIVE_DIR_PIN},
    {BL_DRIVE_PWM_PIN, BL_DRIVE_DIR_PIN},
    {FL_DRIVE_PWM_PIN, FL_DRIVE_DIR_PIN},
    {MR_DRIVE_PWM_PIN, MR_DRIVE_DIR_PIN},
    {ML_DRIVE_PWM_PIN, ML_DRIVE_DIR_PIN},
    {BR_STEER_PWM_PIN, BR_STEER_DIR_PIN},
    {FR_STEER_PWM_PIN, FR_STEER_DIR_PIN},
    {BL_STEER_PWM_PIN, BL_STEER_DIR_PIN},
    {FL_STEER_PWM_PIN, FL_STEER_DIR_PIN}};

Encoder encoders[10] = {
    {BR_DRIVE_ENC_PIN, DRIVE_TACH_RATE},
    {FR_DRIVE_ENC_PIN, DRIVE_TACH_RATE},
    {BL_DRIVE_ENC_PIN, DRIVE_TACH_RATE},
    {FL_DRIVE_ENC_PIN, DRIVE_TACH_RATE},
    {MR_DRIVE_ENC_PIN, DRIVE_TACH_RATE},
    {ML_DRIVE_ENC_PIN, DRIVE_TACH_RATE},
    {BR_STEER_ENC_PIN, STEER_TACH_RATE},
    {FR_STEER_ENC_PIN, STEER_TACH_RATE},
    {BL_STEER_ENC_PIN, STEER_TACH_RATE},
    {FL_STEER_ENC_PIN, STEER_TACH_RATE}};

PID drivePids[6] = {
    {DRIVE_PID_PARAMS[BRD][KP], DRIVE_PID_PARAMS[BRD][KI], DRIVE_PID_PARAMS[BRD][KD], DRIVE_PID_PARAMS[BRD][N], SAMPLE_TIME},
    {DRIVE_PID_PARAMS[FRD][KP], DRIVE_PID_PARAMS[FRD][KI], DRIVE_PID_PARAMS[FRD][KD], DRIVE_PID_PARAMS[FRD][N], SAMPLE_TIME},
    {DRIVE_PID_PARAMS[BLD][KP], DRIVE_PID_PARAMS[BLD][KI], DRIVE_PID_PARAMS[BLD][KD], DRIVE_PID_PARAMS[BLD][N], SAMPLE_TIME},
    {DRIVE_PID_PARAMS[FLD][KP], DRIVE_PID_PARAMS[FLD][KI], DRIVE_PID_PARAMS[FLD][KD], DRIVE_PID_PARAMS[FLD][N], SAMPLE_TIME},
    {DRIVE_PID_PARAMS[MRD][KP], DRIVE_PID_PARAMS[MRD][KI], DRIVE_PID_PARAMS[MRD][KD], DRIVE_PID_PARAMS[MRD][N], SAMPLE_TIME},
    {DRIVE_PID_PARAMS[MLD][KP], DRIVE_PID_PARAMS[MLD][KI], DRIVE_PID_PARAMS[MLD][KD], DRIVE_PID_PARAMS[MLD][N], SAMPLE_TIME}};

PID steerPositionPids[4] = {
    {POS_PID_PARAMS[BR][KP], POS_PID_PARAMS[BR][KI], POS_PID_PARAMS[BR][KD], POS_PID_PARAMS[BR][N], SAMPLE_TIME},
    {POS_PID_PARAMS[FR][KP], POS_PID_PARAMS[FR][KI], POS_PID_PARAMS[FR][KD], POS_PID_PARAMS[FR][N], SAMPLE_TIME},
    {POS_PID_PARAMS[BL][KP], POS_PID_PARAMS[BL][KI], POS_PID_PARAMS[BL][KD], POS_PID_PARAMS[BL][N], SAMPLE_TIME},
    {POS_PID_PARAMS[FL][KP], POS_PID_PARAMS[FL][KI], POS_PID_PARAMS[FL][KD], POS_PID_PARAMS[FL][N], SAMPLE_TIME}};

PID steerVelocityPids[4] = {
    {STEER_PID_PARAMS[BR][KP], STEER_PID_PARAMS[BR][KI], STEER_PID_PARAMS[BR][KD], STEER_PID_PARAMS[BR][N], SAMPLE_TIME},
    {STEER_PID_PARAMS[FR][KP], STEER_PID_PARAMS[FR][KI], STEER_PID_PARAMS[FR][KD], STEER_PID_PARAMS[FR][N], SAMPLE_TIME},
    {STEER_PID_PARAMS[BL][KP], STEER_PID_PARAMS[BL][KI], STEER_PID_PARAMS[BL][KD], STEER_PID_PARAMS[BL][N], SAMPLE_TIME},
    {STEER_PID_PARAMS[FL][KP], STEER_PID_PARAMS[FL][KI], STEER_PID_PARAMS[FL][KD], STEER_PID_PARAMS[FL][N], SAMPLE_TIME}};

Potentiometer potentiometers[4] = {
    {BR_POT_PIN}, {FR_POT_PIN}, {BL_POT_PIN}, {FL_POT_PIN}};

float positionPidOutput[4] = {0, 0, 0, 0};
float velocityPidOutput[4] = {0, 0, 0, 0};
float left_stick = 0;
float right_stick = 0;

// --------ROS stuff-----------------
ros::NodeHandle node_handle;

void leftCallback(const std_msgs::Float32 &powerMsg)
{
    // controller outputs opposite signs
    left_stick = -powerMsg.data;
}

void rightCallback(const std_msgs::Float32 &powerMsg)
{
    // controller outputs opposite signs
    right_stick = -powerMsg.data;
}

ros::Subscriber<std_msgs::Float32> subLeft("drive_power", &leftCallback);
ros::Subscriber<std_msgs::Float32> subRight("steer_power", &rightCallback);

void calculate_tank(float velLeft, float velRight)
{
    const float outerWheelDist = dist(OUTER_WHEEL_DY, OUTER_WHEEL_DX);
    const float MIN_TURN_RAD = OUTER_WHEEL_DX;

    if (abs(velLeft - velRight) < 5e-2)
    {
        // Drive straight (Have to special case this cause infinite turn radius)
        float powerVal = (velLeft + velRight) / 2.0 * MAX_DRIVE_VELOCITY;
        for (unsigned i = 0; i < NUM_DRIVE_MOTORS; i++)
        {
            drivePids[i].setTarget(powerVal);
        }

        // Set all to straight
        for (unsigned i = 0; i < NUM_PIVOTS; i++)
        {
            steerPositionPids[i].setTarget(0);
        }
    }
    else
    {
        float turnRadius = (velRight + velLeft) / (velLeft - velRight) * INNER_WHEEL_DX;

        if (abs(turnRadius) < INNER_WHEEL_DX / 2.0)
        {
            // Center turn
            // Set pivot angles
            const float wheelAngle = atan2(OUTER_WHEEL_DY, OUTER_WHEEL_DX);
            steerPositionPids[FL].setTarget(-wheelAngle);
            steerPositionPids[FR].setTarget(wheelAngle);
            steerPositionPids[BL].setTarget(wheelAngle);
            steerPositionPids[BR].setTarget(-wheelAngle);

            // Set drive motor speeds
            float innerSpeed = (velLeft - velRight) / 2.0;
            float outerSpeed = innerSpeed * outerWheelDist / INNER_WHEEL_DX;

            if (abs(outerSpeed) > DRIVE_MAX_PWR) // Handle brownout
            {
                outerSpeed = outerSpeed > 0 ? DRIVE_MAX_PWR : -DRIVE_MAX_PWR;
                outerSpeed *= MAX_DRIVE_VELOCITY;
                innerSpeed = outerSpeed * INNER_WHEEL_DX / outerWheelDist;
            }

            drivePids[FLD].setTarget(-outerSpeed);
            drivePids[FRD].setTarget(-outerSpeed);
            drivePids[BLD].setTarget(-outerSpeed);
            drivePids[BRD].setTarget(-outerSpeed);
            drivePids[MLD].setTarget(-innerSpeed);
            drivePids[MRD].setTarget(-innerSpeed);
        }
        else
        {
            // Arcing turn
            // Make larger than min turn radius
            if (turnRadius > 0 && turnRadius < MIN_TURN_RAD)
            {
                turnRadius = MIN_TURN_RAD;
            }
            else if (turnRadius < 0 && turnRadius > -MIN_TURN_RAD)
            {
                turnRadius = -MIN_TURN_RAD;
            }

            // Calculate the pivot angles
            float leftPivotAngle = atan2(OUTER_WHEEL_DY, (turnRadius + OUTER_WHEEL_DX));
            float rightPivotAngle = atan2(OUTER_WHEEL_DY, (turnRadius - OUTER_WHEEL_DX));

            if (turnRadius < 0.0)
            {
                // Correct to [-90deg, 90deg]
                leftPivotAngle -= PI;
                rightPivotAngle -= PI;
            }

            // Calculate motor speeds
            float outerVel = (turnRadius > 0 ? velLeft : velRight);
            float farthestMotorDist = dist(abs(turnRadius) + OUTER_WHEEL_DX, OUTER_WHEEL_DY);
            float speedPerDist = outerVel / (abs(turnRadius) + INNER_WHEEL_DX);
            float fastestMotorSpeed = farthestMotorDist * speedPerDist;
            if (abs(fastestMotorSpeed) > DRIVE_MAX_PWR) // Hit brownout
            {
                speedPerDist = (fastestMotorSpeed > 0 ? DRIVE_MAX_PWR : -DRIVE_MAX_PWR) / farthestMotorDist;
            }
            float leftOuterSpeed = speedPerDist * dist(turnRadius + OUTER_WHEEL_DX, OUTER_WHEEL_DY);
            float leftInnerSpeed = speedPerDist * abs(turnRadius + INNER_WHEEL_DX);
            float rightOuterSpeed = speedPerDist * dist(turnRadius - OUTER_WHEEL_DX, OUTER_WHEEL_DY);
            float rightInnerSpeed = speedPerDist * abs(turnRadius - INNER_WHEEL_DX);

            // Linear Scale to max velocity
            leftOuterSpeed *= MAX_DRIVE_VELOCITY;
            leftInnerSpeed *= MAX_DRIVE_VELOCITY;
            rightOuterSpeed *= MAX_DRIVE_VELOCITY;
            rightInnerSpeed *= MAX_DRIVE_VELOCITY;

            // Set the pivot values
            steerPositionPids[FL].setTarget(-leftPivotAngle);
            steerPositionPids[BL].setTarget(leftPivotAngle);
            steerPositionPids[FR].setTarget(-rightPivotAngle);
            steerPositionPids[BR].setTarget(rightPivotAngle);

            // Set the driving values
            drivePids[FLD].setTarget(-leftOuterSpeed);
            drivePids[BLD].setTarget(-leftOuterSpeed);
            drivePids[MLD].setTarget(-leftInnerSpeed);
            drivePids[FRD].setTarget(rightOuterSpeed);
            drivePids[BRD].setTarget(rightOuterSpeed);
            drivePids[MRD].setTarget(rightInnerSpeed);
        }
    }
}

/*
    This is a naive implementation of driving.
    
    The leftStick is the power of the left stick ranging from -1.0 to 1.0
    Ditto for the rightStick
*/
void calculate_sticks(float leftStick, float rightStick)
{
    const float outerWheelDist = dist(OUTER_WHEEL_DY, OUTER_WHEEL_DX);
    const float MIN_TURN_RAD = OUTER_WHEEL_DX;

    float driveVelocity = leftStick * MAX_DRIVE_VELOCITY;
    for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
    {
        drivePids[i].setTarget(driveVelocity);
    }

    if (abs(rightStick) < 0.1)
    {
        // Straight
        for (int i = 0; i < NUM_PIVOTS; i++)
        {
            steerPositionPids[i].setTarget(0);
        }
    }
    else
    {
        float steerAngle = rightStick * PIVOT_LIM_OUT;
        steerPositionPids[FL].setTarget(steerAngle);
        steerPositionPids[FR].setTarget(steerAngle);
        steerPositionPids[BL].setTarget(-steerAngle);
        steerPositionPids[BR].setTarget(-steerAngle);
    }

    /*
    if (abs(velLeft - velRight) < 5e-2)
    {
        // Drive straight (Have to special case this cause infinite turn radius)
        float powerVal = (velLeft + velRight) / 2.0 * MAX_DRIVE_VELOCITY;
        for (unsigned i = 0; i < NUM_DRIVE_MOTORS; i++)
        {
            drivePids[i].setTarget(powerVal);
        }

        // Set all to straight
        for (unsigned i = 0; i < NUM_PIVOTS; i++)
        {
            steerPositionPids[i].setTarget(0);
        }
    }
    else
    {
        float turnRadius = (velRight + velLeft) / (velLeft - velRight) * INNER_WHEEL_DX;

        if (abs(turnRadius) < INNER_WHEEL_DX / 2.0)
        {
            // Center turn
            // Set pivot angles
            const float wheelAngle = atan2(OUTER_WHEEL_DY, OUTER_WHEEL_DX);
            steerPositionPids[FL].setTarget(-wheelAngle);
            steerPositionPids[FR].setTarget(wheelAngle);
            steerPositionPids[BL].setTarget(wheelAngle);
            steerPositionPids[BR].setTarget(-wheelAngle);

            // Set drive motor speeds
            float innerSpeed = (velLeft - velRight) / 2.0;
            float outerSpeed = innerSpeed * outerWheelDist / INNER_WHEEL_DX;

            if (abs(outerSpeed) > DRIVE_MAX_PWR) // Handle brownout
            {
                outerSpeed = outerSpeed > 0 ? DRIVE_MAX_PWR : -DRIVE_MAX_PWR;
                outerSpeed *= MAX_DRIVE_VELOCITY;
                innerSpeed = outerSpeed * INNER_WHEEL_DX / outerWheelDist;
            }

            drivePids[FLD].setTarget(-outerSpeed);
            drivePids[FRD].setTarget(-outerSpeed);
            drivePids[BLD].setTarget(-outerSpeed);
            drivePids[BRD].setTarget(-outerSpeed);
            drivePids[MLD].setTarget(-innerSpeed);
            drivePids[MRD].setTarget(-innerSpeed);
        }
        else
        {
            // Arcing turn
            // Make larger than min turn radius
            if (turnRadius > 0 && turnRadius < MIN_TURN_RAD)
            {
                turnRadius = MIN_TURN_RAD;
            }
            else if (turnRadius < 0 && turnRadius > -MIN_TURN_RAD)
            {
                turnRadius = -MIN_TURN_RAD;
            }

            // Calculate the pivot angles
            float leftPivotAngle = atan2(OUTER_WHEEL_DY, (turnRadius + OUTER_WHEEL_DX));
            float rightPivotAngle = atan2(OUTER_WHEEL_DY, (turnRadius - OUTER_WHEEL_DX));

            if (turnRadius < 0.0)
            {
                // Correct to [-90deg, 90deg]
                leftPivotAngle -= PI;
                rightPivotAngle -= PI;
            }

            // Calculate motor speeds
            float outerVel = (turnRadius > 0 ? velLeft : velRight);
            float farthestMotorDist = dist(abs(turnRadius) + OUTER_WHEEL_DX, OUTER_WHEEL_DY);
            float speedPerDist = outerVel / (abs(turnRadius) + INNER_WHEEL_DX);
            float fastestMotorSpeed = farthestMotorDist * speedPerDist;
            if (abs(fastestMotorSpeed) > DRIVE_MAX_PWR) // Hit brownout
            {
                speedPerDist = (fastestMotorSpeed > 0 ? DRIVE_MAX_PWR : -DRIVE_MAX_PWR) / farthestMotorDist;
            }
            float leftOuterSpeed = speedPerDist * dist(turnRadius + OUTER_WHEEL_DX, OUTER_WHEEL_DY);
            float leftInnerSpeed = speedPerDist * abs(turnRadius + INNER_WHEEL_DX);
            float rightOuterSpeed = speedPerDist * dist(turnRadius - OUTER_WHEEL_DX, OUTER_WHEEL_DY);
            float rightInnerSpeed = speedPerDist * abs(turnRadius - INNER_WHEEL_DX);

            // Linear Scale to max velocity
            leftOuterSpeed *= MAX_DRIVE_VELOCITY;
            leftInnerSpeed *= MAX_DRIVE_VELOCITY;
            rightOuterSpeed *= MAX_DRIVE_VELOCITY;
            rightInnerSpeed *= MAX_DRIVE_VELOCITY;

            // Set the pivot values
            steerPositionPids[FL].setTarget(-leftPivotAngle);
            steerPositionPids[BL].setTarget(leftPivotAngle);
            steerPositionPids[FR].setTarget(-rightPivotAngle);
            steerPositionPids[BR].setTarget(rightPivotAngle);

            // Set the driving values
            drivePids[FLD].setTarget(-leftOuterSpeed);
            drivePids[BLD].setTarget(-leftOuterSpeed);
            drivePids[MLD].setTarget(-leftInnerSpeed);
            drivePids[FRD].setTarget(rightOuterSpeed);
            drivePids[BRD].setTarget(rightOuterSpeed);
            drivePids[MRD].setTarget(rightInnerSpeed);
        }
    }
    */
}

//=====setup==============================
void setup()
{
    // --- ROS Startup ---
    node_handle.initNode();
    node_handle.subscribe(subLeft);
    node_handle.subscribe(subRight);

    // Initialize motors and encoders
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        motors[i].init();
        motors[i].run(0);
        encoders[i].init();
    }

    // Initialize potentiometers
    for (int i = 0; i < 4; i++)
    {
        potentiometers[i].init();
    }

    // Initialize drive PID
    for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
    {
        drivePids[i].setTarget(0);
        drivePids[i].setTargetLimits(-MAX_DRIVE_VELOCITY, MAX_DRIVE_VELOCITY);
        drivePids[i].setLimits(-1.0, 1.0);
    }

    // Initialize angle PID
    for (int i = 0; i < NUM_STEER_MOTORS; i++)
    {
        steerPositionPids[i].setTarget(0);
        steerPositionPids[i].setTargetLimits(-PI, PI);
        steerPositionPids[i].setLimits(-MAX_STEER_VELOCITY, MAX_STEER_VELOCITY);

        steerVelocityPids[i].setTarget(0);
        steerVelocityPids[i].setTargetLimits(-MAX_STEER_VELOCITY, MAX_STEER_VELOCITY);
        steerVelocityPids[i].setLimits(-1.0, 1.0);
    }

    // Set motors on right side to inverted so they spin in the same direction as the other wheels
    motors[FRD].setInverted(true);
    motors[MRD].setInverted(true);
    motors[BRD].setInverted(true);

    motors[FLS].setInverted(true);
    motors[BLS].setInverted(true);
    motors[FRS].setInverted(true);
    motors[BRS].setInverted(true);

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        motors[i].run(0);
    }
}

//=====loop==============================
void loop()
{
    // Read serial input
    static bool running = true;

    for (int i = 0; i < NUM_STEER_MOTORS; i++)
    {
        if (potentiometers[i].getAngle() > PI / 2 || potentiometers[i].getAngle() < -PI / 2)
        {
            motors[i + NUM_DRIVE_MOTORS].run(0);
        }
    }

    static unsigned long lastDisplay = millis();
    if (millis() - lastDisplay > 50)
    {
        Serial.print(potentiometers[BR].getAngle());
        Serial.print(" ");
        Serial.print(potentiometers[FR].getAngle());
        Serial.print(" ");
        Serial.print(potentiometers[BL].getAngle());
        Serial.print(" ");
        Serial.print(potentiometers[FL].getAngle());
        Serial.print(" ");
        Serial.print(POTENTIOMETER_OFFSETS[BR]);

        Serial.println();

        lastDisplay = millis();
    }

    static unsigned long lastUpdate = millis();
    if (millis() - lastUpdate > 1000 * SAMPLE_TIME)
    {
        // calculate_tank(left_vel, right_vel);
        calculate_sticks(left_stick, right_stick);

        // Update drive tachometer velocities and use the sign of the motor's power to infer direction
        for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
        {
            encoders[i].estimateSpeed(isPos(drivePids[i].getTarget()));
        }

        // Run drive motors based on PID instructions
        for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
        {
            // Check for runaway motors
            if (encoders[i].getFilteredSpeed() > MAX_DRIVE_VELOCITY + 5 || encoders[i].getFilteredSpeed() < -MAX_DRIVE_VELOCITY - 5)
            {
                motors[i].run(0);
            }
            // Normal operation. Run motors based on PID output
            else
            {
                motors[i].run(drivePids[i].calculateOutput(encoders[i].getFilteredSpeed()));
            }
        }

        // Update potentiometers
        for (int i = 1; i < NUM_STEER_MOTORS; i++)
        {
            potentiometers[i].update();
        }

        // update unique potentiometer with offset
        // this is a special case since the potentiometer model is different from the others
        potentiometers[BR].update(POTENTIOMETER_OFFSETS[BR]);

        // Update steering tachometer velocities and use the sign of the motor's power to infer direction
        for (int i = 0; i < NUM_STEER_MOTORS; i++)
        {
            encoders[i + NUM_DRIVE_MOTORS].estimateSpeed(isPos(steerVelocityPids[i].getTarget()));
        }

        // Calculate velocities for steering motors
        for (int i = 0; i < NUM_STEER_MOTORS; i++)
        {
            // Simple output is a different PID calculation because the numerical approximation goofs for small inputs
            positionPidOutput[i] = steerPositionPids[i].simpleOutput(potentiometers[i].getAngle());
            steerVelocityPids[i].setTarget(positionPidOutput[i]);
            velocityPidOutput[i] = steerVelocityPids[i].calculateOutput(encoders[i + NUM_DRIVE_MOTORS].getFilteredSpeed());
        }

        // Run steering motors based on PID instructions
        for (int i = 0; i < NUM_STEER_MOTORS; i++)
        {
            motors[i + NUM_DRIVE_MOTORS].run(velocityPidOutput[i]);
        }

        lastUpdate = millis();
    }

    // Update ROS
    node_handle.spinOnce();
}
