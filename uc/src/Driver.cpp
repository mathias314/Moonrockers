#include "Driver.h"

/**
 * @brief Constructor for the BEEG class.
 *
 * @param updateInterval - Interval in seconds to run the control loop.
 */
Driver::Driver(float updateInterval)
    : angleSensors{{BR_POT_PIN}, {FR_POT_PIN}, {BL_POT_PIN}, {FL_POT_PIN}},
      driveMotors{
          {BR_DRIVE_PWM_PIN, BR_DRIVE_DIR_PIN},
          {FR_DRIVE_PWM_PIN, FR_DRIVE_DIR_PIN},
          {BL_DRIVE_PWM_PIN, BL_DRIVE_DIR_PIN},
          {FL_DRIVE_PWM_PIN, FL_DRIVE_DIR_PIN},
          {MR_DRIVE_PWM_PIN, MR_DRIVE_DIR_PIN},
          {ML_DRIVE_PWM_PIN, ML_DRIVE_DIR_PIN}},

      pivotMotors{
          {BR_STEER_PWM_PIN, BR_STEER_DIR_PIN},
          {FR_STEER_PWM_PIN, FR_STEER_DIR_PIN},
          {BL_STEER_PWM_PIN, BL_STEER_DIR_PIN},
          {FL_STEER_PWM_PIN, FL_STEER_DIR_PIN}},

      driveEncoders{
          {BR_DRIVE_ENC_PIN, DRIVE_TACH_RATE},
          {FR_DRIVE_ENC_PIN, DRIVE_TACH_RATE},
          {BL_DRIVE_ENC_PIN, DRIVE_TACH_RATE},
          {FL_DRIVE_ENC_PIN, DRIVE_TACH_RATE},
          {MR_DRIVE_ENC_PIN, DRIVE_TACH_RATE},
          {ML_DRIVE_ENC_PIN, DRIVE_TACH_RATE}},

      pivotEncoders{
          {BR_STEER_ENC_PIN, STEER_TACH_RATE},
          {FR_STEER_ENC_PIN, STEER_TACH_RATE},
          {BL_STEER_ENC_PIN, STEER_TACH_RATE},
          {FL_STEER_ENC_PIN, STEER_TACH_RATE}},

      drivePids{
          {DRIVE_PID_PARAMS[BRD][KP], DRIVE_PID_PARAMS[BRD][KI], DRIVE_PID_PARAMS[BRD][KD], DRIVE_PID_PARAMS[BRD][N], updateInterval},
          {DRIVE_PID_PARAMS[FRD][KP], DRIVE_PID_PARAMS[FRD][KI], DRIVE_PID_PARAMS[FRD][KD], DRIVE_PID_PARAMS[FRD][N], updateInterval},
          {DRIVE_PID_PARAMS[BLD][KP], DRIVE_PID_PARAMS[BLD][KI], DRIVE_PID_PARAMS[BLD][KD], DRIVE_PID_PARAMS[BLD][N], updateInterval},
          {DRIVE_PID_PARAMS[FLD][KP], DRIVE_PID_PARAMS[FLD][KI], DRIVE_PID_PARAMS[FLD][KD], DRIVE_PID_PARAMS[FLD][N], updateInterval},
          {DRIVE_PID_PARAMS[MRD][KP], DRIVE_PID_PARAMS[MRD][KI], DRIVE_PID_PARAMS[MRD][KD], DRIVE_PID_PARAMS[MRD][N], updateInterval},
          {DRIVE_PID_PARAMS[MLD][KP], DRIVE_PID_PARAMS[MLD][KI], DRIVE_PID_PARAMS[MLD][KD], DRIVE_PID_PARAMS[MLD][N], updateInterval}},
      
      pivotPositionPids{
          {POS_PID_PARAMS[BR][KP], POS_PID_PARAMS[BR][KI], POS_PID_PARAMS[BR][KD], POS_PID_PARAMS[BR][N], SAMPLE_TIME},
          {POS_PID_PARAMS[FR][KP], POS_PID_PARAMS[FR][KI], POS_PID_PARAMS[FR][KD], POS_PID_PARAMS[FR][N], SAMPLE_TIME},
          {POS_PID_PARAMS[BL][KP], POS_PID_PARAMS[BL][KI], POS_PID_PARAMS[BL][KD], POS_PID_PARAMS[BL][N], SAMPLE_TIME},
          {POS_PID_PARAMS[FL][KP], POS_PID_PARAMS[FL][KI], POS_PID_PARAMS[FL][KD], POS_PID_PARAMS[FL][N], SAMPLE_TIME}},
      
      pivotVelocityPids{
          {STEER_PID_PARAMS[BR][KP], STEER_PID_PARAMS[BR][KI], STEER_PID_PARAMS[BR][KD], STEER_PID_PARAMS[BR][N], SAMPLE_TIME},
          {STEER_PID_PARAMS[FR][KP], STEER_PID_PARAMS[FR][KI], STEER_PID_PARAMS[FR][KD], STEER_PID_PARAMS[FR][N], SAMPLE_TIME},
          {STEER_PID_PARAMS[BL][KP], STEER_PID_PARAMS[BL][KI], STEER_PID_PARAMS[BL][KD], STEER_PID_PARAMS[BL][N], SAMPLE_TIME},
          {STEER_PID_PARAMS[FL][KP], STEER_PID_PARAMS[FL][KI], STEER_PID_PARAMS[FL][KD], STEER_PID_PARAMS[FL][N], SAMPLE_TIME}}
{
    // Initialize the drive motor list so we can loop through it as needed.
    for (unsigned i = 0; i < NUM_DRIVE_MOTORS; i++)
    {
        motors[i] = &driveMotors[i];
        encoders[i] = &driveEncoders[i];
    }
    for (unsigned i = 0; i < NUM_PIVOTS; i++)
    {
        motors[NUM_DRIVE_MOTORS + i] = &pivotMotors[i];
        encoders[NUM_DRIVE_MOTORS + i] = &pivotEncoders[i];
    }

    this->updateInterval = updateInterval;
}

void Driver::init()
{
    // Initialize all motors and encoders
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        motors[i]->init();
        encoders[i]->init();
        motors[i]->run(0);
    }

    // Initialize all potentiometers
    for (unsigned i = 0; i < NUM_PIVOTS; i++)
    {
        angleSensors[i].init();
    }

    // Initialize drive PID controllers
    for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
    {
        drivePids[i].setTarget(0);
        drivePids[i].setTargetLimits(-30, 30);
        drivePids[i].setLimits(-1.0, 1.0);
    }

    // Initialize angle PID
    for (int i = 0; i < NUM_PIVOTS; i++)
    {
        pivotPositionPids[i].setTarget(0);
        pivotPositionPids[i].setTargetLimits(-PI, PI);
        pivotPositionPids[i].setLimits(-70, 70);

        pivotVelocityPids[i].setTarget(0);
        pivotVelocityPids[i].setTargetLimits(-70, 70);
        pivotVelocityPids[i].setLimits(-1.0, 1.0);
    }
}

/**
 * @brief Main update loop for the bot control. Call this as often as possible.
 */
void Driver::update()
{
    static unsigned long debugTime = millis();
    bool debugPrint = false;
    // Send keepalive to motors
    if (millis() - lastKeepAliveTime > KEEP_ALIVE_INTERVAL)
    {
        lastKeepAliveTime = millis();
    }

    // Update pivots
    if (micros() - lastUpdateTime > updateInterval * 1000000)
    {
        float error;
        float targetVel;
        float output;
        for (unsigned i = 0; i < NUM_PIVOTS; i++)
        {
            // Update Potentiometers
            angleSensors[i].update();

            if (!botStopped)
            {
                // Proportional position control
                error = pivotTargets[i] - angleSensors[i].getAngle();
                targetVel = error * PIV_POS_KP;

                // Soft position limits (die if past)
                if ((angleSensors[i].getAngle() < -PIVOT_MAX_ANGLE && targetVel < 0) ||
                    (angleSensors[i].getAngle() > PIVOT_MAX_ANGLE && targetVel > 0))
                {
                    targetVel = 0;
                }

                // PID velocity control
                pivotPids[i].setTarget(targetVel);
                output = pivotPids[i].calculateOutput(angleSensors[i].getVelocity());
                pivotMotors[i].setPower(output);
            }
        }

        lastUpdateTime = micros();
    }
}

void Driver::setMotorPower(Joint motor, float power)
{
    botStopped = false;
    if (power > DRIVE_MAX_PWR)
    {
        power = DRIVE_MAX_PWR;
    }
    else if (power < -DRIVE_MAX_PWR)
    {
        power = -DRIVE_MAX_PWR;
    }
    driveMotors[motor].setPower(power);
}

/**
 * @brief Set the angle (in radians) for the given pivot.
 *
 * @param pivot - Pivot to control.
 * @param angle - Angle to set.
 */
void Driver::setPivotAngle(Joint pivot, float angle)
{
    botStopped = false;
    if (angle > PIVOT_MAX_ANGLE)
    {
        pivotTargets[pivot] = PIVOT_MAX_ANGLE;
    }
    else if (angle < -PIVOT_MAX_ANGLE)
    {
        pivotTargets[pivot] = -PIVOT_MAX_ANGLE;
    }
    else
    {
        pivotTargets[pivot] = angle;
    }
}

/**
 * @brief Set the pivot velocity PID values.
 *
 * @param pivot - Pivot to control.
 * @param kp
 * @param ki
 * @param kd
 * @param N
 */
void Driver::setPivotPIDConstants(Joint pivot, float kp, float ki, float kd, float N)
{
    pivotPids[pivot].setConstants(kp, ki, kd, N, updateInterval);
}

void Driver::drive(float velLeft, float velRight)
{ // Weeeeeeeeeeeeeeee.
    const float outerWheelDist = dist(OUTER_WHEEL_DY, OUTER_WHEEL_DX);
    const float MIN_TURN_RAD = OUTER_WHEEL_DX;
    static unsigned long ledUpdateTime = millis();

    if (abs(velLeft - velRight) < 5e-2)
    { // Drive straight (Have to special case this cause infinite turn radius)
        float powerVal = (velLeft + velRight) / 2.0;
        for (unsigned i = 0; i < NUM_DRIVE_MOTORS; i++)
        {
            if (i % 2)
            {
                driveMotors[i].setPower(powerVal * 0.7);
            }
            else
            {
                driveMotors[i].setPower(-powerVal * 0.7);
            }
        }
        // Set all to straight
        for (unsigned i = 0; i < NUM_PIVOTS; i++)
        {
            setPivotAngle((Joint)i, 0);
        }
    }
    else
    {
        float turnRadius = (velRight + velLeft) / (velLeft - velRight) * INNER_WHEEL_DX;

        if (abs(turnRadius) < INNER_WHEEL_DX / 2.0)
        { // Center turn
            // Set pivot angles
            const float wheelAngle = atan2(OUTER_WHEEL_DY, OUTER_WHEEL_DX);
            setPivotAngle(FRONT_LEFT, -wheelAngle);
            setPivotAngle(FRONT_RIGHT, wheelAngle);
            setPivotAngle(BACK_LEFT, wheelAngle);
            setPivotAngle(BACK_RIGHT, -wheelAngle);

            // Set drive motor speeds
            float innerSpeed = (velLeft - velRight) / 2.0;
            float outerSpeed = innerSpeed * outerWheelDist / INNER_WHEEL_DX;

            if (abs(outerSpeed) > DRIVE_MAX_PWR) // Handle brownout
            {
                outerSpeed = outerSpeed > 0 ? DRIVE_MAX_PWR : -DRIVE_MAX_PWR;
                innerSpeed = outerSpeed * INNER_WHEEL_DX / outerWheelDist;
            }

            driveMotors[FRONT_LEFT].setPower(-outerSpeed);
            driveMotors[FRONT_RIGHT].setPower(-outerSpeed);
            driveMotors[BACK_LEFT].setPower(-outerSpeed);
            driveMotors[BACK_RIGHT].setPower(-outerSpeed);
            driveMotors[CENTER_LEFT].setPower(-innerSpeed);
            driveMotors[CENTER_RIGHT].setPower(-innerSpeed);
        }
        else
        { // Arcing turn
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
            { // Correct to [-90deg, 90deg]
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

            // Set the pivot values
            setPivotAngle(FRONT_LEFT, -leftPivotAngle);
            setPivotAngle(BACK_LEFT, leftPivotAngle);
            setPivotAngle(FRONT_RIGHT, -rightPivotAngle);
            setPivotAngle(BACK_RIGHT, rightPivotAngle);

            // Set the driving values
            driveMotors[FRONT_LEFT].setPower(-leftOuterSpeed);
            driveMotors[BACK_LEFT].setPower(-leftOuterSpeed);
            driveMotors[CENTER_LEFT].setPower(-leftInnerSpeed);
            driveMotors[FRONT_RIGHT].setPower(rightOuterSpeed);
            driveMotors[BACK_RIGHT].setPower(rightOuterSpeed);
            driveMotors[CENTER_RIGHT].setPower(rightInnerSpeed);
        }
    }
}

void Driver::stop()
{
    botStopped = true;
    for (unsigned i = 0; i < NUM_DRIVE_MOTORS; i++)
    {
        driveMotors[i].setPower(0);
    }
}

/**
 * @brief Calculate the norm (2D distance) using the two values.
 *
 * @param x1 - First val.
 * @param x2 - Second val.
 * @return the distnace.
 */
inline float Driver::dist(float x1, float x2) { return sqrt(x1 * x1 + x2 * x2); }

/**
 * Give an appropriate neopixel color for the given velocity.
 *
 * @param vel - velocity.
 * @return Neopixel color value.
 */
uint32_t Driver::driveDirToColor(float vel)
{
    if (vel < -0.1)
    {
        return Adafruit_NeoPixel::Color(255, 0, 0);
    }
    else if (vel > 0.1)
    {
        return Adafruit_NeoPixel::Color(0, 255, 0);
    }

    return Adafruit_NeoPixel::Color(0, 0, 255);
}