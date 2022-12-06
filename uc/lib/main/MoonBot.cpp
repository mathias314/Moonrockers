#include "MoonBot.h"

/**
 * @brief Constructor for the BEEG class.
 *
 * @param updateInterval - Interval in seconds to run the control loop.
 */
MoonBot::MoonBot(float updateInterval)
    : driveMotors{FL_MTR_ID, FR_MTR_ID, BL_MTR_ID, BR_MTR_ID, CL_MTR_ID, CR_MTR_ID},
      pivotMotors{FL_PIVOT_MTR_ID, FR_PIVOT_MTR_ID, BL_PIVOT_MTR_ID, BR_PIVOT_MTR_ID},
      angleSensors{{FL_PIVOT_SENSOR_PIN, ANGLE_SENSOR_CALIBRATIONS[0]},
                   {FR_PIVOT_SENSOR_PIN, ANGLE_SENSOR_CALIBRATIONS[1]},
                   {BL_PIVOT_SENSOR_PIN, ANGLE_SENSOR_CALIBRATIONS[2]},
                   {BR_PIVOT_SENSOR_PIN, ANGLE_SENSOR_CALIBRATIONS[3]}},
      pivotPids{{PIV_KP, PIV_KI, PIV_KD, PIV_N, updateInterval},
                {PIV_KP, PIV_KI, PIV_KD, PIV_N, updateInterval},
                {PIV_KP, PIV_KI, PIV_KD, PIV_N, updateInterval},
                {PIV_KP, PIV_KI, PIV_KD, PIV_N, updateInterval}}
    //   leftLeds(NUM_LEFT_LEDS, LEFT_LED_PIN, NEO_GRB + NEO_KHZ800),
    //   rightLeds(NUM_RIGHT_LEDS, RIGHT_LED_PIN, NEO_GRB + NEO_KHZ800) 
{
    // Initialize the drive motor list so we can loop through it as needed.
    for (unsigned i = 0; i < NUM_DRIVE_MOTORS; i++) {
        motors[i] = &driveMotors[i];
    }
    for (unsigned i = 0; i < NUM_PIVOTS; i++) {
        motors[NUM_DRIVE_MOTORS+i] = &pivotMotors[i];
    }

    this->updateInterval = updateInterval;
}

void MoonBot::init() {
    // Initialize all motors
    for (unsigned i = 0; i < NUM_MOTORS; i++) {
        motors[i]->clearFaults();
    }

    // Initialize all pivots
    for (unsigned i = 0; i < NUM_PIVOTS; i++) {
        angleSensors[i].init();
        pivotPids[i].setLimits(-0.4, 0.4);
        pivotPids[i].setTargetLimits(-PI / 2, PI / 2);
        pivotPids[i].setTargetRampRate(5);
        pivotPids[i].setTarget(0, false);
    }
}

void MoonBot::addCanSender(void (*canSender)(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len,
                                             const uint8_t *dat)) {
    FrcMotorController::addCanSender(canSender);
}

void MoonBot::addCanReceiver(unsigned pinNum, unsigned (*canReceiver)(uint32_t *id, uint8_t *dat)) {
    FrcMotorController::addCanReceiver(pinNum, canReceiver);
}

/**
 * @brief Main update loop for the bot control. Call this as often as possible.
 */
void MoonBot::update() {
    // Send keepalive to motors
    if (millis() - lastKeepAliveTime > KEEP_ALIVE_INTERVAL) {
        FrcMotorController::sendKeepAlive();
        lastKeepAliveTime = millis();
    }

    // Update pivots
    if (millis() - lastUpdateTime > updateInterval) {
        for (unsigned i = 0; i < NUM_PIVOTS; i++) {
            angleSensors[i].update();

            float powerVal = pivotPids[i].calculateOutput(angleSensors[i].getFilteredAngle());
            pivotMotors[i].setPower(powerVal);
        }
        lastUpdateTime = millis();
    }
}

void MoonBot::setMotorPower(Joint motor, float power) { driveMotors[motor].setPower(power); }

/**
 * @brief Set the angle (in radians) for the given pivot.
 *
 * @param pivot - Pivot to control.
 * @param angle - Angle to set.
 */
void MoonBot::setPivotAngle(Joint pivot, float angle) { pivotPids[pivot].setTarget(angle); }

void MoonBot::drive(float velLeft, float velRight) {  // Weeeeeeeeeeeeeeee.
    const float outerWheelDist = dist(OUTER_WHEEL_DY, OUTER_WHEEL_DX);
    const float MIN_TURN_RAD = OUTER_WHEEL_DX;

    if (abs(velLeft - velRight) < 5e-2) {  // Drive straight (Have to special case this cause infinite turn radius)
        float powerVal = (velLeft + velRight) / 2.0;
        for (unsigned i = 0; i < NUM_DRIVE_MOTORS; i++) {
            driveMotors[i].setPower(powerVal);
        }
        // Set all to straight
        for (unsigned i = 0; i < NUM_PIVOTS; i++) {
            pivotPids[i].setTarget(0);
        }
    } else {
        float turnRadius = (velRight + velLeft) / (velLeft - velRight) * INNER_WHEEL_DX;

        if (abs(turnRadius) < INNER_WHEEL_DX / 3.0) {  // Center turn
            // Set pivot angles
            const float wheelAngle = atan2(OUTER_WHEEL_DY, OUTER_WHEEL_DX);
            pivotPids[FRONT_LEFT].setTarget(-wheelAngle);
            pivotPids[FRONT_RIGHT].setTarget(wheelAngle);
            pivotPids[BACK_LEFT].setTarget(wheelAngle);
            pivotPids[BACK_RIGHT].setTarget(-wheelAngle);

            // Set drive motor speeds
            float innerSpeed = (velLeft - velRight) / 2.0;
            float outerSpeed = innerSpeed * outerWheelDist / INNER_WHEEL_DX;

            if (abs(outerSpeed) > 1.0)  // Handle brownout
            {
                outerSpeed = 1.0;
                innerSpeed = outerSpeed * INNER_WHEEL_DX / outerWheelDist;
            }

            driveMotors[FRONT_LEFT].setPower(outerSpeed);
            driveMotors[FRONT_RIGHT].setPower(outerSpeed);
            driveMotors[BACK_LEFT].setPower(outerSpeed);
            driveMotors[BACK_RIGHT].setPower(outerSpeed);
            driveMotors[CENTER_LEFT].setPower(innerSpeed);
            driveMotors[CENTER_RIGHT].setPower(innerSpeed);
            Serial.printf("CENTER|out: %.2f, in: %.2f\n", outerSpeed, innerSpeed);
        } else {  // Arcing turn
            // Make larger than min turn radius
            if (turnRadius > 0 && turnRadius < MIN_TURN_RAD) {
                turnRadius = MIN_TURN_RAD;
            } else if (turnRadius < 0 && turnRadius > -MIN_TURN_RAD) {
                turnRadius = -MIN_TURN_RAD;
            }

            // Calculate the pivot angles
            float leftPivotAngle = atan2(OUTER_WHEEL_DY, (turnRadius + OUTER_WHEEL_DX));
            float rightPivotAngle = atan2(OUTER_WHEEL_DY, (turnRadius - OUTER_WHEEL_DX));

            if (turnRadius < 0.0) {  // Correct to [-90deg, 90deg]
                leftPivotAngle -= PI;
                rightPivotAngle -= PI;
            }

            // Calculate motor speeds
            float outerVel = (turnRadius > 0 ? velLeft : velRight);
            float farthestMotorDist = dist(abs(turnRadius) + OUTER_WHEEL_DX, OUTER_WHEEL_DY);
            float speedPerDist = outerVel / (abs(turnRadius) + INNER_WHEEL_DX);
            float fastestMotorSpeed = farthestMotorDist * speedPerDist;
            if (fastestMotorSpeed > 1.0)  // Hit brownout
            {
                speedPerDist = 1.0 / farthestMotorDist;
            }
            float leftOuterSpeed = speedPerDist * dist(turnRadius + OUTER_WHEEL_DX, OUTER_WHEEL_DY);
            float leftInnerSpeed = speedPerDist * abs(turnRadius + INNER_WHEEL_DX);
            float rightOuterSpeed = speedPerDist * dist(turnRadius - OUTER_WHEEL_DX, OUTER_WHEEL_DY);
            float rightInnerSpeed = speedPerDist * abs(turnRadius - INNER_WHEEL_DX);

            // Set the pivot values
            pivotPids[FRONT_LEFT].setTarget(-leftPivotAngle);
            pivotPids[BACK_LEFT].setTarget(leftPivotAngle);
            pivotPids[FRONT_RIGHT].setTarget(rightPivotAngle);
            pivotPids[BACK_RIGHT].setTarget(-rightPivotAngle);

            // Set the driving values
            driveMotors[FRONT_LEFT].setPower(-leftOuterSpeed);
            driveMotors[BACK_LEFT].setPower(-leftOuterSpeed);
            driveMotors[CENTER_LEFT].setPower(-leftInnerSpeed);
            driveMotors[FRONT_RIGHT].setPower(rightOuterSpeed);
            driveMotors[BACK_RIGHT].setPower(rightOuterSpeed);
            driveMotors[CENTER_RIGHT].setPower(rightInnerSpeed);
            Serial.printf("CENTER|LO: %.2f, LI: %.2f, RO: %.2f, RI: %.2f\n", leftOuterSpeed, leftInnerSpeed, rightOuterSpeed, rightInnerSpeed);
        }
    }
}

void MoonBot::stop()
{
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
inline float MoonBot::dist(float x1, float x2) { return sqrt(x1 * x1 + x2 * x2); }