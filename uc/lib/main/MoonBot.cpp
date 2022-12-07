#include "MoonBot.h"

/**
 * @brief Constructor for the BEEG class.
 *
 * @param updateInterval - Interval in seconds to run the control loop.
 */
MoonBot::MoonBot(float updateInterval)
    : angleSensors{{FL_PIVOT_SENSOR_PIN, ANGLE_SENSOR_CALIBRATIONS[0]},
                   {FR_PIVOT_SENSOR_PIN, ANGLE_SENSOR_CALIBRATIONS[1]},
                   {BL_PIVOT_SENSOR_PIN, ANGLE_SENSOR_CALIBRATIONS[2]},
                   {BR_PIVOT_SENSOR_PIN, ANGLE_SENSOR_CALIBRATIONS[3]}},
      driveMotors{FL_MTR_ID, FR_MTR_ID, BL_MTR_ID, BR_MTR_ID, CL_MTR_ID, CR_MTR_ID},
      pivotMotors{FL_PIVOT_MTR_ID, FR_PIVOT_MTR_ID, BL_PIVOT_MTR_ID, BR_PIVOT_MTR_ID},
      pivotPids{{PIV_VEL_KP, PIV_VEL_KI, PIV_VEL_KD, PIV_VEL_N, updateInterval},
                {PIV_VEL_KP, PIV_VEL_KI, PIV_VEL_KD, PIV_VEL_N, updateInterval},
                {PIV_VEL_KP, PIV_VEL_KI, PIV_VEL_KD, PIV_VEL_N, updateInterval},
                {PIV_VEL_KP, PIV_VEL_KI, PIV_VEL_KD, PIV_VEL_N, updateInterval}},
      leftLeds(NUM_LEFT_LEDS, LEFT_LED_PIN, NEO_GRB + NEO_KHZ800),
      rightLeds(NUM_RIGHT_LEDS, RIGHT_LED_PIN, NEO_GRB + NEO_KHZ800) 
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
        pivotPids[i].setLimits(-PIVOT_MAX_PWR, PIVOT_MAX_PWR);
        pivotPids[i].setTargetLimits(-PIVOT_VELOCITY, PIVOT_VELOCITY);
        pivotPids[i].setTarget(0);
    }

    leftLeds.begin();
    rightLeds.begin();
    leftLeds.fill(Adafruit_NeoPixel::Color(0, 0, 255));
    rightLeds.fill(Adafruit_NeoPixel::Color(0, 0, 255));
    leftLeds.show();
    rightLeds.show();
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
    static unsigned long debugTime = millis();
    bool debugPrint = false;
    // Send keepalive to motors
    if (millis() - lastKeepAliveTime > KEEP_ALIVE_INTERVAL) {
        FrcMotorController::sendKeepAlive();
        lastKeepAliveTime = millis();
    }

    // Update pivots
    if (micros() - lastUpdateTime > updateInterval*1000000) {
        float error;
        float targetVel;
        float output;
        for (unsigned i = 0; i < NUM_PIVOTS; i++) {
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

void MoonBot::setMotorPower(Joint motor, float power) { 
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
void MoonBot::setPivotAngle(Joint pivot, float angle) { 
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
void MoonBot::setPivotPIDConstants(Joint pivot, float kp, float ki, float kd, float N)
{
    pivotPids[pivot].setConstants(kp, ki, kd, N, updateInterval);
}

void MoonBot::drive(float velLeft, float velRight) {  // Weeeeeeeeeeeeeeee.
    const float outerWheelDist = dist(OUTER_WHEEL_DY, OUTER_WHEEL_DX);
    const float MIN_TURN_RAD = OUTER_WHEEL_DX;
    static unsigned long ledUpdateTime = millis();

    if (millis() - ledUpdateTime > 100) {
        rightLeds.fill(driveDirToColor(velRight));
        leftLeds.fill(driveDirToColor(velLeft));
        
        leftLeds.show();
        rightLeds.show();
    }

    if (abs(velLeft - velRight) < 5e-2) {  // Drive straight (Have to special case this cause infinite turn radius)
        float powerVal = (velLeft + velRight) / 2.0;
        for (unsigned i = 0; i < NUM_DRIVE_MOTORS; i++) {
            if (i % 2)
            {
                driveMotors[i].setPower(powerVal*0.7);
            }
            else
            {
                driveMotors[i].setPower(-powerVal*0.7);
            }
            
        }
        // Set all to straight
        for (unsigned i = 0; i < NUM_PIVOTS; i++) {
            setPivotAngle((Joint)i, 0);
        }
    } else {
        float turnRadius = (velRight + velLeft) / (velLeft - velRight) * INNER_WHEEL_DX;

        if (abs(turnRadius) < INNER_WHEEL_DX / 2.0) {  // Center turn
            // Set pivot angles
            const float wheelAngle = atan2(OUTER_WHEEL_DY, OUTER_WHEEL_DX);
            setPivotAngle(FRONT_LEFT, -wheelAngle);
            setPivotAngle(FRONT_RIGHT, wheelAngle);
            setPivotAngle(BACK_LEFT, wheelAngle);
            setPivotAngle(BACK_RIGHT, -wheelAngle);

            // Set drive motor speeds
            float innerSpeed = (velLeft - velRight) / 2.0;
            float outerSpeed = innerSpeed * outerWheelDist / INNER_WHEEL_DX;

            if (abs(outerSpeed) > DRIVE_MAX_PWR)  // Handle brownout
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
            if (abs(fastestMotorSpeed) > DRIVE_MAX_PWR)  // Hit brownout
            {
                speedPerDist = (fastestMotorSpeed > 0 ? DRIVE_MAX_PWR : - DRIVE_MAX_PWR) / farthestMotorDist;
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

void MoonBot::stop()
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
inline float MoonBot::dist(float x1, float x2) { return sqrt(x1 * x1 + x2 * x2); }

/**
 * Give an appropriate neopixel color for the given velocity.
 * 
 * @param vel - velocity.
 * @return Neopixel color value. 
 */
uint32_t MoonBot::driveDirToColor(float vel){
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