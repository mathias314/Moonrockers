/**
 * PWM so we don't have to deal with timer interrupts.
 * Motor is controlled by the following commands over serial:
 *     - ' ' = STOP
 *     - '+' = Increase power by 10%.
 *     - '-' = Decrease power by 10%.
 *     - 'i' = Invert motor direction.
 *
 * To Run:
 *      pio run -t upload -c examples.ini -e SAM_test
 * */
#include <Arduino.h>
#include <math.h>
#include "PwmMotor.h"
#include "globals.h"
#include "Encoder.h"
#include "PID.h"
#include "Potentiometer.h"

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

float time = 0;

//=====setup==============================
void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(2500);

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
        drivePids[i].setTargetLimits(-30, 30);
        drivePids[i].setLimits(-1.0, 1.0);
    }

    // Initialize angle PID
    for (int i = 0; i < NUM_STEER_MOTORS; i++)
    {
        steerPositionPids[i].setTarget(0);
        steerPositionPids[i].setTargetLimits(-PI, PI);
        steerPositionPids[i].setLimits(-70, 70);

        steerVelocityPids[i].setTarget(0);
        steerVelocityPids[i].setTargetLimits(-70, 70);
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
    unsigned long before;
    unsigned long after;
    // Read serial input
    static bool running = true;

    if (Serial.available())
    {
        readVal = Serial.read();
        while (Serial.available())
        {
            Serial.read();
        }
        switch (readVal)
        {
        case '+':
        case '=':
            for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
            {
            }
            running = true;
            break;

        case '-':
        case '_':
            for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
            {
                drivePids[i].setTarget(drivePids[i].getTarget() - 20);
            }
            running = true;
            break;

        case 'w':
            for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
            {
                drivePids[i].setTarget(200);
            }
            running = true;
            break;

        case 's':
            for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
            {
                drivePids[i].setTarget(-200);
            }
            running = true;
            break;
        case 'a':
            steerPositionPids[BR].setTarget(steerPositionPids[BR].getTarget() + 0.3);
            steerPositionPids[FR].setTarget(steerPositionPids[FR].getTarget() - 0.3);
            steerPositionPids[BL].setTarget(steerPositionPids[BL].getTarget() + 0.3);
            steerPositionPids[FL].setTarget(steerPositionPids[FL].getTarget() - 0.3);
            running = true;
            break;

        case 'd':
            steerPositionPids[BR].setTarget(steerPositionPids[BR].getTarget() - 0.3);
            steerPositionPids[FR].setTarget(steerPositionPids[FR].getTarget() + 0.3);
            steerPositionPids[BL].setTarget(steerPositionPids[BL].getTarget() - 0.3);
            steerPositionPids[FL].setTarget(steerPositionPids[FL].getTarget() + 0.3);
            running = true;
            break;
            /*
                                case 'p':
                                    for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
                                    {
                                        DRIVE_PID_PARAMS[i][KP] += 0.01;
                                        drivePids[i].setConstants(DRIVE_PID_PARAMS[i][KP], DRIVE_PID_PARAMS[i][KI], DRIVE_PID_PARAMS[i][KD], DRIVE_PID_PARAMS[i][N], SAMPLE_TIME);
                                    }
                                    break;

                                case ';':
                                    for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
                                    {
                                        DRIVE_PID_PARAMS[i][KP] -= 0.01;
                                        drivePids[i].setConstants(DRIVE_PID_PARAMS[i][KP], DRIVE_PID_PARAMS[i][KI], DRIVE_PID_PARAMS[i][KD], DRIVE_PID_PARAMS[i][N], SAMPLE_TIME);
                                    }
                                    break;

                                case 'i':
                                    for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
                                    {
                                        DRIVE_PID_PARAMS[i][KI] += 0.01;
                                        drivePids[i].setConstants(DRIVE_PID_PARAMS[i][KP], DRIVE_PID_PARAMS[i][KI], DRIVE_PID_PARAMS[i][KD], DRIVE_PID_PARAMS[i][N], SAMPLE_TIME);
                                    }
                                    break;

                                case 'k':
                                    for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
                                    {
                                        DRIVE_PID_PARAMS[i][KI] -= 0.01;
                                        drivePids[i].setConstants(DRIVE_PID_PARAMS[i][KP], DRIVE_PID_PARAMS[i][KI], DRIVE_PID_PARAMS[i][KD], DRIVE_PID_PARAMS[i][N], SAMPLE_TIME);
                                    }
                                    break;

                                case 'd':
                                    for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
                                    {
                                        DRIVE_PID_PARAMS[i][KD] += 0.01;
                                        drivePids[i].setConstants(DRIVE_PID_PARAMS[i][KP], DRIVE_PID_PARAMS[i][KI], DRIVE_PID_PARAMS[i][KD], DRIVE_PID_PARAMS[i][N], SAMPLE_TIME);
                                    }
                                    break;

                                case 'c':
                                    for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
                                    {
                                        DRIVE_PID_PARAMS[i][KD] -= 0.01;
                                        drivePids[i].setConstants(DRIVE_PID_PARAMS[i][KP], DRIVE_PID_PARAMS[i][KI], DRIVE_PID_PARAMS[i][KD], DRIVE_PID_PARAMS[i][N], SAMPLE_TIME);
                                    }
                                    break;
                        */

        case ']':
            POTENTIOMETER_OFFSETS[BR] += 0.1;
            break;

        case '[':
            POTENTIOMETER_OFFSETS[BR] -= 0.1;
            break;
        case 'q':
        case 'e':
        case 'z':
            for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
            {
                drivePids[i].setTarget(0);
            }

            for (int i = 0; i < NUM_STEER_MOTORS; i++)
            {
                steerPositionPids[i].setTarget(0);
                steerVelocityPids[i].setTarget(0);
            }

            for (int i = 0; i < NUM_MOTORS; i++)
            {
                motors[i].run(0);
            }
            break;

        default:
            break;
        }
    }

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
        /*
        Serial.print("Target:");
        Serial.print(drivePids[FLD].getTarget());
        Serial.print("  Power:");
        Serial.print(motors[FLD].getPower());
        Serial.print("  Speed:");
        Serial.print(encoders[FLD].estimateSpeed());
        Serial.print("  kP:");
        Serial.print(DRIVE_PID_PARAMS[FLD][KP]);
        Serial.print("  kI:");
        Serial.print(DRIVE_PID_PARAMS[FLD][KI]);
        Serial.print("  kD:");
        Serial.print(DRIVE_PID_PARAMS[FLD][KD]);
        */

/*
        Serial.print("  PositionL:");
        Serial.print(potentiometers[BL].getAngle());
        Serial.print("  TargetL:");
        Serial.print(steerPositionPids[BL].getTarget());
        Serial.print("  PosOutL:");
        Serial.print(positionPidOutput[BL]);
        Serial.print("  VelOutL:");
        Serial.print(velocityPidOutput[BL]);
        Serial.print("  VelL:");
        Serial.print(encoders[BLS].getFilteredSpeed());

        Serial.print("  PositionF:");
        Serial.print(potentiometers[FL].getAngle());
        Serial.print("  TargetF:");
        Serial.print(steerPositionPids[FL].getTarget());
        Serial.print("  PosOutF:");
        Serial.print(positionPidOutput[FL]);
        Serial.print("  VelOutF:");
        Serial.print(velocityPidOutput[FL]);
        Serial.print("  VelF:");
        Serial.print(encoders[FLS].getFilteredSpeed());
        Serial.print("     ");
        */
        Serial.print(potentiometers[BR].getAngle());
        Serial.print(" ");
        Serial.print(potentiometers[FR].getAngle());
        Serial.print(" ");
        Serial.print(potentiometers[BL].getAngle());
        Serial.print(" ");
        Serial.print(potentiometers[FL].getAngle());
        Serial.print(" ");
        Serial.print(POTENTIOMETER_OFFSETS[BR]);
        /*
         Serial.print("FL:");
         Serial.print(potentiometers[FL].getAngle());
         Serial.print("\tFR:");
         Serial.print(potentiometers[FR].getAngle());
         Serial.print("\tBL:");
         Serial.print(potentiometers[BL].getAngle());
         Serial.print("\tBR:");
         Serial.print(potentiometers[BR].getAngle());
        */
        Serial.println();

        lastDisplay = millis();
    }

    static unsigned long lastUpdate = millis();
    if (millis() - lastUpdate > 1000 * SAMPLE_TIME)
    {
        // Update drive tachometer velocities and use the sign of the motor's power to infer direction
        for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
        {
            encoders[i].estimateSpeed(isPos(drivePids[i].getTarget()));
        }

        // Run drive motors based on PID instructions
        for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
        {
            motors[i].run(drivePids[i].calculateOutput(encoders[i].getFilteredSpeed()));
        }

        // Update potentiometers
        for (int i = 1; i < NUM_STEER_MOTORS; i++)
        {
            potentiometers[i].update();
        }
        // update unique potentiometer with offset
        potentiometers[BR].update(POTENTIOMETER_OFFSETS[BR]);

        // Update steering tachometer velocities and use the sign of the motor's power to infer direction
        for (int i = 0; i < NUM_STEER_MOTORS; i++)
        {
            encoders[i + NUM_DRIVE_MOTORS].estimateSpeed(isPos(steerVelocityPids[i].getTarget()));
        }

        // Calculate velocities for steering motors
        for (int i = 0; i < NUM_STEER_MOTORS; i++)
        {
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
}
