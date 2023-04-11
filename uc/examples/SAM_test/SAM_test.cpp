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

// enum motorLocation {BRD, MRD, FRD, BLD, MLD, FLD, BRS, FRS, BLS, BLS};
enum motorLocation
{
    BRD,
    FRD,
    BLD,
    FLD,
    MRD,
    MLD,
    BRS,
    FRS,
    BLS,
    FLS
};

PwmMotor motors[10] = {
    {13, 34}, // BRD x
    {6, 48},  // FRD x
    {5, 30},  // BLD x
    {14, 22}, // FLD x
    {8, 42},  // MRD x
    {3, 26},  // MLD x
    {9, 40},  // BRS x
    {7, 44},  // FRS x
    {4, 28},  // BLS x
    {2, 24}   // FLS x
};

Encoder encoders[10] = {
    {35, DRIVE_TACH_RATE}, // BRD x
    {49, DRIVE_TACH_RATE}, // FRD x
    {31, DRIVE_TACH_RATE}, // BLD x
    {23, DRIVE_TACH_RATE}, // FLD x
    {43, DRIVE_TACH_RATE}, // MRD x
    {27, DRIVE_TACH_RATE}, // MLD x
    {41, STEER_TACH_RATE}, // BRS x
    {45, STEER_TACH_RATE}, // FRS x
    {29, STEER_TACH_RATE}, // BLS x
    {25, STEER_TACH_RATE}  // FLS x
};

PID drivePids[6] = {
    {DRIVE_PID_PARAMS[KP], DRIVE_PID_PARAMS[KI], DRIVE_PID_PARAMS[KD], DRIVE_PID_PARAMS[N], SAMPLE_TIME},
    {DRIVE_PID_PARAMS[KP], DRIVE_PID_PARAMS[KI], DRIVE_PID_PARAMS[KD], DRIVE_PID_PARAMS[N], SAMPLE_TIME},
    {DRIVE_PID_PARAMS[KP], DRIVE_PID_PARAMS[KI], DRIVE_PID_PARAMS[KD], DRIVE_PID_PARAMS[N], SAMPLE_TIME},
    {DRIVE_PID_PARAMS[KP], DRIVE_PID_PARAMS[KI], DRIVE_PID_PARAMS[KD], DRIVE_PID_PARAMS[N], SAMPLE_TIME},
    {DRIVE_PID_PARAMS[KP], DRIVE_PID_PARAMS[KI], DRIVE_PID_PARAMS[KD], DRIVE_PID_PARAMS[N], SAMPLE_TIME},
    {DRIVE_PID_PARAMS[KP], DRIVE_PID_PARAMS[KI], DRIVE_PID_PARAMS[KD], DRIVE_PID_PARAMS[N], SAMPLE_TIME}};

PID steerPositionPids[4] = {
    {POS_PID_PARAMS[KP], POS_PID_PARAMS[KI], POS_PID_PARAMS[KD], POS_PID_PARAMS[N], SAMPLE_TIME},
    {POS_PID_PARAMS[KP], POS_PID_PARAMS[KI], POS_PID_PARAMS[KD], POS_PID_PARAMS[N], SAMPLE_TIME},
    {POS_PID_PARAMS[KP], POS_PID_PARAMS[KI], POS_PID_PARAMS[KD], POS_PID_PARAMS[N], SAMPLE_TIME},
    {POS_PID_PARAMS[KP], POS_PID_PARAMS[KI], POS_PID_PARAMS[KD], POS_PID_PARAMS[N], SAMPLE_TIME}};

PID steerVelocityPids[4] = {
    {STEER_PID_PARAMS[KP], STEER_PID_PARAMS[KI], STEER_PID_PARAMS[KD], STEER_PID_PARAMS[N], SAMPLE_TIME},
    {STEER_PID_PARAMS[KP], STEER_PID_PARAMS[KI], STEER_PID_PARAMS[KD], STEER_PID_PARAMS[N], SAMPLE_TIME},
    {STEER_PID_PARAMS[KP], STEER_PID_PARAMS[KI], STEER_PID_PARAMS[KD], STEER_PID_PARAMS[N], SAMPLE_TIME},
    {STEER_PID_PARAMS[KP], STEER_PID_PARAMS[KI], STEER_PID_PARAMS[KD], STEER_PID_PARAMS[N], SAMPLE_TIME}};

enum potLocation
{
    BR,
    FR,
    BL,
    FL
};

Potentiometer potentiometers[4] = {
    {A0}, // BR
    {A6}, // FR
    {A4}, // BL
    {A2}  // FL
};

float positionPidOutput[4] = {0, 0, 0, 0};
float velocityPidOutput[4] = {0, 0, 0, 0};


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
        drivePids[i].setTargetLimits(-40, 40);
        drivePids[i].setLimits(-1.0, 1.0);
    }

    // Initialize angle PID
    for (int i = 0; i < NUM_STEER_MOTORS; i++)
    {
        steerPositionPids[i].setTarget(0);
        steerPositionPids[i].setTargetLimits(-PI, PI);
        steerPositionPids[i].setLimits(-40, 40);

        steerVelocityPids[i].setTarget(0);
        steerVelocityPids[i].setTargetLimits(-40, 40);
        steerVelocityPids[i].setLimits(-1.0, 1.0);
    }

    // Set motors on right side to inverted so they spin in the same direction as the other wheels
    motors[FRD].setInverted(true);
    motors[MRD].setInverted(true);
    motors[BRD].setInverted(true);
    motors[FRS].setInverted(true);
    motors[FLS].setInverted(true);
    motors[BLS].setInverted(true);

    motors[BLS].setMinimum(MOTOR_MINIMUM_SPEEDS[BLS]);
}

//=====loop==============================
void loop()
{
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
                drivePids[i].setTarget(drivePids[i].getTarget() + 20);
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

        case 'f':
            for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
            {
                drivePids[i].setTarget(200);
            }
            running = true;
            break;

        case 'm':
            for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
            {
                drivePids[i].setTarget(-200);
            }
            running = true;
            break;

        case 'u':
            steerPositionPids[BL].setTarget(steerPositionPids[BL].getTarget() + 0.3);
            running = true;
            break;

        case 'd':
            steerPositionPids[BL].setTarget(steerPositionPids[BL].getTarget() - 0.3);
            running = true;
            break;

        case 'z':
            for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
            {
                drivePids[i].setTarget(0);
                motors[i].run(0);
            }

            for (int i = 0; i < NUM_STEER_MOTORS; i++)
            {
                steerPositionPids[i].setTarget(0);
                steerVelocityPids[i].setTarget(0);
                motors[NUM_DRIVE_MOTORS + i].run(0);
            }
            running = true;
            break;

        default:
            break;
        }
    }

    static unsigned long lastDisplay = millis();
    if (millis() - lastDisplay > 50)
    {
        Serial.print("Min:");
        Serial.print(motors[BLS].getMinimum());
        Serial.print("\tPosition:");
        Serial.print(potentiometers[BL].getAngle());
        Serial.print("\tTarget:");
        Serial.print(steerPositionPids[BL].getTarget());
        Serial.print("\tPosOut:");
        Serial.print(positionPidOutput[BL]);
        Serial.print("\tVelOut:");
        Serial.print(velocityPidOutput[BL]);
        Serial.print("\tVel:");
        Serial.print(encoders[BLS].getFilteredSpeed());
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

        // Update steering tachometer velocities and use the sign of the motor's power to infer direction
        for (int i = 0; i < NUM_STEER_MOTORS; i++)
        {
            encoders[i + NUM_DRIVE_MOTORS].estimateSpeed(isPos(steerVelocityPids[i].getTarget()));
        }

        // Update potentiometers
        for (int i = 0; i < NUM_STEER_MOTORS; i++)
        {
            potentiometers[i].update();
        }

        for (int i = 0; i < NUM_STEER_MOTORS; i++)
        {
            positionPidOutput[i] = steerPositionPids[i].simpleOutput(potentiometers[i].getAngle());
            steerVelocityPids[i].setTarget(positionPidOutput[i]);
            velocityPidOutput[i] = steerVelocityPids[i].calculateOutput(encoders[i + NUM_DRIVE_MOTORS].getFilteredSpeed());
        }

        motors[BLS].run(velocityPidOutput[BL]);

        // Run drive motors based on PID instructions
        /*
        for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
        {
            motors[i].run(drivePids[i].calculateOutput(encoders[i].getFilteredSpeed()));
        }
        */

        // Run steering motors based on potentiometer readings
        /*
        for (int i = 0; i < NUM_STEER_MOTORS; i++)
        {
            float targetVelocity = steerPositionPids[i].calculateOutput(potentiometers[i].getAngle());
            motors[i + NUM_DRIVE_MOTORS].run(steerVelocityPids[i].calculateOutput(targetVelocity));
        }
        */

        /*
        for (int i = 0; i < NUM_STEER_MOTORS; i++)
        {
            potentiometers[i].update();
            targetSteerVelocites[i] = steerPositionPids[i].calculateOutput(potentiometers[i].getAngle());
            outputSteerVelocities[i] = steerVelocityPids[i].calculateOutput(targetSteerVelocites[i]);
            motors[i + NUM_DRIVE_MOTORS].run(outputSteerVelocities[i]);
        }
        */
        lastUpdate = millis();
    }
}
