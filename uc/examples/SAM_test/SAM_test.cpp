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

float kP = 0.01, kI = 0.02, kD = 0.0, N = 0.0, sampleTime = 0.005;

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

/*
// FOR MEGA ONLY
PwmMotor motors[10] = {
    {0, 0}, // BRD x
    {0, 0},  // FRD x
    {4, 30},  // BLD x
    {0, 0}, // FLD x
    {0, 0},  // MRD x
    {0, 0},  // MLD x
    {0, 0},  // BRS x
    {0, 0},  // FRS x
    {0, 0},  // BLS x
    {0, 0}   // FLS x
};
*/

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
    {kP, kI, kD, N, sampleTime},
    {kP, kI, kD, N, sampleTime},
    {kP, kI, kD, N, sampleTime},
    {kP, kI, kD, N, sampleTime},
    {kP, kI, kD, N, sampleTime},
    {kP, kI, kD, N, sampleTime}};

PID anglePids[4] = {
    {kP, kI, kD, N, sampleTime},
    {kP, kI, kD, N, sampleTime},
    {kP, kI, kD, N, sampleTime},
    {kP, kI, kD, N, sampleTime}};

PID steerPids[4] = {
    {kP, kI, kD, N, sampleTime},
    {kP, kI, kD, N, sampleTime},
    {kP, kI, kD, N, sampleTime},
    {kP, kI, kD, N, sampleTime}};

/*
// FOR MEGA ONLY
Encoder encoders[10] = {
    {0, DRIVE_TACH_RATE}, // BRD x
    {0, DRIVE_TACH_RATE}, // FRD x
    {19, DRIVE_TACH_RATE}, // BLD x
    {0, DRIVE_TACH_RATE}, // FLD x
    {0, DRIVE_TACH_RATE}, // MRD x
    {0, DRIVE_TACH_RATE}, // MLD x
    {0, STEER_TACH_RATE}, // BRS x
    {0, STEER_TACH_RATE}, // FRS x
    {0, STEER_TACH_RATE}, // BLS x
    {0, STEER_TACH_RATE}  // FLS x
};
*/

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

//=====setup==============================
void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(2500);

    // Initialize motors and encoders
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        motors[i].init();
        motors[i].setTarget(0);
        motors[i].run(0);
        encoders[i].init();
    }

    // Set motors on right side to inverted so they spin in the same direction as the other wheels
    motors[FRD].setInverted(true);
    motors[MRD].setInverted(true);
    motors[BRD].setInverted(true);

    for (int i = 0; i < 4; i++)
    {
        potentiometers[i].init();
    }

    // Initialize drive PID
    for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
    {
        drivePids[i].setTarget(0);
        drivePids[i].setTargetLimits(-60, 60);
        drivePids[i].setLimits(-1.0, 1.0);
    }

    // Initialize angle PID
    for (int i = 0; i < NUM_STEER_MOTORS; i++)
    {
        anglePids[i].setTarget(0);
        anglePids[i].setTargetLimits(-PI / 2, PI / 2);
        anglePids[i].setLimits(-60, 60);

        steerPids[i].setTarget(0);
        steerPids[i].setTargetLimits(-200, 200);
        steerPids[i].setLimits(-1.0, 1.0);
    }

    while (1) {
        for (int i = -100; i < 100; i++) {
            motors[FRD].run(i/100.0);
            encoders[FRD].estimateSpeed();
            Serial.println(encoders[FRD].getFilteredSpeed());
            delay(100);
        }
        for (int i = 100; i > -100; i--) {
            motors[FRD].run(i/100.0);
            encoders[FRD].estimateSpeed();
            Serial.println(encoders[FRD].getFilteredSpeed());
            delay(100);
        }
    }
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

        case 'z':
            // motors[motorIndex].setTarget(0.0);
            for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
            {
                drivePids[i].setTarget(0);
                motors[i].run(0);
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
        Serial.print(" Power:");
        Serial.print(motors[FRD].getPower());
        Serial.print(", Target:");
        Serial.print(drivePids[FRD].getTarget());
        Serial.print(", Velocity:");
        Serial.println(encoders[FRD].getFilteredSpeed());

        lastDisplay = millis();
    }

    static unsigned long lastUpdate = millis();
    if (millis() - lastUpdate > 1000 * sampleTime)
    {
        // Update tachometer velocities and use the sign of the motor's power to infer direction
        for (int i = 0; i < NUM_MOTORS; i++)
        {
            encoders[i].estimateSpeed(isPos(drivePids[i].getTarget()));
        }

        // Update potentiometers
        for (int i = 0; i < 4; i++)
        {
            potentiometers[i].update();
        }

        // Run drive motors based on PID instructions
        for (int i = 0; i < NUM_DRIVE_MOTORS; i++)
        {
            motors[i].run(drivePids[i].calculateOutput(encoders[i].getFilteredSpeed()));
        }

        // Run steering motors based on potentiometer readings
        /*
        for (int i = 0; i < NUM_STEER_MOTORS; i++)
        {
            float targetVelocity = anglePids[i].calculateOutput(potentiometers[i].getAngle());
            motors[i + NUM_DRIVE_MOTORS].run(steerPids[i].calculateOutput(targetVelocity));
        }
        */

        lastUpdate = millis();
    }
}
