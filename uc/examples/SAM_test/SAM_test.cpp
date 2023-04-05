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
#define numMotors 10
#define numDriveMotors 6

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
    {kP, kI, kD, N, sampleTime}
};

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
    for (int i = 0; i < numMotors; i++)
    {
        motors[i].init();
        motors[i].setTarget(0);
        motors[i].run(0);
        encoders[i].init();
    }

    for (int i = 0; i < 4; i++)
    {
        potentiometers[i].init();
    }

    // Initialize PID
    for (int i = 0; i < numDriveMotors; i++)
    {
        drivePids[i].setTarget(0);
        drivePids[i].setTargetLimits(-230, 230);
        drivePids[i].setLimits(-1.0, 1.0);
    }
}

//=====loop==============================
void loop()
{
    // Read serial input
    static bool running = false;
    int motorIndex = BLD;

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
            drivePids[motorIndex].setTarget(drivePids[motorIndex].getTarget() + 20);
            //motors[motorIndex].run(0.75);
            running = true;
            break;
        case '-':
        case '_':
            drivePids[motorIndex].setTarget(drivePids[motorIndex].getTarget() - 20);
            //motors[motorIndex].run(-0.75);
            running = true;
            break;

        case 'f':
            //motors[motorIndex].setTarget(1.0);
            drivePids[motorIndex].setTarget(230);
            running = true;
            break;

        case 'm':
            //motors[motorIndex].setTarget(-1.0);
            drivePids[motorIndex].setTarget(-230);
            running = true;
            break;

        case 'z':
            //motors[motorIndex].setTarget(0.0);
            drivePids[motorIndex].setTarget(0.0);
            running = true;
            break;

        default:
            break;
        }
    }

    drivePids[motorIndex].calculateCoeffs(encoders[motorIndex].getFilteredSpeed());

    static unsigned long lastDisplay = millis();
    if (millis() - lastDisplay > 50)
    {
        Serial.print("Power:");
        Serial.print(motors[motorIndex].getPower() * 230);
        Serial.print(", Target:");
        Serial.print(drivePids[motorIndex].getTarget());
        Serial.print(", Encoder:");
        Serial.println(encoders[motorIndex].getFilteredSpeed());
        lastDisplay = millis();
    }

    static unsigned long lastUpdate = millis();
    if (millis() - lastUpdate > 1000 * sampleTime)
    {
        // Use the sign of the motor's power to inform direction
        /*
        for (int i = 0; i < numMotors; i++)
        {
            //motors[i].runTarget();
            encoders[i].estimateSpeed();
        }
        for (int i = 0; i < 4; i++)
        {
            potentiometers[i].update();
        }
        */
        encoders[motorIndex].estimateSpeed();
        motors[motorIndex].run(drivePids[motorIndex].calculateOutput(encoders[motorIndex].getFilteredSpeed()));
        lastUpdate = millis();
    }
}
