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

float sampleTime = 0.005;

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

enum potLocation {BR, FR, BL, FL};
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
        motors[i].run(.75);
        encoders[i].init();
    }

    for (int i = 0; i < 4; i++)
    {
        potentiometers[i].init();
    }
}

//=====loop==============================
void loop()
{
    static unsigned long lastDisplay = millis();
    if (millis() - lastDisplay > 50)
    {
        /*
        Serial.print("BRD:");
        Serial.print(encoders[BRD].getSpeed());
        Serial.print(", MRD:");
        Serial.print(encoders[MRD].getSpeed());
        Serial.print(", FRD:");
        Serial.print(encoders[FRD].getSpeed());
        Serial.print(", BLD:");
        Serial.print(encoders[BLD].getSpeed());
        Serial.print(", MLD:");
        Serial.print(encoders[MLD].getSpeed());
        Serial.print(", FLD:");
        Serial.print(encoders[FLD].getSpeed());
        Serial.print(", FLS:");
        Serial.print(encoders[FLS].getSpeed());
        Serial.print(", BLS:");
        Serial.print(encoders[BLS].getSpeed());
        Serial.print(", FRS:");
        Serial.print(encoders[FRS].getSpeed());
        Serial.print(", BRS:");
        Serial.println(encoders[BRS].getSpeed());
        */
        Serial.print("BR:");
        Serial.print(potentiometers[BR].getAngle() * 57.2958);
        Serial.print(", FR:");
        Serial.print(potentiometers[FR].getAngle() * 57.2958);
        Serial.print(", BL:");
        Serial.print(potentiometers[BL].getAngle() * 57.2958);
        Serial.print(", FL:");
        Serial.print(potentiometers[FL].getAngle() * 57.2958);
        Serial.println();
        /*
        Serial.print("        ");
        Serial.print("BR:");
        Serial.print(potentiometers[BR].getVelocity());
        Serial.print(", FR:");
        Serial.print(potentiometers[FR].getVelocity());
        Serial.print(", BL:");
        Serial.print(potentiometers[BL].getVelocity());
        Serial.print(", FL:");
        Serial.println(potentiometers[FL].getVelocity());
        */

        lastDisplay = millis();
    }

    static unsigned long lastUpdate = millis();
    if (millis() - lastUpdate > 1000 * sampleTime)
    {
        // Use the sign of the motor's power to inform direction
        for (int i = 0; i < numMotors; i++)
        {
            encoders[i].estimateSpeed();
        }
        for (int i = 0; i < 4; i++)
        {
            potentiometers[i].update();
        }

        lastUpdate = millis();
    }
}
