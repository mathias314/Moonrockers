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


#define isPos(x) ((x) > 0 ? true : false)
#define numMotors 10
#define numDriveMotors 6

char readVal = '\0';
float speed = 0;
bool inverted = false;

float sampleTime = 0.005;


enum motorLocation {BRD, MRD, FRD, BLD, MLD, FLD};

PwmMotor motors[6] = {
    {19, 22},   // BRD
    {18, 30},   // MRD
    {18, 34},   // FRD
    {16, 42},   // BLD
    {14, 50},   // MLD
    {2, A15}    // FLD
};

Encoder encoders[6] = {
    {23, DRIVE_TACH_RATE},  // BRD
    {33, DRIVE_TACH_RATE},  // MRD
    {35, DRIVE_TACH_RATE},  // FRD
    {41, DRIVE_TACH_RATE},  // BLD
    {43, DRIVE_TACH_RATE},  // MLD
    {A14, DRIVE_TACH_RATE}  // FLD
};

//=====setup==============================
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(2500);

    // Initialize motors and encoders
    for (int i = 0; i < numDriveMotors; i++) {
        //motors[i].init();
        //motors[i].run(.75);
        encoders[i].init();
    }
    //pinMode(19, OUTPUT);
    analogWrite(15, 256/2);
}

//=====loop==============================
void loop() {
    static unsigned long lastDisplay = millis();
    if (millis() - lastDisplay > 50) {
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
        Serial.println(encoders[FLD].getSpeed());
        lastDisplay = millis();
    }

    static unsigned long lastUpdate = millis();
    if (millis() - lastUpdate > 1000 * sampleTime) {
        // Use the sign of the motor's power to inform direction
        for (int i = 0; i < numDriveMotors; i++) {
            encoders[i].estimateSpeed();
        }
        lastUpdate = millis();
    } 
}
