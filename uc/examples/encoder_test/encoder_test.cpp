/**
 * PWM so we don't have to deal with timer interrupts. 
 * Motor is controlled by the following commands over serial:
 *     - ' ' = STOP  
 *     - '+' = Increase power by 10%.
 *     - '-' = Decrease power by 10%.
 *     - 'i' = Invert motor direction.
 *
 * To Run:
 *      pio run -t upload -c examples.ini -e encoder_test
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


PwmMotor motors[10] = {
    // Drive motors
    {/*speedPin*/ 2, /*dirPin*/ 22},    // Drive back left
    {/*speedPin*/ 3, /*dirPin*/ 23},    // Drive back right
    {/*speedPin*/ 4, /*dirPin*/ 24},    // Drive middle left
    {/*speedPin*/ 5, /*dirPin*/ 25},    // Drive middle right
    {/*speedPin*/ 6, /*dirPin*/ 26},    // Drive front left
    {/*speedPin*/ 7, /*dirPin*/ 27},    // Drive front right
    
    // Steer motors
    {/*speedPin*/ 8, /*dirPin*/ 28},    // Steer back left
    {/*speedPin*/ 9, /*dirPin*/ 29},    // Steer back right
    {/*speedPin*/ 10, /*dirPin*/ 30},   // Steer front left
    {/*speedPin*/ 11, /*dirPin*/ 31}    // Steer front right
};

Encoder encoders[10] = {
    // Drive encoders
    {/*PPR Pin*/ A8, /*PPRs*/ 324},     // Drive back left
    {/*PPR Pin*/ A9, /*PPRs*/ 324},     // Drive back right
    {/*PPR Pin*/ A10, /*PPRs*/ 324},    // Drive middle left
    {/*PPR Pin*/ A11, /*PPRs*/ 324},    // Drive middle right
    {/*PPR Pin*/ A12, /*PPRs*/ 324},    // Drive front left
    {/*PPR Pin*/ A13, /*PPRs*/ 324},    // Drive front right
    
    // Steer encoders
    {/*PPR Pin*/ 21, /*PPR*/ 324},     // Steer back left
    {/*PPR Pin*/ 20, /*PPR*/ 324},     // Steer back right
    {/*PPR Pin*/ 19, /*PPR*/ 324},     // Steer front left
    {/*PPR Pin*/ 18, /*PPR*/ 324}      // Steer front right
};

enum motorLocations {
    driveBackLeft,
    driveBackRight,
    driveMiddleLeft,
    driveMiddleRight,
    driveFrontLeft,
    driveFrontRight,
    steerBackLeft,
    steerBackRight,
    steerFrontLeft,
    steerFrontRight
};

float sampleTime = 0.005;
float stdDeviation = 0;

//=====setup==============================
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(2500);

    // Initialize motors and encoders
    Serial.println("Initializing motors...");
    for (int i = 0; i < numMotors; i++) {
        motors[i].init();
        if (!encoders[i].init()) {
            Serial.println("***Encoder Initialization failure***");
            while(1);
        }
    }
}

//=====loop==============================
void loop() {
    static bool running = false;
    
    if (Serial.available()) {
        readVal = Serial.read();
        while (Serial.available()) {
            Serial.read();
        }
        switch (readVal) {
            case ' ':
                for (int i = 0; i < numDriveMotors; i++) {
                    motors[i].stop();
                }
                Serial.println("STOP");
                running = false;
                break;
            case '+':
            case '=':
                for (int i = 0; i < numDriveMotors; i++) {
                    motors[i].setTarget(motors[i].getPower() + 0.1);
                }
                running = true;
                break;
            case '-':
            case '_':
                for (int i = 0; i < numDriveMotors; i++) {
                    motors[i].setTarget(motors[i].getPower() - 0.1);
                }
                running = true;
                break;

            case 'f':
                for (int i = 0; i < numDriveMotors; i++) {
                    motors[i].setTarget(1.0);
                }
                running = true;
                break;

            case 'm':
                for (int i = 0; i < numDriveMotors; i++) {
                    motors[i].setTarget(-1.0);
                }
                running = true;
                break;

            case 'z':
                for (int i = 0; i < numDriveMotors; i++) {
                    motors[i].setTarget(0.0);
                }
                running = true;
                break;

            case '0':
                break;

            default:
                break;
        }
        
    }

    static unsigned long lastDisplay = millis();
    if (millis() - lastDisplay > 50) {
        /*
        Serial.print("Speed1:");
        Serial.print(encoder1.getSpeed());
        Serial.print(",");
        Serial.print("Filtered1:");
        Serial.print(encoder1.getFilteredSpeed());
        Serial.print(",");
        Serial.print("Target1:");
        Serial.print(target1 * 230);
        Serial.print(",");

        Serial.print("Speed2:");
        Serial.print(encoder2.getSpeed());
        Serial.print(",");
        Serial.print("Filtered2:");
        Serial.print(encoder2.getFilteredSpeed());
        Serial.print(",");
        Serial.print("Target2:");
        Serial.println(target2 * 230);
        */
        Serial.print("Target:");
        Serial.print(motors[0].getTarget());
        Serial.print(",StdDev:");
        Serial.print(stdDeviation);

        lastDisplay = millis();
    } 

    static unsigned long lastUpdate = millis();
    if (millis() - lastUpdate > 1000 * sampleTime) {
        float mean = 0;
        for (int i = 0; i < numDriveMotors; i++) {
            mean += encoders[i].estimateSpeed(isPos(motors[i].getPower()));
            if (running) {
                motors[i].runTarget();
            }
        }

        // Calculate standard deviation of drive motors velocity
        mean = mean / numDriveMotors;
        float radicand = 0;
        for (int i = 0; i < numDriveMotors; i++) {
            radicand += square((encoders[i].getFilteredSpeed() - mean));
        }
        radicand = radicand / numDriveMotors;
        stdDeviation = sqrtf(radicand);
        
        lastUpdate = millis();
    }
}
