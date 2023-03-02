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


PwmMotor motors[10] = {
    // Drive motors
    {/*pwmPin*/ 10, /*dirPin*/ 8},    // Drive back left
    {/*pwmPin*/ 3, /*dirPin*/ 33},    // Drive back right
    {/*pwmPin*/ 9, /*dirPin*/ 25},    // Drive middle left
    {/*pwmPin*/ 2, /*dirPin*/ 29},    // Drive middle right
    {/*pwmPin*/ 7, /*dirPin*/ 49},    // Drive front left
    {/*pwmPin*/ 6, /*dirPin*/ 45},    // Drive front right
    
    // Steer motors
    {/*pwmPin*/ 11, /*dirPin*/ 11},    // Steer back left
    {/*pwmPin*/ 4, /*dirPin*/ 37},    // Steer back right
    {/*pwmPin*/ 8, /*dirPin*/ 51},   // Steer front left
    {/*pwmPin*/ 5, /*dirPin*/ 41}    // Steer front right
};

Encoder encoders[10] = {
    // Drive encoders
    {/*ISR Pin*/ 9, /*PPRs*/ 324},     // Drive back left
    {/*ISR Pin*/ 32, /*PPRs*/ 324},     // Drive back right
    {/*ISR Pin*/ 24, /*PPRs*/ 324},    // Drive middle left
    {/*ISR Pin*/ 28, /*PPRs*/ 324},    // Drive middle right
    {/*ISR Pin*/ 48, /*PPRs*/ 324},    // Drive front left
    {/*ISR Pin*/ 44, /*PPRs*/ 324},    // Drive front right
    
    // Steer encoders
    {/*ISR Pin*/ 10, /*PPR*/ 504},     // Steer back left
    {/*ISR Pin*/ 36, /*PPR*/ 504},     // Steer back right
    {/*ISR Pin*/ 52, /*PPR*/ 504},     // Steer front left
    {/*ISR Pin*/ 40, /*PPR*/ 504}      // Steer front right
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

//=====setup==============================
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(2500);

    // Initialize motors and encoders
    Serial.println("Initializing motors...");
    for (int i = 0; i < numMotors; i++) {
        motors[i].init();
        /*
        if (!encoders[i].init()) {
            Serial.println("***Encoder Initialization Failure***");
            while(1);
        }
        */
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
        
        Serial.print("Target:");
        Serial.print(motors[0].getTarget());
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

        lastUpdate = millis();
    }
}
