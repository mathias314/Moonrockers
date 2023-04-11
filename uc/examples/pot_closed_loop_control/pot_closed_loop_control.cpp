/**
 * For testing and tuning of closed loop control using a potentiometer for position control. 
 * 
 * To Run:
 *      pio run -t upload -c examples.ini -e pot_closed_loop_control
*/

#include <Arduino.h>
#include "globals.h"
#include "canFuncs.h"
#include "MoonBot.h"

MoonBot robot(UPDATE_INTERVAL / 1000.0);

MoonBot::Joint mtrToControl = MoonBot::BACK_LEFT;

float kp = PIV_VEL_KP, ki = PIV_VEL_KI, kd = PIV_VEL_KD, n = PIV_VEL_N; 

void updateCoefficients()
{
    robot.setPivotPIDConstants(mtrToControl, kp, ki, kd, n);

    Serial.print("p: ");
    Serial.print(kp);
    Serial.print(", i: ");
    Serial.print(ki);
    Serial.print(", d: ");
    Serial.print(kd);
    Serial.print(", N: ");
    Serial.println(n);
}

//=====setup================================================================
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(2500);
    delay(1000);
    while (!Serial)
        ;  // wait for Serial

    Serial.println("initializing...");
    Serial.flush();

    //set the baudrate and let it know we are using the 8MHz oscilator on the CAN module
    if (!canInit()) {
        Serial.println("CAN initialization failed.");
        Serial.print("Retrying");
        while (!canInit()) {
            delay(1000);
            Serial.print(".");
        }
    }
    Serial.println("CAN BUS Shield Init OK!");

    // Add the sender and receiver for the CAN communications.
    Serial.println("Initializing robot");
    robot.addCanSender(canSend);
    robot.init();
    //Serial.println("Adding CAN receiver...");
    //FrcMotorController::addCanReceiver(CAN_INT_PIN, canReceive);

    Serial.println("Starting main loop.");
}

//=====loop================================================================
void loop() {
    static bool running = false;
    static unsigned lastInterval = 0;
    static float powerVal = 0;

    if (Serial.available())
    {
        switch (Serial.read())
        {
        case ' ':
            running = false;
            robot.stop();
            Serial.println("[STOP]");
            break;

        case 's':
            robot.setPivotAngle(mtrToControl, 0);
            running = true;
            Serial.println("[STRAIGHT]");
            break;

        case 'l':
            robot.setPivotAngle(mtrToControl, PI/4);
            running = true;
            Serial.println("[LEFT]");
            break;

        case 'r':
            robot.setPivotAngle(mtrToControl, -PI/4);
            running = true;
            Serial.println("[RIGHT]");
            break;

        case 'p':
            kp = Serial.parseFloat();
            updateCoefficients();
            break;

        case 'i':
            ki = Serial.parseFloat();
            updateCoefficients();
            break;

        case 'd':
            kd = Serial.parseFloat();
            updateCoefficients();
            break;

        case 'n':
            n = Serial.parseFloat();
            updateCoefficients();
            break;
        
        default:
            break;
        }
        while (Serial.available())
        {
            Serial.read();
        }
    }

    // Do the update
    robot.update();

    // Output status information
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 50) {
        // Serial.print("raw:");
        // Serial.print(powerVal);
        // Serial.print(", angle:");
        // Serial.print(", ");
        Serial.print(robot.angleSensors[mtrToControl].getRawVelocity());
        Serial.print(", ");
        Serial.print(robot.angleSensors[mtrToControl].getVelocity());
        Serial.print(", ");
        Serial.print(robot.angleSensors[mtrToControl].getRawAngle()*RAD_TO_DEG);
        Serial.print(", ");
        Serial.println(robot.angleSensors[mtrToControl].getAngle()*RAD_TO_DEG);
        // Serial.print(", filtered:");
        
        lastPrint = millis();
    }
}
