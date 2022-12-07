//////////////////////////////////////////////////////////////////////////////
// Description: Example code for controlling a PWM motor. This uses hardware 
// PWM so we don't have to deal with timer interrupts. 
//
//  Motor is controlled by the following commands over serial:
//     - ' ' = STOP
//     - '+' = Increase power by 10%.
//     - '-' = Decrease power by 10%.
//     - 'i' = Invert motor direction.
/////////////////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include "PwmMotor.h"
#include "globals.h"


char readVal = '\0';
float speed = 0;
bool inverted = false;

PwmMotor motor1(GATE_PIN, 1);
PwmMotor motor2(CONVEYOR_PIN, 0);


//=====setup==============================
void setup() {
    Serial.begin(115200);

    Serial.println("Initializing motor...");
    motor1.init();
    motor2.init();
}

//=====loop==============================
void loop() {
    switch (readVal) {
        case ' ':
            motor1.setPower(0);
            motor2.setPower(0);
            Serial.println("STOP");
            break;
        case '+':
        case '=':
            motor1.setPower(motor1.getPower() + 0.1);
            Serial.println(motor1.getPower());
            break;
        case '-':
        case '_':
            motor1.setPower(motor1.getPower() - 0.1);
            Serial.println(motor1.getPower());
            break;
        case 'u':
            motor2.setPower(motor2.getPower() + 0.1);
            Serial.println(motor2.getPower());
            break;
        case 'd':
            motor2.setPower(motor2.getPower() - 0.1);
            Serial.println(motor2.getPower());
            break;
        case 'i':
            inverted = !inverted;
            // motor.setInverted(inverted);
            Serial.print("Inverted: ");
            Serial.println(inverted);
            break;
        default:
            break;
    }

    // wait for a value, read it, and clear the extra
    while (Serial.available() <= 0) {
        delay(100);
    }
    readVal = Serial.read();
    Serial.println(readVal);
    while (Serial.available()) {
        Serial.read();
    }
}
