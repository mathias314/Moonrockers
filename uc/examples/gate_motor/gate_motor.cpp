//////////////////////////////////////////////////////////////////////////////
// Description: Example code for controlling the small gate motors with the
//  SPARK Mini Motor controller.
//
//  Motor is controlled by the following commands over serial:
//     - ' ' = STOP
//     - '+' = Increase power by 10%.
//     - '-' = Decrease power by 10%.
//     - 'i' = Invert motor direction.
/////////////////////////////////////////////////////////////////////////////
#include <Arduino.h>

#include "SparkMini.h"

SparkMini motor(9);

char readVal = '\0';
float speed = 0;
bool inverted = false;

//=====setup==============================
void setup() {
    Serial.begin(115200);

    motor.init();
}

//=====loop==============================
void loop() {
    switch (readVal) {
        case ' ':
            speed = 0;
            break;
        case '+':
        case '=':
            speed += 0.1;
            break;
        case '-':
        case '_':
            speed -= 0.1;
            break;
        case 'i':
            inverted = !inverted;
            motor.setInverted(inverted);
            Serial.print("Inverted: ");
            Serial.println(inverted);
            break;
        default:
            break;
    }

    motor.setPower(speed);
    Serial.println(speed);

    // wait for a value, read it, and clear the extra
    while (Serial.available() <= 0) {
        delay(100);
    }
    readVal = Serial.read();
    while (Serial.available()) {
        Serial.read();
    }
}
