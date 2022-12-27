/**
 * PWM so we don't have to deal with timer interrupts. 
 * Motor is controlled by the following commands over serial:
 *     - ' ' = STOP  
 *     - '+' = Increase power by 10%.
 *     - '-' = Decrease power by 10%.
 *     - 'i' = Invert motor direction.
 *
 * To Run:
 *      pio run -t upload -c examples.ini -e pwm_motor
 * */
#include <Arduino.h>
#include "PwmMotor.h"
#include "globals.h"
#include "Encoder.h"


char readVal = '\0';
float speed = 0;
bool inverted = false;

PwmMotor motor(4, 5);

Encoder encoder(19, 324);  // 323

//=====setup==============================
void setup() {
    Serial.begin(115200);

    Serial.println("Initializing motor...");
    motor.init();
    

    // Initialize the encoder 
    Serial.println("Initializing encoder...");
    encoder.init();
}

//=====loop==============================
void loop() {

    if (Serial.available()) {
        readVal = Serial.read();
        while (Serial.available()) {
            Serial.read();
        }
        switch (readVal) {
            case ' ':
                motor.stop();
                Serial.println("STOP");
                break;
            case '+':
            case '=':
                motor.run(motor.getPower() + 0.1);
                Serial.println(motor.getPower());
                break;
            case '-':
            case '_':
                motor.run(motor.getPower() - 0.1);
                Serial.println(motor.getPower());
                break;
            case 'u':
                motor.run(motor.getPower() + 0.1);
                Serial.println(motor.getPower());
                break;
            case 'd':
                motor.run(motor.getPower() - 0.1);
                Serial.println(motor.getPower());
                break;
            case 'i':
                inverted = !inverted;
                motor.setInverted(inverted);
                Serial.print("Inverted: ");
                Serial.println(inverted);
                break;
            case 'c':
                encoder.absTot = 0;
                break;
            default:
                break;
        }
    }

    static unsigned long lastDisplay = millis();
    if (millis() - lastDisplay > 100) {
        Serial.print(encoder.getSpeed());
        Serial.print(",");
        Serial.println(encoder.estimateSpeed());
        lastDisplay = millis();
    } 
}
