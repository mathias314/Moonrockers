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
#include "PID.h"

#define isPos(x) ((x) > 0 ? true : false)

char readVal = '\0';
float speed = 0;
bool inverted = false;

PwmMotor motor(/*speedPin*/ BR_DRIVE_PWM_PIN, /*dirPin*/ BR_DRIVE_DIR_PIN);

Encoder encoder(BR_DRIVE_ENC_PIN, DRIVE_TACH_RATE);  // 323

//PID(float Kp, float Ki, float Kd, float N, float sample_time);
float kP = 0.01, kI = 0.02, kD = 0.0, n = 0.0, sampleTime = 0.005;
PID motorPid(kP, kI, kD, n, sampleTime);

void updateCoefficients()
{
    motorPid.setConstants(kP, kI, kD, n, sampleTime);

    Serial.print("p: ");
    Serial.print(kP);
    Serial.print(", i: ");
    Serial.print(kI);
    Serial.print(", d: ");
    Serial.print(kD);
    Serial.print(", N: ");
    Serial.println(n);
}

//=====setup==============================
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(2500);

    Serial.println("Initializing motor...");
    motor.init();
    

    // Initialize the encoder 
    Serial.println("Initializing encoder...");
    encoder.init();

    motorPid.setLimits(-1.0, 1.0);
    motorPid.setTargetLimits(-230, 230);
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
                motor.stop();
                Serial.println("STOP");
                running = false;
                break;
            case '+':
            case '=':
                // motor.run(motor.getPower() + 0.1);
                motorPid.setTarget(motorPid.getTarget() + 10);
                motor.setTarget(motor.getPower() + 0.1);
                // Serial.println(motorPid.getTarget());
                running = true;
                break;
            case '-':
            case '_':
                // motor.run(motor.getPower() - 0.1);
                motorPid.setTarget(motorPid.getTarget() - 10);
                motor.setTarget(motor.getPower() - 0.1);
                // Serial.println(motorPid.getTarget());
                running = true;
                break;

            case 'p':
                kP = Serial.parseFloat();
                updateCoefficients();
                break;

            case 'i':
                kI = Serial.parseFloat();
                updateCoefficients();
                break;

            case 'd':
                kD = Serial.parseFloat();
                updateCoefficients();
                break;

            case 'n':
                n = Serial.parseFloat();
                updateCoefficients();
                break;

            case 'f':
                motorPid.setTarget(230);
                motor.setTarget(1.0);
                running = true;
                break;

            case 'm':
                motorPid.setTarget(-230);
                motor.setTarget(-1.0);
                running = true;
                break;

            case 'z':
                motorPid.setTarget(0);
                motor.setTarget(0);
                running = true;
                break;

            default:
                break;
        }
    }

    static unsigned long lastDisplay = millis();
    if (millis() - lastDisplay > 50) {
        Serial.print("Speed:");
        Serial.print(encoder.getSpeed());
        Serial.print(",");
        Serial.print("Filtered:");
        Serial.print(encoder.getFilteredSpeed());
        Serial.print(",");
        Serial.print("Target:");
        Serial.println(motor.getTarget());
        lastDisplay = millis();
    } 

    static unsigned long lastUpdate = millis();
    if (millis() - lastUpdate > 1000 * sampleTime) {
        // Use the sign of the motor's power to inform direction
        encoder.estimateSpeed();

        if (running) {
            motor.runTarget();
        }

        lastUpdate = millis();
    } 
}
