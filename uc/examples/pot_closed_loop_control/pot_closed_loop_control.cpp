/**
 * For testing and tuning of closed loop control using a potentiometer for position control. 
 * 
 * To Run:
 *      pio run -t upload -c examples.ini -e pot_closed_loop_control
*/

#include <Arduino.h>
#include <SPI.h>
#include "TalonSrx.h"
#include "globals.h"
#include "PID.h"
#include "Potentiometer.h"

//! Use this to select the type of CAN module being used!
#define SPI_CAN

#ifdef SPI_CAN  //============SPI CAN Module============
#include <mcp_can.h>

#include "can-serial.h"
#include "mcp2515_can.h"
#include "mcp_can.h"

#define UPDATE_INTERVAL 10

#define STAT_LED 6

mcp2515_can CAN(CAN_CS_PIN);  // Set CS pin

bool canInit() {
    return CAN.begin(CAN_1000KBPS, MCP_16MHz) == CAN_OK;   //! Make sure oscilator set to correct value!!!
}

void canSend(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *dat) {
    //sendMsgBuf(unsigned long id, byte ext, byte len, const byte *buf)
    CAN.sendMsgBuf(id, ext, len, dat);
}

unsigned canReceive(uint32_t *id, uint8_t *dat) {
    byte len = 0;
    CAN.readMsgBuf(&len, dat);
    *id = CAN.getCanId();
    return len;
}

#else  //============Serial CAN module================
#include <Serial_CAN_Module.h>

Serial_CAN can;

bool canInit() {
    can.begin(Serial1, 9600);
    can.baudRate(SERIAL_RATE_9600);
    return can.canRate(CAN_RATE_1000);
}
void canSend(unsigned long id, uchar ext, uchar rtrBit, uchar len, const uchar *dat) {
    can.send(id, ext, rtrBit, len, dat);
}
#endif

TalonSrx motor(FL_PIVOT_MTR_ID);

Potentiometer angleSensor(FL_PIVOT_SENSOR_PIN, ANGLE_SENSOR_CALIBRATIONS[0]);

// float kp = 1.0, ki = 5.0, kd = 0.03, N = 10;
float kp = PIV_KP, ki = PIV_KI, kd = PIV_KD, N = PIV_N;
// PID(Kp, Ki, Kd, N, sample_time)
PID pid(kp, ki, kd, N, UPDATE_INTERVAL / 1000.0);

void updateCoefficients()
{
    pid.setConstants(kp, ki, kd, N, UPDATE_INTERVAL / 1000.0);

    Serial.print("p: ");
    Serial.print(kp);
    Serial.print(", i: ");
    Serial.print(ki);
    Serial.print(", d: ");
    Serial.print(kd);
    Serial.print(", N: ");
    Serial.println(N);
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
    FrcMotorController::addCanSender(canSend);
    //Serial.println("Adding CAN receiver...");
    //FrcMotorController::addCanReceiver(CAN_INT_PIN, canReceive);

    // Clear faults    
    Serial.println("Clearing faults...");
    //motor.clearFaults();

    Serial.println("Angle sensor init...");
    angleSensor.init();

    // Set PID tuning values
    pid.setLimits(-0.4, 0.4);
    pid.setTargetLimits(-PI/2, PI/2);
    pid.setTargetRampRate(5);
    pid.setTarget(0, false);

    // Serial.println("LED pin...");
    // pinMode(STAT_LED, OUTPUT);
    // digitalWrite(STAT_LED, LOW);
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
            Serial.println("[STOP]");
            break;

        case 's':
            pid.setTarget(0);
            running = true;
            Serial.println("[STRAIGHT]");
            break;

        case 'l':
            pid.setTarget(PI/2);
            running = true;
            Serial.println("[LEFT]");
            break;

        case 'r':
            pid.setTarget(-PI/2);
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
            N = Serial.parseFloat();
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

    // Update at a fixe interval (<100ms I think)
    static unsigned long lastKeepAlive = millis();
    if (millis() - lastKeepAlive > 40) {
        FrcMotorController::sendKeepAlive();
        lastInterval = millis() - lastKeepAlive;
        lastKeepAlive = millis();
    }
    
    // Do the control loop update
    static unsigned long lastControlUpdate = millis();
    if (millis() - lastControlUpdate > UPDATE_INTERVAL) {
        angleSensor.update();

        if (running)
        {
            powerVal = pid.calculateOutput(angleSensor.getFilteredAngle());
            motor.setPower(powerVal);
        }
        else
        {
            motor.setPower(0);
        }

        lastControlUpdate = millis();
    }

    // Output status information
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
        // Serial.print("raw:");
        // Serial.print(powerVal);
        // Serial.print(", angle:");
        // Serial.print(", ");
        Serial.print(angleSensor.getRaw());
        // Serial.print(", angle:");
        Serial.print(", ");
        Serial.print(angleSensor.getAngle()*RAD_TO_DEG);
        // Serial.print(", filtered:");
        Serial.print(", ");
        Serial.println(angleSensor.getFilteredAngle()*RAD_TO_DEG);
        
        lastPrint = millis();
    }
}
