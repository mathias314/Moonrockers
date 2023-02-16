//////////////////////////////////////////////////////////////////////////////
//Description: Example code for running the SPARK MAX using an
//             Arduino and a MCP2515 CAN Bus Interface module. This code uses
//             the CAN_BUS_SHIELD library found here:
//             https://github.com/Seeed-Studio/CAN_BUS_Shield
//             The SPARK MAX must have the most recent firmware and be set to
//             some non-zero ID number.
//
// Motor is controlled by the following commands over serial:
//    - ' ' = STOP
//    - 'r X.YY' = Set percent output to value in [-1.0, 1.0].
//    - 'v X.YY' = Set velocity to X.yy.
//    - 'p X.YY' = Run motor to X.YY rotations.
//
// PID must be tuned for this to work!!! Velocity control must be in slot #1, 
// and position control must be in slot #2.
/////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <SPI.h>
#include "SparkMax.h"
#include "globals.h"

//! Use this to select the type of CAN module being used!
#define SPI_CAN

#ifdef SPI_CAN  //============SPI CAN Module============
#include <mcp_can.h>

#include "can-serial.h"
#include "mcp2515_can.h"
#include "mcp_can.h"

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

SparkMax motor(1);

//Servo collectionChainDrive;


//=====setup================================================================
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(5000);
    while (!Serial)
        ;  // wait for Serial

    Serial.println("initializing...");
    Serial.flush();

    //set the baudrate and let it know we are using the 8MHz oscilator on the CAN module
    if (!canInit()) {
        Serial.println("CAN initialization failed.");
        while (true) {
            delay(1000);
        }
    }
    Serial.println("CAN BUS Shield Init OK!");

    // Add the sender and receiver for the CAN communications.
    FrcMotorController::addCanSender(canSend);
    Serial.println("Adding CAN receiver...");
    FrcMotorController::addCanReceiver(CAN_INT_PIN, canReceive);

    // Clear faults    
    Serial.println("Clearing faults...");
    motor.clearFaults();

    //Serial.println("LED pin...");
    //pinMode(STAT_LED, OUTPUT);
    //digitalWrite(STAT_LED, LOW);
    Serial.println("Starting main loop.");
}

//=====loop================================================================
void loop() {
    static unsigned long lastPrint = 0;
    char readVal = 0;
    // Update at a fixe interval (<100ms I think)
    delay(80);
    FrcMotorController::sendKeepAlive();
    
    // Check if any serial data available
    while (Serial.available()) {
        readVal = Serial.read();
    }

    // Control the selected motor (If given one)
    float val = 0.0;
    switch (readVal) {
    case ' ': // STOP
        motor.setPower(0.0);
        Serial.println("[stop]");
        break;
    case 'r': // Percent output control
        val = Serial.parseFloat(SKIP_WHITESPACE);
        motor.setPower(val);
        Serial.print("[percent]:");
        Serial.println(val);
        break;
    case 'v':  // Velocity control
        val = Serial.parseFloat(SKIP_WHITESPACE);
        motor.setPidSlot(0);
        motor.setVelocity(val);
        Serial.println("[velocity]:");
        Serial.println(val);
        break;
    case 'p':  // Position control
        val = Serial.parseFloat(SKIP_WHITESPACE);
        motor.setPidSlot(1);
        motor.setPosition(val);
        Serial.println("[position]:");
        Serial.println(val);
        break;
    default:
        break;
    }

    // Output status information
    if (millis() - lastPrint > 200) {
        Serial.print("T:");
        Serial.print(motor.getTemperature());
        Serial.print(", %:");
        Serial.print(motor.getAppliedOutput());
        Serial.print(", A:");
        Serial.print(motor.getOutputCurrent());
        Serial.print(", V:");
        Serial.print(motor.getBusVoltage());
        Serial.print(", P:");
        Serial.print(motor.getPosition());
        Serial.print(", v:");
        Serial.println(motor.getVelocity());
        lastPrint = millis();
    }
}
