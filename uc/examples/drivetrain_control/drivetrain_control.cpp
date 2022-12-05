//////////////////////////////////////////////////////////////////////////////
//Description: Example code for running/testing the four motors on the bot.
//             This code uses the CAN_BUS_SHIELD library found here:
//             https://github.com/Seeed-Studio/CAN_BUS_Shield
//             The SPARK MAX must have the most recent firmware and be set to
//             some non-zero ID number. 
/////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <SPI.h>
#include "SparkMax.h"

//! Use this to select the type of CAN module being used!
#define SPI_CAN

#ifdef SPI_CAN  //============SPI CAN Module============
#include <mcp_can.h>

#include "can-serial.h"
#include "mcp2515_can.h"
#include "mcp_can.h"

const int spiCSPin = 9;
mcp2515_can CAN(spiCSPin);  // Set CS pin

bool canInit() {
    return CAN_OK == CAN.begin(CAN_1000KBPS, MCP_16MHz);   //! Make sure oscilator set to correct value!!!
}

void canSend(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *dat) {
    //sendMsgBuf(unsigned long id, byte ext, byte len, const byte *buf)
    CAN.sendMsgBuf(id, ext, len, dat);
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

SparkMc mtrFrontLeft(1);
SparkMc mtrFrontRight(2);
SparkMc mtrBackRight(3);
SparkMc mtrBackLeft(4);

SparkMc* motors[] = {&mtrFrontLeft, &mtrFrontRight, &mtrBackRight, &mtrBackLeft};

//=====setup================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial)
        ;  // wait for Serial

    Serial.println("initializing...");

    //set the baudrate and let it know we are using the 8MHz oscilator on the CAN module
    if (!canInit()) {
        Serial.println("CAN initialization failed.");
        while (true) {
            delay(1000);
        }
    }
    Serial.println("CAN BUS Shield Init OK!");

    // Add the sender for the CAN communications.
    FrcMotorController::addCanSender(canSend);

    // Clear faults    
    motors[0]->clearFaults();
    motors[1]->clearFaults();
    motors[2]->clearFaults();
    motors[3]->clearFaults();
}

//=====loop================================================================
void loop() {
    static bool running[4] = {0};  // Motor states
    char readVal = 0;
    static unsigned long lastPeriod = 0;
    static unsigned long lastSend = millis();

    // Update at a fixe interval (<100ms I think)
    if (millis() - lastSend >= 80) {
        lastPeriod = millis() - lastSend;
        FrcMotorController::sendKeepAlive();
        lastSend = millis();
    }

    // Check if any serial data available
    while (Serial.available()) {
        readVal = Serial.read();
    }

    // Control the selected motor (If given one)
    int mtrNum = readVal - '1';
    if (mtrNum >= 0 && mtrNum < 4) {
        // Toggle
        running[mtrNum] = !running[mtrNum];

        // Update the motor
        motors[mtrNum]->setPower(running[mtrNum] ? 0.2 : 0.0);

        // Print new state
        Serial.print(mtrNum+1);
        Serial.println(running[mtrNum] ? ": run" : ": stop");
    }

    static unsigned long lastPrint = millis();
    if (millis() - lastPrint > 1000) {
        Serial.println(lastPeriod);
        lastPrint = millis();
    }
}
