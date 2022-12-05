//////////////////////////////////////////////////////////////////////////////
//Description: Example code for running the SPARK MAX using an
//             Arduino and a MCP2515 CAN Bus Interface module. This code uses
//             the CAN_BUS_SHIELD library found here:
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

#include <mcp2515_can.h>

#define FREQUENCY 4
#define AMPLITUDE 0.5
#define OFFSET 0.01
#define MOVE_POWER 0.2

const int spiCSPin = 9;
mcp2515_can CAN(spiCSPin);  // Set CS pin

bool canInit() {
    const byte clockset = MCP_16MHz;  //! Make sure oscilator set to correct value!!!
    const uint32_t speedset = CAN_1000KBPS;

    // Initialize
    byte result = CAN.begin(speedset, clockset);

    // Retry if needed
    unsigned tryCount = 1;
    while (tryCount < 10 && result != CAN_OK) {
        delay(250);
        result = CAN.begin(speedset, clockset);
        tryCount++;
    }

    return result == CAN_OK;
}

void canSend(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *dat) {
    //sendMsgBuf(unsigned long id, byte ext, byte len, const byte *buf)
    CAN.sendMsgBuf(id, ext, len, dat);
}

#else  //============Serial CAN module================
#include <Serial_CAN_Module.h>

#include "can-serial.h"

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

SparkMax deliveryMotor(1);

const float PERIOD_CONV = FREQUENCY * 2 * PI / 1000.0;

//=====setup================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) {
    }  // wait for Serial

    Serial.println("initializing...");
    pinMode(spiCSPin, OUTPUT);
    digitalWrite(spiCSPin, HIGH);

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
    deliveryMotor.clearFaults();
}

//=====loop================================================================
void loop() {
    static char mode = ' ';  // Motor run mode
    char readVal = 0;

    // Update at a fixe interval (<100ms I think)
    delay(80);
    FrcMotorController::sendKeepAlive();
    static unsigned long startTime = millis();

    // Check if any serial data available
    while (Serial.available()) {
        readVal = Serial.read();
    }

    // Control the motor
    switch (readVal) {
    case ' ':
        Serial.println("stop");
        deliveryMotor.setPower(0.0);
        mode = readVal;
        break;
    case 'u':
        Serial.println("up");
        deliveryMotor.setPower(MOVE_POWER + OFFSET);
        mode = readVal;
        break;
    case 'd':
        Serial.println("down");
        deliveryMotor.setPower(-MOVE_POWER);
        mode = readVal;
        break;
    case 'v':
        Serial.println("vibrate");
        startTime = millis();
        mode = readVal;
        break;
    default:
        break;
    }

    // Do an update at set intervals for vibrate mode
    static unsigned long lastUpdate = millis();
    if (mode == 'v' && lastUpdate - millis() > 10) {
        deliveryMotor.setPower(OFFSET + AMPLITUDE * sin((millis() - startTime) * PERIOD_CONV));

        lastUpdate = millis();
    }
}
