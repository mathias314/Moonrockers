//////////////////////////////////////////////////////////////////////////////
//Description: Example code for running the Victor SPX or Talon SRX using an
//             Arduino and a MCP2515 CAN Bus Interface module. This code uses
//             the CAN_BUS_SHIELD library found here:
//             https://github.com/Seeed-Studio/CAN_BUS_Shield
//             Talon/Victor module must be flashed with the non-FRC version
//             of the firmware (...PhoenixNonFRCUpdate.crf) using the HERO
//             Development Board. Also Phoenix 5.8 should be used. This can
//             be installed from here:
//             http://www.ctr-electronics.com/installer-archive
/////////////////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <SPI.h>

//! Use this to select the type of CAN module being used!
//#define SPI_CAN

#ifdef   SPI_CAN//============SPI CAN Module============
#include <mcp_can.h>
#include "mcp_can.h"
#include "can-serial.h"
#include "mcp2515_can.h"

const int spiCSPin = 9;
mcp2515_can CAN(spiCSPin); // Set CS pin

bool canInit() {
    return CAN_OK == CAN.begin(CAN_1000KBPS/*, MCP_8MHz*/);
}

void canSend(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *dat) {
    //sendMsgBuf(unsigned long id, byte ext, byte len, const byte *buf)
    CAN.sendMsgBuf(id, ext, len, dat);
}

#else  //============Serial CAN module================
#include <Serial_CAN_Module.h>

Serial_CAN can;

bool canInit() {
    can.begin(Serial2, 9600);
    can.baudRate(SERIAL_RATE_9600);
    return can.canRate(CAN_RATE_1000);
}
void canSend(unsigned long id, uchar ext, uchar rtrBit, uchar len, const uchar *dat) {
    can.send(id, ext, rtrBit, len, dat);
}
#endif



//set the device number
const uint32_t deviceNumber = 0;  //must be  0 - 62

//frame which is used for percent output.
const uint32_t CONTROL = 0x040080;

//ID used for sending the enable frame
const uint32_t ENABLE = 0x0401BF;

// Define some bits for the CAN communications
const uint8_t RTR_BIT = 0;
const uint8_t EXT_BIT = 1;

//value to send to the moters (-+1023)
int32_t controlVal = 400;  ///<<-------CHANGE THIS TO CONTROL!!!!

//encode output into bytes
byte first_byte = (byte)(controlVal >> 0x10);
byte second_byte = (byte)(controlVal >> 0x08);
byte third_byte = (byte)(controlVal);

//build the control CANbus frame.  The first three bytes is the demand value,
//which typically is the output value [-1023,+1023]

//data for the CONTROL frame
unsigned char data[8] = {first_byte, second_byte, third_byte, 0, 0, 0, 0, 0};

//data for the ENABLE frame
unsigned char enable[8] = {1, 0, 0, 0, 0, 0, 0, 0};


/*void printReceive() {
    unsigned long id = 0;
    unsigned char dta[8];

    if (can.recv(&id, dta)) {
        Serial.print("GET DATA FROM ID: ");
        Serial.println(id);
        for (int i = 0; i < 8; i++) {
            Serial.print("0x");
            Serial.print(dta[i], HEX);
            Serial.print('\t');
        }
        Serial.println();
    }
}*/


void printCanMessage(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *buf)
{
    Serial.print("ID:");
    Serial.print(id, HEX);
    Serial.print(",ext:");
    Serial.print(ext, HEX);
    Serial.print(",rtr:");
    Serial.print(rtrBit, HEX);
    Serial.print(",dat:{");
    for (int i = 0; i < 8; i++) {
        if (i < len) {
            Serial.print(data[i]);
        } else {
            Serial.print(0);
        }
        if (i != 7) Serial.print(",");
    }
    Serial.println("}");
}

//=====setup================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial);  // wait for Serial

    Serial.println("initializing...");

    //set the baudrate and let it know we are using the 8MHz oscilator on the CAN module
    if (!canInit()) {
        Serial.println("CAN initialization failed.");
        while (true) {
            delay(1000);
        }
    }
    Serial.println("CAN BUS Shield Init OK!");
}


//=====loop================================================================
void loop() {
    static bool running = false;
    char readVal = 0;

    //sendMsgBuf(unsigned long id, byte ext, byte len, const byte *buf, bool wait_sent = true)
    //id - ID of the device.
    //ext - the status of the frame. '0' means standard frame (11 bit ID). '1' means extended frame (29 bit ID).
    //len - the length of this frame in bytes.
    //buf - the content of this message.

    //send the CONTROL frame
    if (running) {
        //send(unsigned long id, uchar ext, uchar rtrBit, uchar len, const uchar *buf)
        canSend(CONTROL | deviceNumber | 0x02040000, EXT_BIT, RTR_BIT, 8, data);  //0x01040000 for Victor, 0x02040000 for Talon
        //printCanMessage(CONTROL | deviceNumber | 0x01040000, 1, 1, 8, data);
    }
    //send the ENABLE frame. Must be sent at least once every 100ms otherwise the motor controller goes sleepy-sleep.
    canSend(ENABLE, EXT_BIT, RTR_BIT, 8, enable);

    delay(80);

    //wait for a value, read it, and clear the extra
    while (Serial.available()) {
        readVal = Serial.read();
    }

    if (readVal == ' ') {
        running = !running;
        Serial.println(running ? "run" : "stop");
    }

    //printReceive();
}
