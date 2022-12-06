#include "canFuncs.h"
#include "globals.h"

mcp2515_can CAN(CAN_CS_PIN);  // Set CS pin

bool canInit() {
    return CAN.begin(CAN_1000KBPS, MCP_16MHz) == CAN_OK;  //! Make sure oscilator set to correct value!!!
}

void canSend(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *dat) {
    // sendMsgBuf(unsigned long id, byte ext, byte len, const byte *buf)
    CAN.sendMsgBuf(id, ext, len, dat);
}

unsigned canReceive(uint32_t *id, uint8_t *dat) {
    byte len = 0;
    CAN.readMsgBuf(&len, dat);
    *id = CAN.getCanId();
    return len;
}