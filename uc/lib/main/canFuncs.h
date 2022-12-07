/**
 * Helper functions for managing CAN initializing, sending, and receiving. 
 */

#ifndef _CAN_FUNCS_H_
#define _CAN_FUNCS_H_

#include "mcp2515_can.h"
#include "mcp_can.h"

bool canInit();
void canSend(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *dat);
unsigned canReceive(uint32_t *id, uint8_t *dat);

#endif