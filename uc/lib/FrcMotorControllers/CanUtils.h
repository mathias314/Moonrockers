#ifndef __CAN_DEFINES_H__
#define __CAN_DEFINES_H__
#include <Arduino.h>

typedef enum {
    deviceBroadcast=0,
    robotController,
    motorController,
    relayController,
    gyroSensor,
    accelerometerSensor,
    ultrasonicSensor,
    gearToothSensor,
    powerDistribution,
    pneumaticsController,
    miscCANDevice,
    IOBreakout,
    dev_rsvd12,dev_rsvd13,dev_rsvd14,dev_rsvd15,
    dev_rsvd16,dev_rsvd17,dev_rsvd18,dev_rsvd19,
    dev_rsvd20,dev_rsvd21,dev_rsvd22,dev_rsvd23,
    dev_rsvd24,dev_rsvd25,dev_rsvd26,dev_rsvd27,
    dev_rsvd28,dev_rsvd29,dev_rsvd30,
    firmwareUpdate=31
} CanDeviceType;

// CAN Device types
typedef enum {
    manufacturerBroadcast=0,
    NI=1,
    LM=2, //(TI)
    DEKA=3,
    CTRE=4,
    REV=5,
    Grapple=6,
    MindSensors=7,
    TeamUse=8
} CanDeviceManufacturer;

// Define some bits for the CAN communications
const uint8_t RTR_BIT = 0;
const uint8_t EXT_BIT = 1;

uint32_t makeCanId(CanDeviceManufacturer manufacturer, CanDeviceType device, uint16_t msgId, uint16_t deviceId);
uint8_t* float2buff(float val, uint8_t* arr);
uint8_t* int2buff(uint16_t val, uint8_t* arr);
void printCanMessage(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *buf);
void clearBuffer(uint8_t* buff, unsigned len=8);

#endif
