#include "CanUtils.h"


uint32_t makeCanId(CanDeviceManufacturer manufacturer, CanDeviceType device, uint16_t msgId, uint16_t deviceId) {
    return uint32_t(device) << 24 | uint32_t(manufacturer) << 16 | uint16_t(msgId) << 6 | deviceId;
}

/**
 * @brief Copy a floating point value to a byte array. 
 * 
 * @param val - Value to copy
 * @param arr - byte array
 * @return the array which was passed in.
 */
uint8_t* float2buff(float val, uint8_t* arr) {
    uint8_t* valPtr = (uint8_t*)&val;

    clearBuffer(arr);
    memcpy(arr, valPtr, sizeof(float));

    return arr;
}

/**
 * @brief Copy a 16-bit int value to a byte array. 
 * 
 * @param val - Value to copy
 * @param arr - byte array
 * @return the array which was passed in.
 */
uint8_t* int2buff(uint16_t val, uint8_t* arr) {
    uint8_t* valPtr = (uint8_t*)&val;
    
    clearBuffer(arr);
    memcpy(arr, valPtr, sizeof(uint16_t));

    return arr;
}

void printCanMessage(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *buf) {
    Serial.print("ID:");
    Serial.print(id, HEX);
    Serial.print(",ext:");
    Serial.print(ext, HEX);
    Serial.print(",rtr:");
    Serial.print(rtrBit, HEX);
    Serial.print(",dat:{");
    for (int i = 0; i < 8; i++) {
        if (i < len) {
            Serial.print(buf[i], HEX);
        } else {
            Serial.print(0);
        }
        if (i != 7) Serial.print(",");
    }
    Serial.println("}");
}

/**
 * @brief Set a buffer to all zeros. 
 * 
 * @param buff - byte array. 
 * @param len - Length of the array.
 */
void clearBuffer(uint8_t* buff, unsigned len) {
    memset(buff, 0, len);
}
