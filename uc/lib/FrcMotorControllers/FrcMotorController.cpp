
#include "FrcMotorController.h"

// Set some static class variables
unsigned FrcMotorController::instanceCount = 0;
FrcMotorController** FrcMotorController::devices = nullptr;
volatile int FrcMotorController::interruptPin = -1;
void (*FrcMotorController::canSender)(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *dat) = nullptr;
unsigned (*FrcMotorController::canReceiver)(uint32_t *id, uint8_t *dat) = nullptr;

unsigned FrcMotorController::keepAliveCount = 0;
FrcMotorController::fptr* FrcMotorController::keepAliveHandlers = nullptr;

/**
 * @brief Constructor for the class. Add an instance for the global CAN communications.
 * 
 * @param deviceId - ID for this device on the CAN network.
 */
FrcMotorController::FrcMotorController(uint8_t deviceId) {
    this->deviceId = deviceId;
    
    // Add another instance for ISR stuff
    instanceCount++;
    devices = (FrcMotorController**)realloc(devices, instanceCount * sizeof(FrcMotorController*));
    devices[instanceCount-1] = this;
}

void FrcMotorController::addKeepAliveCallback(fptr callback) {
    // Add the keep alive callback (if we don't have it yet)
    if (callback != nullptr)
    {
        bool needAdd = true;
        unsigned idx = 0;
        while (needAdd && idx < keepAliveCount)
        {
            if (keepAliveHandlers[idx] == callback) {
                needAdd = false;
            }
            idx++;
        }
        if (needAdd)
        {
            keepAliveCount++;
            keepAliveHandlers = (fptr*)realloc(keepAliveHandlers, keepAliveCount * sizeof(fptr));
            keepAliveHandlers[keepAliveCount-1] = callback;
        }
    }
}

/**
 * Add the function which will be used for sending CAN commands. This 
 * function will be used by all objects created using this class. 
 * The function must take in:  
 *  - id - The ID of the CAN message
 *  - ext - Whether the message is extended or regular (0 or 1).
 *  - rtrBit - The value for this bit in the message. 
 *  - len - Then number of bytes in the message. 
 *  - dat - Pointer to the data array.
 * 
 * @param canSender - The function to call for sending commands.
 */
void FrcMotorController::addCanSender(void (*canSender)(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *dat)) {
    FrcMotorController::canSender = canSender;
}

/**
 * Add a receiver function and set the interrupt pin. 
 * The receive function will be called repeatedly until the interrupt pin goes HIGH.
 * 
 * For the receiver function, id = the ID of the CAN message, dat = an array of data. The function should return the number of 
 * bytes received and placed into the data array.
 * 
 * @param pinNum - Pin number for the interrupt.
 * @param canReceiver - Function to call to receive CAN data.
 */
void FrcMotorController::addCanReceiver(unsigned pinNum, unsigned (*canReceiver)(uint32_t *id, uint8_t *dat)) {
    FrcMotorController::interruptPin = pinNum;
    FrcMotorController::canReceiver = canReceiver;
    pinMode(pinNum, INPUT);
    attachInterrupt(digitalPinToInterrupt(pinNum), FrcMotorController::receiveAllIncoming, FALLING); 
}

/**
 * @brief Send some data over the CAN network.
 * 
 * @param id - ID for the message.
 * @param len - Length of the message.
 * @param dat - Message data (array of bytes).
 */
void FrcMotorController::canSend(unsigned long id, uint8_t len, const uint8_t *dat) {
    // Ignore any incoming messages during the send
    if (FrcMotorController::interruptPin != -1) {
        detachInterrupt(digitalPinToInterrupt(FrcMotorController::interruptPin));
    }
    
    // Do the send using the provided sender
    canSender(id, EXT_BIT, RTR_BIT, len, dat);
    
    // Receive anything that happened during that time and re-enable interrupts
    if (FrcMotorController::interruptPin != -1) {
        FrcMotorController::receiveAllIncoming();
        attachInterrupt(digitalPinToInterrupt(FrcMotorController::interruptPin), FrcMotorController::receiveAllIncoming, FALLING); 
    }
}

/**
 * Receive all incoming CAN messages.
 */
void FrcMotorController::receiveAllIncoming() {
    while (digitalRead(FrcMotorController::interruptPin) == LOW) {
        FrcMotorController::receiveIncoming();
    }
}

/**
 * Receive data incoming to the CAN module. This uses the receiver function set using 
 * addCanReceiver(). Any received data is sent to the appropriate MC instance.
 */
bool FrcMotorController::receiveIncoming() {
    // Read the incoming message
    uint32_t id = 0;
    static byte dat[10] = {0};
    unsigned len = FrcMotorController::canReceiver(&id, dat);
    
    // Get the ID for the device
    uint8_t deviceId = id & 0b11111;
    unsigned devIdx = 0;
    while (devIdx < instanceCount && devices[devIdx]->deviceId != deviceId) {
        devIdx++;
    }
    // Check if failed to find device
    if (devIdx == instanceCount) {
        return false;
    }

    uint16_t ourTypeCode = devices[devIdx]->deviceManufacturer | (motorController << 8);
    // Check to make sure the manufacturer and device type are good
    uint16_t typeCode = id >> 16;
    if (typeCode != ourTypeCode) {
        return false;
    }

    // Check to make sure length is good
    if (len != 8) {
        return false;
    }

    // Receive the message
    unsigned msgType = (id >> 6) & 0x3FF;

    // Handle the data on the device
    return devices[devIdx]->handleIncoming(msgType, dat, len);
}

/**
 * Send a single-value CAN message to this motor controller.
 * 
 * @param msgId - id for this message type.
 * @param val - Value to send
 */
void FrcMotorController::sendVal(uint16_t msgId, float val) {
    uint32_t frameId = makeCanId(deviceManufacturer, motorController, msgId, deviceId);
    FrcMotorController::canSend(frameId, 8, float2buff(val, sendBuffer));
}

/**
 * Send a CAN message to this motor controller.
 * 
 * @param msgId - id for this message type.
 * @param buffer - Buffer with values to send.
 */
void FrcMotorController::sendVal(uint16_t msgId, uint8_t* buffer) {
    uint32_t frameId = makeCanId(deviceManufacturer, motorController, msgId, deviceId);
    FrcMotorController::canSend(frameId, 8, buffer);
}

/**
 * Send the keep-alive message to all motor controllers registered with this class. Must be sent every ~100ms.
 * Only needs to be sent once for all connected SPARK MCs. 
 */
void FrcMotorController::sendKeepAlive() {
    // Call all registered handlers
    for (unsigned i = 0; i < keepAliveCount; i++)
    {
        keepAliveHandlers[i]();
    }
}