
#include "SparkMc.h"


// Define some bits for the CAN communications
const uint8_t RTR_BIT = 0;
const uint8_t EXT_BIT = 1;

// Set some static class variables
uint16_t SparkMc::activeDevices = 0;
unsigned SparkMc::instanceCount = 0;
SparkMc** SparkMc::devices = nullptr;
volatile int SparkMc::interruptPin = -1;
void (*SparkMc::canSender)(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *dat) = nullptr;
unsigned (*SparkMc::canReceiver)(uint32_t *id, uint8_t *dat) = nullptr;

// Define the IDS for various faults
typedef enum {
    SPARK_FLT_BROUNOUT = 0,
    SPARK_FLT_OVERCURRENT = 1,
    SPARK_FLT_IWDT_RESET = 2,
    SPARK_FLT_MOTOR_FAULT = 3,
    SPARK_FLT_SENSOR_FAULT = 4,
    SPARK_FLT_STALL = 5,
    SPARK_FLT_EEPROM_CRC = 6,
    SPARK_FLT_CAN_TX = 7,
    SPARK_FLT_CAN_RX = 8,
    SPARK_FLT_HAS_RESET = 9,
    SPARK_FLT_DRV_FAULT = 10,
    SPARK_FLT_OTHER_FAULT = 11,
    SPARK_FLT_SOFT_LIMIT_FWD = 12,
    SPARK_FLT_SOFT_LIMIT_REV = 13,
    SPARK_FLT_HARD_LIMIT_FWD = 14,
    SPARK_FLT_HARD_LIMIT_REV = 15
} SparkFaultIds;


/**
 * @brief Constructor for the class. Add an instance for the global CAN communications.
 * 
 * @param deviceId - ID for this device on the CAN network.
 */
SparkMc::SparkMc(uint8_t deviceId) {
    this->deviceId = deviceId;
    activeDevices |= bit(deviceId);
    
    // Add another instance for ISR stuff
    instanceCount++;
    devices = (SparkMc**)realloc(devices, instanceCount * sizeof(SparkMc*));
    devices[instanceCount-1] = this;
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
void SparkMc::addCanSender(void (*canSender)(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *dat)) {
    SparkMc::canSender = canSender;
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
void SparkMc::addCanReceiver(unsigned pinNum, unsigned (*canReceiver)(uint32_t *id, uint8_t *dat)) {
    SparkMc::interruptPin = pinNum;
    SparkMc::canReceiver = canReceiver;
    pinMode(pinNum, INPUT);
    attachInterrupt(digitalPinToInterrupt(pinNum), SparkMc::receiveAllIncoming, FALLING); 
}

/**
 * @brief Send some data over the CAN network.
 * 
 * @param id - ID for the message.
 * @param len - Length of the message.
 * @param dat - Message data (array of bytes).
 */
void SparkMc::canSend(unsigned long id, uint8_t len, const uint8_t *dat) {
    // Ignore any incoming messages during the send
    if (SparkMc::interruptPin != -1) {
        detachInterrupt(digitalPinToInterrupt(SparkMc::interruptPin));
    }
    
    // Do the send using the provided sender
    canSender(id, EXT_BIT, RTR_BIT, len, dat);
    
    // Receive anything that happened during that time and re-enable interrupts
    if (SparkMc::interruptPin != -1) {
        SparkMc::receiveAllIncoming();
        attachInterrupt(digitalPinToInterrupt(SparkMc::interruptPin), SparkMc::receiveAllIncoming, FALLING); 
    }
}

/**
 * Receive all incoming CAN messages.
 */
void SparkMc::receiveAllIncoming() {
    while (digitalRead(SparkMc::interruptPin) == LOW) {
        SparkMc::receiveIncoming();
    }
}

/**
 * Receive data incoming to the CAN module. This uses the receiver function set using 
 * addCanReceiver(). Any received data is sent to the appropriate MC instance.
 */
bool SparkMc::receiveIncoming() {
    // Read the incoming message
    uint32_t id = 0;
    static byte dat[10] = {0};
    unsigned len = SparkMc::canReceiver(&id, dat);

    const uint16_t OUR_TYPE_CODE = REV | (motorController << 8);
    // Check to make sure the manufacturer and device type are good
    uint16_t typeCode = id >> 16;
    if (typeCode != OUR_TYPE_CODE) {
        return false;
    }
    
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

    // Check to make sure length is good
    if (len != 8) {
        return false;
    }

    // Receive the message
    unsigned msgType = (id >> 6) & 0x3FF;
    switch (msgType) {
    case CMD_API_STAT0:
        memcpy(&devices[devIdx]->lastStat0, dat, sizeof(PeriodicStatus0));
        break;
    case CMD_API_STAT1:
        memcpy(&devices[devIdx]->lastStat1, dat, sizeof(PeriodicStatus1));
        break;
    case CMD_API_STAT2:
        memcpy(&devices[devIdx]->lastStat2, dat, sizeof(PeriodicStatus2));
        break;
    default:
        return false;
    }

    return true;
}

/**
 * Send a single-value CAN message to this motor controller.
 * 
 * @param msgId - id for this message type.
 * @param val - Value to send
 */
void SparkMc::sendVal(uint16_t msgId, float val) {
    uint32_t frameId = makeCanId(REV, motorController, msgId, deviceId);
    SparkMc::canSend(frameId, 8, float2buff(val, sendBuffer));
}

/**
 * Send a CAN message to this motor controller.
 * 
 * @param msgId - id for this message type.
 * @param buffer - Buffer with values to send.
 */
void SparkMc::sendVal(uint16_t msgId, uint8_t* buffer) {
    uint32_t frameId = makeCanId(REV, motorController, msgId, deviceId);
    SparkMc::canSend(frameId, 8, buffer);
}

void SparkMc::setInverted(bool invert) {
    // TODO: Implement this!
}

void SparkMc::setIdleMode(IdleMode mode) {
    // TODO: Implement this!
}

void SparkMc::enableVoltageCompensation(double nominalVoltage) {
    // TODO: Implement this!
}

/**
 * Disables the voltage compensation setting for all modes on the SPARK MAX.
 */
void SparkMc::disableVoltageCompensation() {
    // TODO: Implement this!
}

/**
 * Get the motor temperature in celcius.
 * 
 * @return the motor temperature.
 */
float SparkMc::getTemperature() {
    return lastStat1.mtrTemp;
}

/**
 * Returns the voltage fed into the motor controller.
 */
float SparkMc::getBusVoltage() {
    return lastStat1.mtrVoltage / 128.0;
}

/**
 * Returns motor controller's output duty cycle.
 */
float SparkMc::getAppliedOutput() {
    return float(lastStat0.appliedOutput) / 0x8000u;
}

/**
 * Returns motor controller's output current in Amps.
 */
float SparkMc::getOutputCurrent() {
    return lastStat1.mtrCurrent / 32.0;
}

/**
 * Get the motor position.
 * 
 * @return the motor position in rotations
 */
float SparkMc::getPosition() {
    return lastStat2.sensorPos;
}

/**
 * Get the motor velocity in rpm.
 * 
 * @return the motor velocity.
 */
float SparkMc::getVelocity() {
    return lastStat1.sensorVel;
}

/**
 * Check if actuator hit the forward limit switch.
 * 
 * @return true if hit, false otherwise. 
 */
bool SparkMc::limitForward() {
    return bitRead(lastStat0.faults, SPARK_FLT_HARD_LIMIT_FWD);
}

/**
 * Check if actuator hit the reverse limit switch.
 * 
 * @return true if hit, false otherwise. 
 */
bool SparkMc::limitReverse() {
    return bitRead(lastStat0.faults, SPARK_FLT_HARD_LIMIT_REV);
}

/**
 * Restore motor controller parameters to factory default
 *
 * @param persist If true, burn the flash with the factory default
 * parameters
 *
 * @return CANError Set to CANError::kOk if successful
 */
void SparkMc::restoreFactoryDefaults(bool persist) {
    // TODO: Implement this!
}


/**
 * Writes all settings to flash.
 */
void SparkMc::burnFlash() {
    // TODO: Implement this!
}


/**
 * Set the PID config slot used by the MC.
 * 
 * @param slot - Slot number. Value in 0-3.
 */
void SparkMc::setPidSlot(uint8_t slot) {
    if (slot <= 3) {
        pidSlot = slot;
    }
}

/**
 * Set the control mode to voltage and set the value.
 * 
 * @param power - percent output. Float value in [-1.0, 1.0].
 */
void SparkMc::setPower(float power) {
    sendVal(CMD_API_DC_SET, power);
}

/**
 * Set the control mode to velocity and set the target value.
 * 
 * @param velocity - Target velocity in RPM.
 */
void SparkMc::setVelocity(float velocity) {
    float2buff(velocity, sendBuffer);
    sendBuffer[6] = pidSlot;

    sendVal(CMD_API_SPD_SET, sendBuffer);
}

/**
 * Set the control mode to position and set the target value.
 * 
 * @param position - Target position in rotations.
 */
void SparkMc::setPosition(float position) {
    float2buff(position, sendBuffer);
    sendBuffer[6] = pidSlot;

    sendVal(CMD_API_POS_SET, sendBuffer);
}

/**
 * Clear any faults present with the motor controller. 
 */
void SparkMc::clearFaults() {
    // Clear faults
    uint32_t frameId = makeCanId(REV, motorController, CMD_API_CLEAR_FAULTS, deviceId);
    SparkMc::canSend(frameId, 8, int2buff(bit(deviceId), sendBuffer));
}

/**
 * Send the keep-alive message to all motor controllers registered with this class. Must be sent every ~100ms.
 * Only needs to be sent once for all connected SPARK MCs. 
 */
void SparkMc::sendKeepAlive() {
    static unsigned long lastSend = 0;
    static uint8_t buff[8] = {0};

    if (millis() - lastSend > 10) {
        uint32_t frameId = makeCanId(REV, motorController, CMD_API_NONRIO_HB, 0);
        SparkMc::canSend(frameId, 8, int2buff(activeDevices, buff));
        lastSend = millis();
    }
}