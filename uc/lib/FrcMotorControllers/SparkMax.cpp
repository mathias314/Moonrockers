
#include "SparkMax.h"

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


// CANSpark API class;
const int CMD_API_SETPNT_SET = 0x001;
const int CMD_API_DC_SET = 0x002;
const int CMD_API_SPD_SET = 0x012;
const int CMD_API_SMART_VEL_SET = 0x013;
const int CMD_API_POS_SET = 0x032;
const int CMD_API_VOLT_SET = 0x042;
const int CMD_API_CURRENT_SET = 0x043;
const int CMD_API_SMARTMOTION_SET = 0x052;
const int CMD_API_STAT0 = 0x060;
const int CMD_API_STAT1 = 0x061;
const int CMD_API_STAT2 = 0x062;
const int CMD_API_STAT3 = 0x063;
const int CMD_API_STAT4 = 0x064;
const int CMD_API_CLEAR_FAULTS = 0x06E;
const int CMD_API_DRV_STAT = 0x06A;
const int CMD_API_BURN_FLASH = 0x072;
const int CMD_API_SET_FOLLOWER = 0x073;
const int CMD_API_FACTORY_DEFAULT = 0x074;
const int CMD_API_FACTORY_RESET = 0x075;
const int CMD_API_IDENTIFY = 0x076;
const int CMD_API_NACK = 0x080;
const int CMD_API_ACK = 0x081;
const int CMD_API_BROADCAST = 0x090;
const int CMD_API_HEARTBEAT = 0x092;
const int CMD_API_SYNC = 0x093;
const int CMD_API_ID_QUERY = 0x094;
const int CMD_API_ID_ASSIGN = 0x095;
const int CMD_API_FIRMWARE = 0x098;
const int CMD_API_ENUM = 0x099;
const int CMD_API_LOCK = 0x09B;
const int CMD_API_LOCKB = 0x0B1;
const int CMD_API_NONRIO_HB = 0x0B2;

uint16_t SparkMax::activeDevices = 0;

/**
 * @brief Constructor for the class. Add an instance for the global CAN communications.
 *
 * @param deviceId - ID for this device on the CAN network.
 */
SparkMax::SparkMax(uint8_t deviceId) : FrcMotorController(deviceId) {
    this->deviceManufacturer = REV;
    FrcMotorController::addKeepAliveCallback(SparkMax::handleKeepAlive);
    activeDevices |= bit(deviceId);
}

/**
 * Receive data incoming to the CAN module. This uses the receiver function set using
 * addCanReceiver(). Any received data is sent to the appropriate MC instance.
 */
bool SparkMax::handleIncoming(unsigned msgType, byte* dat, unsigned len) {
    // Read the incoming message
    switch (msgType) {
        case CMD_API_STAT0:
            memcpy(&this->lastStat0, dat, sizeof(PeriodicStatus0));
            break;
        case CMD_API_STAT1:
            memcpy(&this->lastStat1, dat, sizeof(PeriodicStatus1));
            break;
        case CMD_API_STAT2:
            memcpy(&this->lastStat2, dat, sizeof(PeriodicStatus2));
            break;
        default:
            return false;
    }

    return true;
}

void SparkMax::setInverted(bool invert) {
    // TODO: Implement this!
}

void SparkMax::setIdleMode(IdleMode mode) {
    // TODO: Implement this!
}

void SparkMax::enableVoltageCompensation(double nominalVoltage) {
    // TODO: Implement this!
}

/**
 * Disables the voltage compensation setting for all modes on the SPARK MAX.
 */
void SparkMax::disableVoltageCompensation() {
    // TODO: Implement this!
}

/**
 * Get the motor temperature in celcius.
 *
 * @return the motor temperature.
 */
float SparkMax::getTemperature() {
    return lastStat1.mtrTemp;
}

/**
 * Returns the voltage fed into the motor controller.
 */
float SparkMax::getBusVoltage() {
    return lastStat1.mtrVoltage / 128.0;
}

/**
 * Returns motor controller's output duty cycle.
 */
float SparkMax::getAppliedOutput() {
    return float(lastStat0.appliedOutput) / 0x8000u;
}

/**
 * Returns motor controller's output current in Amps.
 */
float SparkMax::getOutputCurrent() {
    return lastStat1.mtrCurrent / 32.0;
}

/**
 * Get the motor position.
 *
 * @return the motor position in rotations
 */
float SparkMax::getPosition() {
    return lastStat2.sensorPos;
}

/**
 * Get the motor velocity in rpm.
 *
 * @return the motor velocity.
 */
float SparkMax::getVelocity() {
    return lastStat1.sensorVel;
}

/**
 * Check if actuator hit the forward limit switch.
 *
 * @return true if hit, false otherwise.
 */
bool SparkMax::limitForward() {
    return bitRead(lastStat0.faults, SPARK_FLT_HARD_LIMIT_FWD);
}

/**
 * Check if actuator hit the reverse limit switch.
 *
 * @return true if hit, false otherwise.
 */
bool SparkMax::limitReverse() {
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
void SparkMax::restoreFactoryDefaults(bool persist) {
    // TODO: Implement this!
}

/**
 * Writes all settings to flash.
 */
void SparkMax::burnFlash() {
    // TODO: Implement this!
}

/**
 * Set the PID config slot used by the MC.
 *
 * @param slot - Slot number. Value in 0-3.
 */
void SparkMax::setPidSlot(uint8_t slot) {
    if (slot <= 3) {
        pidSlot = slot;
    }
}

/**
 * Set the control mode to voltage and set the value.
 *
 * @param power - percent output. Float value in [-1.0, 1.0].
 */
void SparkMax::setPower(float power) {
    sendVal(CMD_API_DC_SET, power);
}

/**
 * Set the control mode to velocity and set the target value.
 *
 * @param velocity - Target velocity in RPM.
 */
void SparkMax::setVelocity(float velocity) {
    float2buff(velocity, sendBuffer);
    sendBuffer[6] = pidSlot;

    sendVal(CMD_API_SPD_SET, sendBuffer);
}

/**
 * Set the control mode to position and set the target value.
 *
 * @param position - Target position in rotations.
 */
void SparkMax::setPosition(float position) {
    float2buff(position, sendBuffer);
    sendBuffer[6] = pidSlot;

    sendVal(CMD_API_POS_SET, sendBuffer);
}

/**
 * Clear any faults present with the motor controller.
 */
void SparkMax::clearFaults() {
    // Clear faults
    sendVal(CMD_API_CLEAR_FAULTS, int2buff(bit(deviceId), sendBuffer));
}

/**
 * Send the keep-alive message to all motor controllers registered with this class. Must be sent every ~100ms.
 * Only needs to be sent once for all connected SPARK MCs.
 */
void SparkMax::handleKeepAlive() {
    static unsigned long lastSend = 0;
    static uint8_t buff[8] = {0};

    if (millis() - lastSend > KEEP_ALIVE_INTERVAL) {
        uint32_t frameId = makeCanId(REV, motorController, CMD_API_NONRIO_HB, 0);
        FrcMotorController::canSend(frameId, 8, int2buff(activeDevices, buff));
        lastSend = millis();
    }
}