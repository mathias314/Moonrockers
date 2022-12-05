
#include "TalonSrx.h"

// Talon SRX message IDs
const int CMD_API_DC_SET = 0x002;
const int CMD_API_KEEP_ALIVE = 0x006;

/**
 * @brief Constructor for the class. Add an instance for the global CAN communications.
 *
 * @param deviceId - ID for this device on the CAN network.
 */
TalonSrx::TalonSrx(uint8_t deviceId) : FrcMotorController(deviceId) {
    this->deviceManufacturer = CTRE;
    FrcMotorController::addKeepAliveCallback(TalonSrx::handleKeepAlive);
}

/**
 * Receive data incoming to the CAN module. This uses the receiver function set using
 * addCanReceiver(). Any received data is sent to the appropriate MC instance.
 */
bool TalonSrx::handleIncoming(unsigned msgType, byte* dat, unsigned len) {
    // Read the incoming message
    //TODO: Implement this!

    return true;
}

void TalonSrx::setInverted(bool invert) {
    // TODO: Implement this!
}

void TalonSrx::setIdleMode(IdleMode mode) {
    // TODO: Implement this!
}

void TalonSrx::enableVoltageCompensation(double nominalVoltage) {
    // TODO: Implement this!
}

/**
 * Disables the voltage compensation setting for all modes on the SPARK MAX.
 */
void TalonSrx::disableVoltageCompensation() {
    // TODO: Implement this!
}

/**
 * Get the motor temperature in celcius.
 *
 * @return the motor temperature.
 */
float TalonSrx::getTemperature() {
    // TODO: Implement this!
    return 0.0;
}

/**
 * Returns the voltage fed into the motor controller.
 */
float TalonSrx::getBusVoltage() {
    // TODO: Implement this!
    return 0.0;
}

/**
 * Returns motor controller's output duty cycle.
 */
float TalonSrx::getAppliedOutput() {
    // TODO: Implement this!
    return 0.0;
}

/**
 * Returns motor controller's output current in Amps.
 */
float TalonSrx::getOutputCurrent() {
    // TODO: Implement this!
    return 0.0;
}

/**
 * Get the motor position.
 *
 * @return the motor position in rotations
 */
float TalonSrx::getPosition() {
    // TODO: Implement this!
    return 0.0;
}

/**
 * Get the motor velocity in rpm.
 *
 * @return the motor velocity.
 */
float TalonSrx::getVelocity() {
    // TODO: Implement this!
    return 0.0;
}

/**
 * Check if actuator hit the forward limit switch.
 *
 * @return true if hit, false otherwise.
 */
bool TalonSrx::limitForward() {
    // TODO: Implement this!
    return false;
}

/**
 * Check if actuator hit the reverse limit switch.
 *
 * @return true if hit, false otherwise.
 */
bool TalonSrx::limitReverse() {
    // TODO: Implement this!
    return false;
}

/**
 * Restore motor controller parameters to factory default
 *
 * @param persist If true, burn the flash with the factory default
 * parameters
 *
 * @return CANError Set to CANError::kOk if successful
 */
void TalonSrx::restoreFactoryDefaults(bool persist) {
    // TODO: Implement this!
}

/**
 * Writes all settings to flash.
 */
void TalonSrx::burnFlash() {
    // TODO: Implement this!
}

/**
 * Set the PID config slot used by the MC.
 *
 * @param slot - Slot number. Value in 0-3.
 */
void TalonSrx::setPidSlot(uint8_t slot) {
    // TODO: Implement this!
}

/**
 * Set the control mode to voltage and set the value.
 *
 * @param power - percent output. Float value in [-1.0, 1.0].
 */
void TalonSrx::setPower(float power) {
    int intPower = constrain(int(power*1023), -1023, 1023);
    clearBuffer(sendBuffer);
    sendBuffer[0] = (byte)(intPower >> 0x10);
    sendBuffer[1] = (byte)(intPower >> 0x08);
    sendBuffer[2] = (byte)(intPower);
    sendVal(CMD_API_DC_SET, sendBuffer);
}

/**
 * Set the control mode to velocity and set the target value.
 *
 * @param velocity - Target velocity in RPM.
 */
void TalonSrx::setVelocity(float velocity) {
    // TODO: Implement this!
}

/**
 * Set the control mode to position and set the target value.
 *
 * @param position - Target position in rotations.
 */
void TalonSrx::setPosition(float position) {
    // TODO: Implement this!
}

/**
 * Clear any faults present with the motor controller.
 */
void TalonSrx::clearFaults() {
    // Clear faults
    // TODO: Implement this!
}

/**
 * Send the keep-alive message to all motor controllers registered with this class. Must be sent every ~100ms.
 * Only needs to be sent once for all connected SPARK MCs.
 */
void TalonSrx::handleKeepAlive() {
    static unsigned long lastSend = 0;
    static uint8_t buff[8] = {0};

    if (millis() - lastSend > KEEP_ALIVE_INTERVAL) {
        uint32_t frameId = makeCanId(CTRE, deviceBroadcast, CMD_API_KEEP_ALIVE, 0x3F);
        FrcMotorController::canSend(frameId, 8, int2buff(1, buff));
    
        lastSend = millis();
    }
}