#ifndef __SPARK_MC_H__
#define __SPARK_MC_H__

#include <Arduino.h>
#include "CanUtils.h"

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

class SparkMc {
   public:
    enum class IdleMode { kCoast = 0,
                          kBrake = 1 };
    enum class MotorType { kBrushed = 0,
                           kBrushless = 1 };

    static void addCanSender(void (*canSender)(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *dat));
    static void addCanReceiver(unsigned pinNum, unsigned (*canReceiver)(uint32_t *id, uint8_t *dat));
    static void canSend(unsigned long id, uint8_t len, const uint8_t *dat);
    static bool receiveIncoming();
    static void receiveAllIncoming();

    SparkMc(uint8_t deviceId);

    void setPower(float power);
    void setVelocity(float velocity);
    void setPosition(float position);
    void setPidSlot(uint8_t slot);

    void setInverted(bool invert = true);
    void setIdleMode(IdleMode mode);
    void enableVoltageCompensation(double nominalVoltage);
    void disableVoltageCompensation();
    float getBusVoltage();
    float getAppliedOutput();
    float getOutputCurrent();
    float getTemperature();
    float getPosition();
    float getVelocity();
    bool limitForward();
    bool limitReverse();
    void restoreFactoryDefaults(bool persist = false);
    void burnFlash();

    void clearFaults();

    static void sendKeepAlive();

   private:
    typedef struct PACKED {
        int16_t appliedOutput; //16-bit signed integer representing the applied output
        uint16_t faults;
        uint16_t stickyFaults;
        uint8_t sensorInv : 1;
        uint8_t setpointInv : 1;
        uint8_t lock : 2;
        uint8_t mtrType : 1;
        uint8_t isFollower : 1;
        uint8_t roboRIO : 1;
        uint8_t rsvd0 : 1;
    } PeriodicStatus0;

    typedef struct __attribute__((__packed__)) {
        float sensorVel;  // 32-bit float representing the motor velocity in RPM
        uint8_t mtrTemp;  // Motor temperature in Celcius 
        uint16_t mtrVoltage : 12; //12-bit fixed point value representing the motor voltage
        uint16_t mtrCurrent : 12;
    } PeriodicStatus1;

    typedef struct __attribute__((__packed__)) {
        float sensorPos;  // Motor position in rotations
        int32_t iAccum;
    } PeriodicStatus2;

    typedef struct __attribute__((__packed__)) {
        uint32_t analogVoltage : 10;
        int32_t analogVel : 22;
        int32_t analogPos;
    } PeriodicStatus3;

    typedef struct __attribute__((__packed__)) {
        float altEncoderVelocity;
        float altEncoderPosition;
    } PeriodicStatus4;

    static void (*canSender)(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *dat);
    static unsigned (*canReceiver)(uint32_t *id, uint8_t *dat);
    void sendVal(uint16_t msgId, float val);
    void sendVal(uint16_t msgId, uint8_t *buffer);
    void receiveMessage();

    // Track the most recently received status messages
    PeriodicStatus0 lastStat0;
    PeriodicStatus1 lastStat1;
    PeriodicStatus2 lastStat2;

    uint8_t deviceId = 0;

    uint8_t sendBuffer[8] = {0};

    uint8_t pidSlot = 0;  // PID config used for closed loop control. 0-3.

    volatile static int interruptPin;
    static uint16_t activeDevices;  // Data value for the keep-alive message
    static SparkMc **devices;
    static unsigned instanceCount;
};

#endif