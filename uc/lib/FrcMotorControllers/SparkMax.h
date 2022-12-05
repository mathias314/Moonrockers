#ifndef __SPARK_MAX_H__
#define __SPARK_MAX_H__

#include <Arduino.h>

#include "FrcMotorController.h"

class SparkMax : public FrcMotorController {
   public:
    enum class MotorType { kBrushed = 0,
                           kBrushless = 1 };

    SparkMax(uint8_t deviceId);

    void setInverted(bool invert);
    void setIdleMode(IdleMode mode);
    void setPower(float power);
    void setVelocity(float velocity);
    void setPosition(float position);
    void setPidSlot(uint8_t slot);

    void enableVoltageCompensation(double nominalVoltage);
    void disableVoltageCompensation();
    float getAppliedOutput();
    float getBusVoltage();
    float getOutputCurrent();
    float getTemperature();
    float getPosition();
    float getVelocity();
    bool limitForward();
    bool limitReverse();
    void restoreFactoryDefaults(bool persist = false);
    void burnFlash();

    void clearFaults();

   protected:
    static void handleKeepAlive();

    fptr getKeepAliveHandler() {
        return SparkMax::handleKeepAlive;
    }

   private:
    typedef struct PACKED {
        int16_t appliedOutput;  // 16-bit signed integer representing the applied output
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
        float sensorVel;           // 32-bit float representing the motor velocity in RPM
        uint8_t mtrTemp;           // Motor temperature in Celcius
        uint16_t mtrVoltage : 12;  // 12-bit fixed point value representing the motor voltage
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

    // Track the most recently received status messages
    PeriodicStatus0 lastStat0;
    PeriodicStatus1 lastStat1;
    PeriodicStatus2 lastStat2;

    uint8_t pidSlot = 0;  // PID config used for closed loop control. 0-3.
    static uint16_t activeDevices;  // Data value for the keep-alive message

    bool handleIncoming(unsigned msgType, byte* dat, unsigned len);
};

#endif