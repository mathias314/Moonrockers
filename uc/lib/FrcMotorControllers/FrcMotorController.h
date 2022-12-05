/**
 * Base class for FRC motor controllers. Contains the core CAN communication stuff
 * and defines the interface for the motor controller classes.
 */
#ifndef __FRC_MOTOR_CONTROLLER_H__
#define __FRC_MOTOR_CONTROLLER_H__

#include <Arduino.h>

#include "CanUtils.h"

class FrcMotorController {
   public:
    enum class IdleMode { kCoast = 0,
                          kBrake = 1 };

    FrcMotorController(uint8_t deviceId);

    static void addCanSender(void (*canSender)(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *dat));
    static void addCanReceiver(unsigned pinNum, unsigned (*canReceiver)(uint32_t *id, uint8_t *dat));

    virtual void setPower(float power) = 0;
    virtual void setInverted(bool invert = true) = 0;
    virtual void setIdleMode(IdleMode mode) = 0;
    virtual float getAppliedOutput() = 0;

    virtual void clearFaults() = 0;

    static void sendKeepAlive();

   protected:
    typedef void (*fptr)();

    static const unsigned KEEP_ALIVE_INTERVAL = 25;  // Interval in ms for sending keep-alive message
    uint8_t deviceId = 0;
    CanDeviceManufacturer deviceManufacturer;
    uint8_t sendBuffer[8] = {0};

    virtual fptr getKeepAliveHandler() {return nullptr;}

    void sendVal(uint16_t msgId, float val);
    void sendVal(uint16_t msgId, uint8_t *buffer);
    static void canSend(unsigned long id, uint8_t len, const uint8_t *dat);
    static void addKeepAliveCallback(fptr callback);


   private:
    static bool receiveIncoming();
    static void receiveAllIncoming();
    void receiveMessage();

    virtual bool handleIncoming(unsigned msgType, byte *dat, unsigned len) = 0;

    static void (*canSender)(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *dat);
    static unsigned (*canReceiver)(uint32_t *id, uint8_t *dat);

    volatile static int interruptPin;
    static FrcMotorController **devices;
    static unsigned instanceCount;

    // Array of keepalive handlers
    static unsigned keepAliveCount;
    static fptr *keepAliveHandlers;
};

#endif