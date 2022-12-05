#ifndef __SPARK_MAX_H__
#define __SPARK_MAX_H__

#include <Arduino.h>

#include "FrcMotorController.h"

class TalonSrx : public FrcMotorController {
   public:
    TalonSrx(uint8_t deviceId);

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

   private:
    bool handleIncoming(unsigned msgType, byte* dat, unsigned len);
};

#endif