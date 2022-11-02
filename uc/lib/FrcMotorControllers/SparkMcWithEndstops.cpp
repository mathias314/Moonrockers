#include "SparkMcWithEndstops.h"
#include "SparkMc.h"
#include "CanUtils.h"

SparkMcWithEndstops* SparkMcWithEndstops::mcArr[6] = {0};
uint8_t SparkMcWithEndstops::endstopArr[6];
int SparkMcWithEndstops::isrCounter = 0;
void (*SparkMcWithEndstops::isrArr[6])() = {SparkMcWithEndstops::isr0, SparkMcWithEndstops::isr1, SparkMcWithEndstops::isr2, SparkMcWithEndstops::isr3, SparkMcWithEndstops::isr4, SparkMcWithEndstops::isr5};

SparkMcWithEndstops::SparkMcWithEndstops(uint8_t deviceId, int reverseStopPin, int forwardStopPin) : SparkMc(deviceId)
{
  this->reverseStopPin = reverseStopPin;
  this->forwardStopPin = forwardStopPin;
}

void SparkMcWithEndstops::init()
{
  pinMode(reverseStopPin, INPUT_PULLUP);
  pinMode(forwardStopPin, INPUT_PULLUP);

  mcArr[isrCounter] = this;
  endstopArrIndexReverse = isrCounter;
  attachInterrupt(digitalPinToInterrupt(reverseStopPin), isrArr[isrCounter++], FALLING);
  mcArr[isrCounter] = this;
  endstopArrIndexForward = isrCounter;
  attachInterrupt(digitalPinToInterrupt(forwardStopPin), isrArr[isrCounter++], FALLING);
}

void SparkMcWithEndstops::setPower(float power)
{
  if (power <= 0 && !digitalRead(reverseStopPin))
  {
    power = 0;
  }
  else if (power >= 0 && !digitalRead(forwardStopPin))
  {
    power = 0;
  }
  
  SparkMc::setPower(power);
  lastPower = power;
}

void SparkMcWithEndstops::refresh()
{
  // if the interrupt for the reverse endstop fired
  if (endstopArr[endstopArrIndexReverse])
  {
    // clear the flag
    endstopArr[endstopArrIndexReverse] = 0;

    // cut motor power if we were travelling towards the reverse endstop
    if (lastPower < 0)
    {
      mcArr[endstopArrIndexReverse]->setPower(0);
    }
  }

  // if the interrupt for the forward endstop fired
  if (endstopArr[endstopArrIndexForward])
  {
    // clear the flag
    endstopArr[endstopArrIndexForward] = 0;

    // cut motor power if we were travelling towards the forward endstop
    if (lastPower > 0)
    {
      mcArr[endstopArrIndexForward]->setPower(0);
    }
  }
}

void SparkMcWithEndstops::isr0()
{
  endstopArr[0] = 1;
}

void SparkMcWithEndstops::isr1()
{
  endstopArr[1] = 1;
}

void SparkMcWithEndstops::isr2()
{
  endstopArr[2] = 1;
}

void SparkMcWithEndstops::isr3()
{
  endstopArr[3] = 1;
}

void SparkMcWithEndstops::isr4()
{
  endstopArr[4] = 1;
}

void SparkMcWithEndstops::isr5()
{
  endstopArr[5] = 1;
}
