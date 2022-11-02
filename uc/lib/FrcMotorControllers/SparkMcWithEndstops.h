#ifndef SPARKMCWITHENDSTOPS_H
#define SPARKMCWITHENDSTOPS_H

#include "SparkMc.h"

class SparkMcWithEndstops : public SparkMc
{
public:
  SparkMcWithEndstops(uint8_t deviceId, int reverseStopPin, int forwardStopPin);
  void init(); // must call in setup() to set up interrupts and pinModes
  void setPower(float power);
  void refresh(); // call as often as possible to ensure the motor is quickly shut off when it hits an endstop
  static void isr0();
  static void isr1();
  static void isr2();
  static void isr3();
  static void isr4();
  static void isr5();

private:
  int reverseStopPin = 0;
  int forwardStopPin = 0;

  int endstopArrIndexReverse = 0;
  int endstopArrIndexForward = 0;
  float lastPower = 0;

  static SparkMcWithEndstops* mcArr[6];
  static uint8_t endstopArr[6];
  static int isrCounter;
  static void (*isrArr[6])();
};

#endif // SPARKMCWITHENDSTOPS_H
