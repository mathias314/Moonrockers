/* 
 * Class for the Encoder.
 * This class provides functions for tracking encoder ticks and determining speed.
 * 
 * Note: this class is only set up to support one encoder. Attempting to use it for more will break things.
 */

#include "Encoder.h"

// for use by ISR routine
Encoder * Encoder::instance;
/**
 * @brief Constructor for the class.
 * 
 * @param aPin - First encoder pin. Attached to interrupt.
 * @param bPin - second encoder pin.
 * @param ticksPerRotation - Number of encoder ticks per rotation.
 */
Encoder::Encoder(int aPin, int ticksPerRotation) : Encoder(aPin, -1, ticksPerRotation) {}
Encoder::Encoder(int aPin, int bPin, int ticksPerRotation) 
    : aPin(aPin), bPin(bPin), ticksPerRotation(ticksPerRotation), speedFilter(15, 15, 0.2)  { //speedFilter(300, 300, 0.025)
}

/**
 * @brief Initialize the encoder.
 */
void Encoder::init() {
    instance = this;

    //Setup control pins
    pinMode(aPin, INPUT_PULLUP);
    
    //Set up the encoder interrupt pin
    int intNumA = digitalPinToInterrupt(aPin);
    attachInterrupt(intNumA, Encoder::isrA, CHANGE);

    // Get the register and bit for the pins to make the ISR a bit faster
    aPinRegister = portInputRegister(digitalPinToPort(aPin));
    aPinBit = digitalPinToBitMask(aPin);
    
    // Same for b pin if we have it (otherwise, treat this as a tach)
    if (bPin != -1) {
      pinMode(bPin, INPUT_PULLUP);
      int intNumB = digitalPinToInterrupt(bPin);
      attachInterrupt(intNumB, Encoder::isrB, CHANGE);
      bPinRegister = portInputRegister(digitalPinToPort(bPin));
      bPinBit = digitalPinToBitMask(bPin);
    }
    
    //initialize the timer
    lastTickTime = micros();
    lastEstTime = lastTickTime;

    tickConversion = 1000000ul * 60 / ticksPerRotation;
}


/**
 * @brief set whether the encodeer's readings should be inverted.
 * 
 * @param invertDir - true if should be inverted, false otherwise
 */
void Encoder::invertDirection(bool invertDir) {
  this->invertDir = invertDir;
}

float Encoder::estimateSpeed(bool direction) {
  // true direction = positive speed
  if (direction)
  {
    this->invertDir = false;
  }
  else
  {
    this->invertDir = true;
  }
  return this->estimateSpeed();
}

/**
 * Update the current speed estimate and return the filtered value.
 */
float Encoder::estimateSpeed() {
  // Calculate the speed if we got a tick. Otherwise assume 0.
  if (lastTickTime != lastEstTime) {
    speed = ticks * (tickConversion / (lastTickTime - lastEstTime));
  } else {
    speed = 0;
  }
  
  // Reset things
  lastEstTime = lastTickTime;
  absTot += ticks;
  ticks = 0;

  // Invert if needed
  if (invertDir) {
    speed = -speed;
  }

  // Filter
  filteredSpeed = speedFilter.updateEstimate(speed);

  // Update our cached speed
  lastFilteredSpeed = filteredSpeed;


  // Return the speed
  return filteredSpeed;
}

/**
 * @brief Get the current speed the motor is running at in RPMs.
 * 
 * @return the current speed of the motor.
 */
float Encoder::getSpeed() {
  return speed;
}

/**
 * @brief Get the filtered current speed the motor is running at in RPMs.
 * 
 * @return the filtered current speed of the motor.
 */
float Encoder::getFilteredSpeed() {
  return filteredSpeed;
}

/**
 * Count a tick and set the last tick time.
 * 
 * NOTE: This ISR takes about 8 microseconds.
 * 
 * @param trigA - interrupt triggered by channel A or no.
 */
void Encoder::tick(bool trigA) {
  lastTickTime = micros();

  if (bPin == -1)
  {
    ++ticks;
  } else {
    // Get current pin values
    bool aLevel = (*aPinRegister) & aPinBit;
    bool bLevel = (*bPinRegister) & bPinBit;

    // Increase or decrease depending on combination and which pin triggered the interrupt
    if (trigA != (aLevel == bLevel)) {
      ++ticks;
    } else {
      --ticks;
    }
  }
}

/**
 * @brief Handle interrupt. Forward it to stored instance.
 */
void Encoder::isrA() {
  instance->tick(true);
}

void Encoder::isrB() {
  instance->tick(false);
}
