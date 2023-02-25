/* 
 * Class for the Encoder.
 * This class provides functions for tracking encoder ticks and determining speed.
 * 
 * Note: this class is only set up to support one encoder. Attempting to use it for more will break things.
 */

#include "Encoder.h"

// for use by ISR routine
int Encoder::numInstances = 0;
Encoder *Encoder::instance0;
Encoder *Encoder::instance1;
Encoder *Encoder::instance2;
Encoder *Encoder::instance3;
Encoder *Encoder::instance4;
Encoder *Encoder::instance5;
Encoder *Encoder::instance6;
Encoder *Encoder::instance7;
Encoder *Encoder::instance8;
Encoder *Encoder::instance9;


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
bool Encoder::init() {
    if (!bindInstance())
      return false;

    //Setup control pins
    pinMode(aPin, INPUT_PULLUP);
    
    //Set up the encoder interrupt pin
    bindInterruptA(aPin);

    // Get the register and bit for the pins to make the ISR a bit faster
    aPinRegister = portInputRegister(digitalPinToPort(aPin));
    aPinBit = digitalPinToBitMask(aPin);
    
    // Same for b pin if we have it (otherwise, treat this as a tach)
    if (bPin != -1) {
      pinMode(bPin, INPUT_PULLUP);
      bindInterruptB(bPin);
      bPinRegister = portInputRegister(digitalPinToPort(bPin));
      bPinBit = digitalPinToBitMask(bPin);
    }
    
    //initialize the timer
    lastTickTime = micros();
    lastEstTime = lastTickTime;

    tickConversion = 1000000ul * 60 / ticksPerRotation;
    return true;
}

/**
 * If there are less than 6 instances of the class, increment the number of instances, and bind this
 * instance to a static instance for ISR access
 *
 * @return True if an instance was binded, false if all instances are already binded
 */
bool Encoder::bindInstance() {
  Serial.println(numInstances);
  // make sure we aren't at max instances
  if (numInstances >= maxInstances) {
    Serial.println("***Failing by max instances***");
    return false;
  }

  // bind this instance of the class to a static instance for ISR access
  switch (numInstances) {
    case 0:
      instance0 = this;
      break;
    case 1:
      instance1 = this;
      break;
    case 2:
      instance2 = this;
      break;
    case 3:
      instance3 = this;
      break;
    case 4:
      instance4 = this;
      break;
    case 5:
      instance5 = this;
      break;
    case 6:
      instance6 = this;
      break;
    case 7:
      instance7 = this;
      break;
    case 8:
      instance8 = this;
      break;
    case 9:
      instance9 = this;
      break;
    default:
      Serial.println("***Failing by default***");
      return false;
  }

  // increment the static counter of motors
  numInstances++;

  return true;
}

/**
 * Sets up the interrupt for the A pin of the encoder
 *
 * @param aPin The pin number of the A channel of the encoder.
 */
void Encoder::bindInterruptA(int aPin) {
    //Set up the encoder interrupt pin
    int intNumA = digitalPinToInterrupt(aPin);
    switch (numInstances) {
      case 0:
        attachInterrupt(intNumA, isr0A, CHANGE);
        break;

      case 1:
        attachInterrupt(intNumA, isr1A, CHANGE);
        break;

      case 2:
        attachInterrupt(intNumA, isr2A, CHANGE);
        break;

      case 3:
        attachInterrupt(intNumA, isr3A, CHANGE);
        break;

      case 4:
        attachInterrupt(intNumA, isr4A, CHANGE);
        break;

      case 5:
        attachInterrupt(intNumA, isr5A, CHANGE);
        break;

      case 6:
        attachInterrupt(intNumA, isr6A, CHANGE);
        break;

      case 7:
        attachInterrupt(intNumA, isr7A, CHANGE);
        break;

      case 8:
        attachInterrupt(intNumA, isr8A, CHANGE);
        break;

      case 9:
        attachInterrupt(intNumA, isr9A, CHANGE);
        break;

      default:
        break;
    }
}


/**
 * Sets up the interrupt for the B pin of the encoder
 *
 * @param bPin The pin number of the encoder's B pin.
 */
void Encoder::bindInterruptB(int bPin) {
    //Set up the encoder interrupt pin
    int intNumB = digitalPinToInterrupt(bPin);
    switch (numInstances) {
      case 0:
        attachInterrupt(intNumB, isr0B, CHANGE);
        break;

      case 1:
        attachInterrupt(intNumB, isr1B, CHANGE);
        break;

      case 2:
        attachInterrupt(intNumB, isr2B, CHANGE);
        break;

      case 3:
        attachInterrupt(intNumB, isr3B, CHANGE);
        break;

      case 4:
        attachInterrupt(intNumB, isr4B, CHANGE);
        break;

      case 5:
        attachInterrupt(intNumB, isr5B, CHANGE);
        break;

      case 6:
        attachInterrupt(intNumB, isr6B, CHANGE);
        break;

      case 7:
        attachInterrupt(intNumB, isr7B, CHANGE);
        break;

      case 8:
        attachInterrupt(intNumB, isr8B, CHANGE);
        break;

      case 9:
        attachInterrupt(intNumB, isr9B, CHANGE);
        break;

      default:
        break;
    }
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
void Encoder::isr0A() {
  instance0->tick(true);
}
void Encoder::isr1A() {
  instance1->tick(true);
}
void Encoder::isr2A() {
  instance2->tick(true);
}
void Encoder::isr3A() {
  instance3->tick(true);
}
void Encoder::isr4A() {
  instance4->tick(true);
}
void Encoder::isr5A() {
  instance5->tick(true);
}
void Encoder::isr6A() {
  instance6->tick(true);
}
void Encoder::isr7A() {
  instance7->tick(true);
}
void Encoder::isr8A() {
  instance8->tick(true);
}
void Encoder::isr9A() {
  instance9->tick(true);
}


void Encoder::isr0B() {
  instance0->tick(false);
}
void Encoder::isr1B() {
  instance1->tick(false);
}
void Encoder::isr2B() {
  instance2->tick(false);
}
void Encoder::isr3B() {
  instance3->tick(false);
}
void Encoder::isr4B() {
  instance4->tick(false);
}
void Encoder::isr5B() {
  instance5->tick(false);
}
void Encoder::isr6B() {
  instance6->tick(false);
}
void Encoder::isr7B() {
  instance7->tick(false);
}
void Encoder::isr8B() {
  instance8->tick(false);
}
void Encoder::isr9B() {
  instance9->tick(false);
}
