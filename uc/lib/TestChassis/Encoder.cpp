/* 
 * Class for the Encoder.
 * This class provides functions for tracking encoder ticks and determining speed.
 * 
 */

#include "Encoder.h"

// for use by ISR routine
unsigned Encoder::instance_count = 0;
Encoder **Encoder::instances = nullptr;

// Create the static ISRs for each class
#define ISR_DEF(n) []() { Encoder::instances[n]->tick(); }
void (*Encoder::isrs[])() = {
    ISR_DEF(0),
    ISR_DEF(1),
    ISR_DEF(2),
    ISR_DEF(3),
    ISR_DEF(4),
    ISR_DEF(5),
    ISR_DEF(6),
    ISR_DEF(7),
    ISR_DEF(8),
    ISR_DEF(9)
};
#define NUM_ISRS sizeof(Encoder::isrs) / sizeof(*Encoder::isrs)

/**
 * @brief Constructor for the class.
 * 
 * @param trigPin - First encoder pin. Attached to interrupt.
 * @param ticksPerRotation - Number of encoder ticks per rotation.
 */
Encoder::Encoder(int trigPin, float ticksPerRotation) 
    : trigPin(trigPin), ticksPerRotation(ticksPerRotation), speedFilter(50, 50, 0.25)  { //speedFilter(300, 300, 0.025)
    tickConversion = 1000000ul * 60  / ticksPerRotation;
}

/**
 * @brief Initialize the encoder.
 */
void Encoder::init() {
    //Setup control pins
    pinMode(trigPin, INPUT);
    
    //Set up the encoder interrupt pin
    instance_count++;
    instances =
        (Encoder **)realloc(instances, instance_count * sizeof(Encoder *));
    instances[instance_count - 1] = this;
    attachInterrupt(digitalPinToInterrupt(trigPin), isrs[instance_count - 1], CHANGE);
    //initialize the timer
    lastTickTime = micros();
    lastEstTime = lastTickTime;
}

/**
 * Update the current speed estimate and return the filtered value.
 */
float Encoder::estimateSpeed() {
  // Calculate the speed if we got a tick. Otherwise assume 0.
  if (ticks != 0) {
    speed = ticks * (tickConversion / (lastTickTime - lastEstTime));

    // Reset things
    lastEstTime = lastTickTime;
    ticks = 0;
  } else {
    speed = 0;
  }
  
  // Invert if needed
  if (invertDir) {
    speed = -speed;
  }

  // Filter
  filteredSpeed = speedFilter.updateEstimate(speed);

  // Return the speed
  return filteredSpeed;
}

/**
 * Update the current speed estimate and return the filtered value.
 */
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

float Encoder::getPos() {
  return totTicks / ticksPerRotation;
}

void Encoder::resetPos() {
  totTicks = 0;
}

/**
 * Handle an encoder tick.
 */
void Encoder::tick() {
  lastTickTime = micros();

  ++ticks;
  ++totTicks;
}