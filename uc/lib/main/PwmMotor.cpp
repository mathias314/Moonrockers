/**
 * @file PwmMotor.cpp
 * @brief Class to interface with a PWM motor controller. This uses the hardware PWM 
 * output of the Arduino 33 IOT using control timer #1.
 * 
 * THIS WILL NOT RUN ON OTHER BOARDS!!!   
 */
#include "PwmMotor.h"

unsigned PwmMotor::channelCount = 0;

/**
 * Constructor for the class.
 * 
 * @param pin - Pin to use for the output. Must be PWM.
 */
PwmMotor::PwmMotor(unsigned pin, unsigned channel) : pin(pin), channel(channel) {
}

/**
 * Initialize the PWM output to the controller.
 */
void PwmMotor::init() {
    pinMode(pin, OUTPUT);

    // Disable the TCC1 counter
    TCC1->CTRLA.bit.ENABLE = 0;
    while (TCC1->SYNCBUSY.bit.ENABLE);

    // Set up the clock source
    // Feed GCLK0 at 48MHz to TCC0 and TCC1
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Enable GCLK0 as a clock source
                        GCLK_CLKCTRL_GEN_GCLK0 |     // Select GCLK0 at 48MHz
                        GCLK_CLKCTRL_ID_TCC0_TCC1;   // Route GCLK0 to TCC0 and TCC1
    while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

    // Enable the port multiplexer for pin
    EPortType port = g_APinDescription[pin].ulPort;
    uint32_t pinRef = g_APinDescription[pin].ulPin;
    PORT->Group[port].PINCFG[pinRef].bit.PMUXEN = 1;

    // D7 is on EVEN port pin PA06 and TCC1/WO[0] channel 0 is on peripheral E
    uint32_t muxVal = (pinRef & 1 ? PORT_PMUX_PMUXO_E : PORT_PMUX_PMUXE_E);
    PORT->Group[port].PMUX[pinRef >> 1].reg |= muxVal;

    // Set up the wavegen
    // Normal (single slope) PWM operation: timer countinuously counts up to PER register value and then is reset to 0
    TCC1->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;          // Setup single slope PWM on TCC1
    while (TCC1->SYNCBUSY.bit.WAVE);                 // Wait for synchronization

    // Set the frequency for the output
    // f = CLCK / (prescaler*PER + 1)
    TCC1->CTRLA.reg = TC_CTRLA_PRESCALER_DIV64 |      // Set prescaler  
                    TC_CTRLA_PRESCSYNC_PRESC;         // Set the reset/reload to trigger on prescaler clock

    TCC1->PER.reg = 15000;                            // Set the frequency of the PWM on TCC1
    while (TCC1->SYNCBUSY.bit.PER);                  // Wait for synchronization

    // Set the initial duty cycle
    TCC1->CC[channel].reg = TCC1->PER.reg*0.5;
    // Wait for not busy
    bool busy = true;
    while (busy) {
        switch (channel) {
            case 0:
                busy = TCC1->SYNCBUSY.bit.CC0;
                break;
            case 1:
                busy = TCC1->SYNCBUSY.bit.CC1;
                break;
            case 2:
                busy = TCC1->SYNCBUSY.bit.CC2;
                break;
            case 3:
                busy = TCC1->SYNCBUSY.bit.CC3;
                break;
            default:
                busy = false;
        }
    }
  
    // Enable the TCC1 counter
    TCC1->CTRLA.bit.ENABLE = 1;
    while (TCC1->SYNCBUSY.bit.ENABLE);
}

/**
 * Set whether the motor should run in inverted mode.
 * 
 * @param invert - (optional) Whether the motor direction should be inverted. Default to true. 
 */
void PwmMotor::setInverted(bool invert) {
    this->inverted = invert;
    setPower(power);
}

/**
 * Set the output value.
 * 
 * @param power - percent output. Float value in [-1.0, 1.0].
 */
void PwmMotor::setPower(float newPower) {
    power = constrain(newPower, -1.0, 1.0);
    if (inverted) {
        power = -power;
    }
    
    // Calculate the duty cycle 
    unsigned maxVal = TCC1->PER.reg;
    unsigned range = (maxVal+1) * 0.05;
    unsigned duty = (power+1) * range / 2 + range;

    // Set the value
    TCC1->CCB[channel].reg = duty;

    // Wait for not busy
    bool busy = true;
    while (busy) {
        switch (channel) {
            case 0:
                busy = TCC1->SYNCBUSY.bit.CCB0;
                break;
            case 1:
                busy = TCC1->SYNCBUSY.bit.CCB1;
                break;
            case 2:
                busy = TCC1->SYNCBUSY.bit.CCB2;
                break;
            case 3:
                busy = TCC1->SYNCBUSY.bit.CCB3;
                break;
            default:
                busy = false;
        }
    }
}

/**
 * Get the current output value.
 */
float PwmMotor::getPower() {
    return power;
}