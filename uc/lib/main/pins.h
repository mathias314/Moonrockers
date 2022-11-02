/**
 * @file pins.h
 * 
 * Definitions of various pins and constants to be used across ENVs.
 * 
 */
#ifndef __MOONBOT_PINS__
#define __MOONBOT_PINS__

// CAN Bus pins
#define CAN_INT_PIN 9
#define CAN_CS_PIN 8

// CAN bus IDs
#define FL_MTR_ID 1
#define FR_MTR_ID 2
#define BR_MTR_ID 3
#define BL_MTR_ID 4
#define PLUNGE_MTR_ID 5
#define DELIVERY_MTR_ID 6

// Other motor pins
#define GATE_PIN A2
#define GATE_PWM_CHANNEL 1   // TCC1 output channel
#define CONVEYOR_PIN A3
#define CONVEYOR_PWM_CHANNEL 0   // TCC1 output channel

// LEDs
#define FRONT_LED_PIN 7
#define BACK_LED_PIN 6

#define NUM_FRONT_LEDS 5
#define NUM_BACK_LEDS 5

// Load cell pins
#define LEFT_LC_DAT_PIN A0
#define LEFT_LC_CLK_PIN A1
#define RIGHT_LC_DAT_PIN A6
#define RIGHT_LC_CLK_PIN A7

const long LOADCELL_OFFSET = 7196;
const long LOADCELL_SCALE = 145568.18;


#endif