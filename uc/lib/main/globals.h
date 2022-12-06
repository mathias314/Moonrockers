/**
 * @file globals.h
 * 
 * Definitions of various pins and constants to be used across ENVs.
 * 
 */
#ifndef __MOONBOT_GLOBALS__
#define __MOONBOT_GLOBALS__

#ifdef ESP32
    // CAN Bus pins
    #define CAN_INT_PIN 9
    #define CAN_CS_PIN 5

    // Pivot sensor pins
    #define FL_PIVOT_SENSOR_PIN 32
    #define FR_PIVOT_SENSOR_PIN 33
    #define BR_PIVOT_SENSOR_PIN 25
    #define BL_PIVOT_SENSOR_PIN 26
#else
    // CAN Bus pins
    #define CAN_INT_PIN 9
    #define CAN_CS_PIN 8

    // Pivot sensor pins
    #define FL_PIVOT_SENSOR_PIN A0
    #define FR_PIVOT_SENSOR_PIN A1
    #define BR_PIVOT_SENSOR_PIN A2
    #define BL_PIVOT_SENSOR_PIN A3
#endif

// Pivot calibrations
const unsigned ANGLE_SENSOR_CALIBRATIONS[4][3] = {
    {840, 481, 120},  // Front left
    {835, 480, 94}, // Front right
    {804, 464, 101},  // Back left
    {860, 518, 173}  // Back right
};
const float PIV_KP = 1.0, PIV_KI = 0.0, PIV_KD = 0.0, PIV_N = 1;

// CAN bus IDs
#define FL_MTR_ID 4
#define FR_MTR_ID 5
#define CL_MTR_ID 6
#define CR_MTR_ID 2
#define BR_MTR_ID 1
#define BL_MTR_ID 3

#define FL_PIVOT_MTR_ID 8
#define FR_PIVOT_MTR_ID 10
#define BL_PIVOT_MTR_ID 7
#define BR_PIVOT_MTR_ID 9

#define PLUNGE_MTR_ID 5
#define DELIVERY_MTR_ID 6

// LEDs
#define NUM_LEFT_LEDS 5
#define NUM_RIGHT_LEDS 5

#define LEFT_LED_PIN 7
#define RIGHT_LED_PIN 6


#endif