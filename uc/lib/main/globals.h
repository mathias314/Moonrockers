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

// Control loop undate interval in ms
#define UPDATE_INTERVAL 2

// Motor output constraints
const float PIVOT_VELOCITY = 60; // In RPM
const float PIVOT_MAX_ANGLE = PI/2;
const float DRIVE_MAX_PWR = 0.4;
const float PIVOT_MAX_PWR = 1.0;

// Position PID control values
const float PIV_KP = 1.0, PIV_KI = 0.0, PIV_KD = 0.0, PIV_N = 1;   

// Proportional position + PID velocity control
const float PIV_POS_KP = PIVOT_VELOCITY / (10 * DEG_TO_RAD); // This is basically setting the angle where we start to ramp down.
// const float PIV_VEL_KP = 0.0025, PIV_VEL_KI = 0.005, PIV_VEL_KD = 0.0002, PIV_VEL_N = 3;
const float PIV_VEL_KP = 0.0025, PIV_VEL_KI = 0.01, PIV_VEL_KD = 0.0001, PIV_VEL_N = 3;

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
#define NUM_LEFT_LEDS 16
#define NUM_RIGHT_LEDS 16

#define LEFT_LED_PIN 4
#define RIGHT_LED_PIN 21


#endif