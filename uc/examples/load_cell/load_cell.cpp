//////////////////////////////////////////////////////////////////////////////
// Description: Demo of running load cell.
/////////////////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <HX711.h>

#include "pins.h"

#define DOUT_PIN 5
#define SCK_PIN 6

HX711 loadcell;

// 2. Adjustment settings
const long LOADCELL_OFFSET = 7196;
const long LOADCELL_SCALE = 145568.18;

/**
 * Helper function for reading values from user.
 */
void flushSerial() {
    while (Serial.available()) Serial.read();
}

/**
 * Routine to calibrate load cell. Prompts given to user.
 */
void calibrate() {
    flushSerial();

    // Reset load cell
    loadcell.set_scale();

    // Tare the load cell
    flushSerial();
    Serial.println("[TARE] Remove all weight from scale and press ENTER.");
    while (!Serial.available())
        ;
    flushSerial();
    loadcell.tare();
    Serial.print("OFFSET -> ");
    Serial.println(loadcell.get_offset());

    // Scale the load cell
    flushSerial();
    Serial.println("[SCALE] Place a known weight on the scale and press ENTER.");
    while (!Serial.available())
        ;
    flushSerial();
    float value = loadcell.get_units(10);
    Serial.println("Enter the weight (in kg): ");
    float actValue = Serial.parseFloat();
    loadcell.set_scale(value / actValue);
    Serial.print("SCALE -> ");
    Serial.println(loadcell.get_scale());
}

/**
 * Get the current reading from a given loadcell.
 *
 * @param loadcell - load cell from which to read.
 * @param numRead - Number of readds to perform (default=1)
 * @param timeout - Time to wait for response (default=10ms)
 * @return The value using the load cell's calibration. NAN if no response.
 */
float readLoadCell(HX711 &loadcell, unsigned numRead = 1, unsigned timeout = 10) {
    if (loadcell.wait_ready_timeout(timeout)) {
        return loadcell.get_units(numRead);
    }
    return NAN;
}

//=====setup==============================
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(10000);

    // Initialize library
    loadcell.begin(LEFT_LC_DAT_PIN, LEFT_LC_CLK_PIN);
    loadcell.set_scale(LOADCELL_SCALE);
    loadcell.set_offset(LOADCELL_OFFSET);

    // Do calibration routine
    //! Comment out to use predefined values
    // calibrate();
}

//=====loop==============================
void loop() {
    static unsigned long time = 0;
    static float value;

    // Acquire reading
    Serial.print("Weight: ");
    time = millis();
    value = readLoadCell(loadcell);
    time = millis() - time;
    Serial.print(value, 2);
    Serial.print(", time: ");
    Serial.println(time);
    delay(1000);
}
