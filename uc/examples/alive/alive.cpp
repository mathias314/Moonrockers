/**
 * Real simple program to test if the Arduino is alive. Blink LED and print serial data. 
 * 
 * To run:
 *      pio run -t upload -c examples.ini -e alive
 */

#include <Arduino.h>


void setup() {
    Serial.begin(115200);

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
}

void loop() {
    Serial.println("BLONK");
    digitalWrite(PIN_LED, HIGH);
    delay(200);
    digitalWrite(PIN_LED, LOW);
    delay(1000);
}