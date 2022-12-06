//////////////////////////////////////////////////////////////////////////////
// Description: Demo of controlling pretty LEDs.
/////////////////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "globals.h"

Adafruit_NeoPixel pixels(NUM_BACK_LEDS, BACK_LED_PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 500


void setup() {
    Serial.begin(115200);
  pixels.begin();
}

void loop() {
    Serial.println("Setting");
  pixels.clear();

  for(int i=0; i<NUMPIXELS; i++) {

    pixels.setPixelColor(i, pixels.Color(0, 150, 0));
    pixels.show();
    delay(DELAYVAL);
  }
}