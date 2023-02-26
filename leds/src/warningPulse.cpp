#include <FastLED.h>
#include "constants.h"
#include "warningPulse.h"

void WarningPulse::init(CRGB color) {
    pulseColor = color;
}

void WarningPulse::update() {    
    fill_solid(g_leds, NUM_LEDS, pulseColor);
    FastLED.show();
    FastLED.delay(1000);
    FastLED.clear();
    FastLED.delay(250);
}