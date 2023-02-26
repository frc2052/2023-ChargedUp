#include <FastLED.h>
#include "constants.h"
#include "pulse.h"

void Pulse::init(CRGB color) {
    pulseColor = color;
}

void Pulse::update() {    
    fill_solid(g_leds, NUM_LEDS, pulseColor);
    FastLED.show();
    FastLED.delay(2000);
    FastLED.clear();
    FastLED.delay(750);
}