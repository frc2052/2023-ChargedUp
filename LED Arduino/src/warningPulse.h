#ifndef WARNINGPULSE_H
#define WARNINGPULSE_H

#include <FastLED.h>

extern CRGB g_leds[];

class WarningPulse {
    private:
        CRGB pulseColor = CRGB(0, 0, 255); //initialize to white

    public:
        void init(CRGB color);
        void update();
};

#endif