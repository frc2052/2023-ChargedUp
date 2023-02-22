#include <Arduino.h>
#include "FastLED.h"
#include "constants.h"
#include "pulse.h"
#include "warningPulse.h"

int PIN_ONE = 32;
int PIN_TWO = 33;
int PIN_FOUR = 25;
int PIN_EIGHT = 26;
int PIN_SIXTEEN = 27;

CRGB g_leds[NUM_LEDS]; //create our LED array object for all our LEDs
Pulse pulse = Pulse();
WarningPulse warningPulse = WarningPulse();

void setup() {
  FastLED.addLeds<CHIP_SET, DATA_PIN, COLOR_ORDER>(g_leds, NUM_LEDS);
  FastLED.setMaxPowerInVoltsAndMilliamps(VOLTS, MAX_AMPS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();
}

  void cone()
  {
    pulse.init(CRGB(255, 215, 0));
    pulse.update();
  }

  void cube()
  {
    pulse.init(CRGB(128, 0, 128));
    pulse.update();    
  }

  void redDisabled()
  {
    pulse.init(CRGB(255, 0, 0));
    pulse.update();    
  }

  void blueDisabled()
  {
    pulse.init(CRGB(0, 0, 255));
    pulse.update();
  }

  void noAutoWarning()
  {
    warningPulse.init(CRGB(255, 255, 255));
    warningPulse.update();
  }

void loop() {
  int code = 0;
  if (digitalRead(PIN_ONE))
  {
    code = code + 1;
  }
  if (digitalRead(PIN_TWO))
  {
    code = code + 2;
  }
  if (digitalRead(PIN_FOUR))
  {
    code = code + 4;
  }
  if (digitalRead(PIN_EIGHT))
  {
    code = code + 8;
  }
  if (digitalRead(PIN_SIXTEEN))
  {
    code = code + 16;
  }

  switch (code) {
    case 1:
      cone();
      break;
    case 2:
      cube();
      break;
    case 3:
      redDisabled();
      break;
    case 4:
      blueDisabled();
      break;
    case 5:
      noAutoWarning();
      break;
    default:
      FastLED.clear();
      break;
  }
}