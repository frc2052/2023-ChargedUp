#include <Arduino.h>
#include "FastLED.h"
#include "constants.h"
#include "pulse.h"
#include <WiFi.h>

int PIN_ONE = 12;
int PIN_TWO = 14;
int PIN_FOUR = 27;
int PIN_EIGHT = 26;
int PIN_SIXTEEN = 25;
int PIN_THIRTY_TWO = 33;
int PIN_SIXTY_FOUR = 32;
int PIN_ONE_HUNDRED_TWENTY_EIGHT = 35;

int currentCode = -1;

unsigned long startMillis;
unsigned long currentMillis;

CRGB g_leds[NUM_LEDS]; //create our LED array object for all our LEDs
Pulse pulse = Pulse();

void setup() {
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  FastLED.addLeds<CHIP_SET, DATA_PIN, COLOR_ORDER>(g_leds, NUM_LEDS);
  FastLED.setMaxPowerInVoltsAndMilliamps(VOLTS, MAX_AMPS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();

        
}

  void cone()
  {
    pulse.init(CRGB(255, 175, 0), 1000, 500);
    pulse.update();
  }

  void cube()
  {
    pulse.init(CRGB(128, 0, 128), 1000, 500);
    pulse.update();    
  }

  void redDisabled()
  {
    pulse.init(CRGB(255, 0, 0), 1000, 500);
    pulse.update();    
  }

  void blueDisabled()
  {
    pulse.init(CRGB(0, 0, 255), 1000, 500);
    pulse.update();
  }

  void noAutoWarning()
  {
    pulse.init(CRGB(255, 255, 255), 1000, 500);
    pulse.update();
  }

  void currentLimiting()
  {
    pulse.init(CRGB(255, 172, 28), 500, 250);
    pulse.update();
  }

void loop() {
  int code = 0;
  if (digitalRead(PIN_ONE))
  {
    code += 1;
  }
  if (digitalRead(PIN_TWO))
  {
    code += 2;
  }
  if (digitalRead(PIN_FOUR))
  {
    code += 4;
  }
  if (digitalRead(PIN_EIGHT))
  {
    code += 8;
  }
  if (digitalRead(PIN_SIXTEEN))
  {
    code += 16;
  }
  if (digitalRead(PIN_THIRTY_TWO))
  {
    code += 32;
  }
  if (digitalRead(PIN_SIXTY_FOUR))
  {
    code += 64;
  }
  if (digitalRead(PIN_ONE_HUNDRED_TWENTY_EIGHT))
  {
    code += 128;
  }

  if (currentCode != code)
  {
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
      case 6:
        currentLimiting();
        break;
      default:
        FastLED.clear();
        break;
    }
  }
  currentCode = code;
  if (code >=1 && code <=6)
  {
    pulse.update();
  }
  else
  {
    FastLED.show();
  }
}