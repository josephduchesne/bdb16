#include <Arduino.h>

// based on https://github.com/FastLED/FastLED/blob/master/examples/Fire2012/Fire2012.ino

#include <FastLED.h>
#define NUM_LEDS 16

CRGB leds[NUM_LEDS];

#define COOLING  55
#define SPARKING 55
#define FRAMES_PER_SECOND 60
bool gReverseDirection = false;
uint8_t gHue = 0; // rotating "base color" used by many of the patterns


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  FastLED.addLeds<WS2811, PIN_RGB, GRB>(leds, NUM_LEDS);
  //FastLED.setBrightness(5 );
}

void confetti() 
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
}


void Fire2012()
{
  // Array of temperature readings at each simulation cell
  static uint8_t heat[NUM_LEDS];

  // Step 1.  Cool down every cell a little
  for( int i = 0; i < NUM_LEDS; i++) {
    heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / NUM_LEDS) + 2));
  }

  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for( int k= NUM_LEDS - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
  }
  
  // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
  if( random8() < SPARKING ) {
    int y = random8(7);
    heat[y] = qadd8( heat[y], random8(160,255) );
  }

  // Step 4.  Map from heat cells to LED colors
  for( int j = 0; j < NUM_LEDS; j++) {
    CRGB color = HeatColor( heat[j]);
    int pixelnumber;
    if( gReverseDirection ) {
      pixelnumber = (NUM_LEDS-1) - j;
    } else {
      pixelnumber = j;
    }
    leds[pixelnumber] = color;
  }
}

void loop() {
  digitalWrite(LED_BUILTIN, (millis()%1000) > 500);

  EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow


  //Fire2012(); // run simulation frame
  bpm();
  FastLED.show(); // display this frame
  FastLED.delay(1000 / FRAMES_PER_SECOND);
}
