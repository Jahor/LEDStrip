/*
This example shows usage of LED Strip to control christmas lights consisting of 10 4-LED "pixels"
*/

#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>
#include <LEDStrip.h>


#define CIRCLE_SIZE 4
#define CIRCLES 10
#define LEDS (CIRCLES * CIRCLE_SIZE)

// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_RGB     Pixels are wired for RGB bitstream
//   NEO_GRB     Pixels are wired for GRB bitstream
//   NEO_KHZ400  400 KHz bitstream (e.g. FLORA pixels)
//   NEO_KHZ800  800 KHz bitstream (e.g. High Density LED strip)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LEDS, 10, NEO_GRB + NEO_KHZ800);

LEDStrip whole;

LEDStrip circles[CIRCLES];
LEDStrip half[2][CIRCLES];

LEDStrip* pCircles[CIRCLES];
LEDStrip* pHalf[2][CIRCLES];
LEDStrip* p2[2][CIRCLES / 2 + 1];
LEDStrip* p3[3][CIRCLES / 3 + 1];

LEDStrip cl;
LEDStrip clh[2];

LEDStrip cl2[2];
LEDStrip cl3[3];
LEDStrip clp[CIRCLE_SIZE];

LEDStrip* parts[1 /* whole */ + CIRCLES * 3 + 3 /* cl, clh1, clh2 */ + 2 /* %2 */ + 3 /* %3 */] = {&whole};

unsigned long patternTime = 0;
byte autoSwitch = 1;

typedef void (*Pattern)(void);

#define COLORED_FUNCTION(FUNC) FUNC ## InRed, FUNC ## InOrange, FUNC ## InYellow, FUNC ## InGreen, FUNC ## InSkyBlue, FUNC ## InBlue, FUNC ## InMagenta

#define COLORED_FUNCTION_DEFINITION(F) \
void F ## InRed()     { F(RED);     }\
void F ## InOrange()  { F(ORANGE);  }\
void F ## InYellow()  { F(YELLOW);  }\
void F ## InGreen()   { F(GREEN);   }\
void F ## InSkyBlue()   { F(SKY_BLUE);   }\
void F ## InBlue()    { F(BLUE);    }\
void F ## InMagenta() { F(MAGENTA); }

#define COLORED_FUNCTION_DEFINITION_FLAME(F) \
void F ## InRed()     { F(RED_FLAME);     }\
void F ## InOrange()  { F(ORANGE_FLAME);  }\
void F ## InYellow()  { F(YELLOW_FLAME);  }\
void F ## InGreen()   { F(GREEN_FLAME);   }\
void F ## InSkyBlue() { F(SKY_BLUE_FLAME); }\
void F ## InBlue()    { F(BLUE_FLAME);    }\
void F ## InMagenta() { F(MAGENTA_FLAME); }

COLORED_FUNCTION_DEFINITION(rotateAll)
COLORED_FUNCTION_DEFINITION(run)
COLORED_FUNCTION_DEFINITION(comet)
COLORED_FUNCTION_DEFINITION(randomCircle)
COLORED_FUNCTION_DEFINITION(sinColor)
COLORED_FUNCTION_DEFINITION_FLAME(flame)

Pattern patterns[] = {rainbowCycle, rainbowWipe, rainbowRun,
redAndBlueHalfsGlowing, 
redAndBlue, 
COLORED_FUNCTION(flame),
COLORED_FUNCTION(sinColor), 
COLORED_FUNCTION(randomCircle), 
COLORED_FUNCTION(rotateAll),
COLORED_FUNCTION(run),
COLORED_FUNCTION(comet), 
allWhites};

signed int patternN = 0;

void setup() {
  NeoStripInit(&whole, &strip, 0, LEDS);
  byte c2[2] = {0, 0}, c3[3] = {0, 0, 0};
  
  for (int i = 0; i < CIRCLES; i++) {
    NeoStripInit(&circles[i], &strip, i * CIRCLE_SIZE, CIRCLE_SIZE);
    pCircles[i] = &circles[i];
    parts[1 + i * 3 + 0] = &circles[i];
    
    NeoStripInit(&half[0][i], &strip, i * CIRCLE_SIZE, CIRCLE_SIZE / 2);
    parts[1 + i * 3 + 1] = &half[0][i];
    pHalf[0][i] = &half[0][i];
    
    NeoStripInit(&half[1][i], &strip, i * CIRCLE_SIZE + CIRCLE_SIZE / 2, CIRCLE_SIZE / 2);
    parts[1 + i * 3 + 2] = &half[1][i];
    pHalf[1][i] = &half[1][i];
    
    byte s2 = i % 2;
    p2[s2][c2[s2]++] = &circles[i];
    byte s3 = i % 3;
    p3[s3][c3[s3]++] = &circles[i];
  }
  
  MetaStripInit(&cl, pCircles, CIRCLES);
  parts[1 + CIRCLES * 3 + 0] = &cl;
  MetaStripInit(&clh[0], pHalf[0], CIRCLES);
  parts[1 + CIRCLES * 3 + 1] = &clh[0];
  MetaStripInit(&clh[1], pHalf[1], CIRCLES);
  parts[1 + CIRCLES * 3 + 2] = &clh[1];

  MetaStripInit(&cl2[0], p2[0], c2[0]);
  parts[1 + CIRCLES * 3 + 3] = &cl2[0];
  MetaStripInit(&cl2[1], p2[1], c2[1]);
  parts[1 + CIRCLES * 3 + 4] = &cl2[1];

  MetaStripInit(&cl3[0], p3[0], c3[0]);
  parts[1 + CIRCLES * 3 + 5] = &cl3[0];
  MetaStripInit(&cl3[1], p3[1], c3[1]);
  parts[1 + CIRCLES * 3 + 6] = &cl3[1];
  MetaStripInit(&cl3[2], p3[2], c3[2]);
  parts[1 + CIRCLES * 3 + 7] = &cl3[2];
  
  pinMode(13, OUTPUT);
  
  strip.begin();
  strip.show();
  setPattern(EEPROM.read(0), 0);
}

void setPattern(signed int n, byte user) {
  if (n < 0) {
    n = ARRAY_SIZE(patterns) - 1;
    digitalWrite(13, HIGH);    
  }
  if(n >= ARRAY_SIZE(patterns)) {
    n = 0;
  }
  Color transparent;
  CONFIGURE_COLOR_TRANSPARENT(transparent);

  for(int i = 0; i < ARRAY_SIZE(parts); i++) {
    LEDStripSetColor(parts[i], &transparent);
  }
  patterns[n]();
  patternN = n;
  EEPROM.write(0, patternN);
  if(user) {
    EEPROM.write(0, patternN);
    autoSwitch = 0;
  }
  patternTime = 0;
}

void rainbowCycle() {
  Color color;
  CONFIGURE_COLOR_SIMPLE(color, rainbowCycleFrame, 1, TRANSPARENT);
  LEDStripSetColor(&cl, &color);
}

void simpleRainbowCycle() {
  Color color;
  CONFIGURE_COLOR_SIMPLE(color, rainbowSimpleFrame, 1, TRANSPARENT);
  LEDStripSetColor(&cl, &color);
}

void redAndBlueHalfsGlowing() {
  Color color;
  CONFIGURE_COLOR_SIMPLE(color, colorGlowFrame, 50, RED);
  LEDStripSetColor(&clh[0], &color);
  CONFIGURE_COLOR(color, colorGlowFrame, 64, 50, BLUE, 0,0,0,0);
  LEDStripSetAllPixels(&clh[1], &color);
  //cube1.setColor(Color(rainbowWholeFrame, 16));
 // cl.setColor(Color(rainbowSimpleFrame, 16));
}

void randomCircle(const RGB_t rgb) {
  Color color;
  CONFIGURE_COLOR_SIMPLE(color, randomPixelFrame, 10, rgb);
  LEDStripSetColor(&cl, &color);
}

void run(const RGB_t rgb) {
  Color color;
  CONFIGURE_COLOR_SIMPLE_WP(color, colorRunFrame, 10, rgb, 2, 0);
  LEDStripSetColor(&cl, &color);
}

void redAndBlue() {
  Color color;
  CONFIGURE_COLOR_RGB(color, RED);
  LEDStripSetColor(&cl2[0], &color);
  CONFIGURE_COLOR_RGB(color, BLUE);
  LEDStripSetColor(&cl2[1], &color);
}

void redGreenAndBlueWipes() {
  Color color;
  CONFIGURE_COLOR_SIMPLE(color, colorWipeFrame, 10, RED);
  LEDStripSetColor(&cl3[0], &color);
  CONFIGURE_COLOR_SIMPLE(color, colorWipeFrame, 10, GREEN);
  LEDStripSetColor(&cl3[1], &color);
  CONFIGURE_COLOR_SIMPLE(color, colorWipeFrame, 10, BLUE);
  LEDStripSetColor(&cl3[2], &color);
}

void rotateAll(const RGB_t rgb) {
  Color color;
  CONFIGURE_COLOR_SIMPLE_WP(color, colorRunFrame, 20, rgb, 1, 0);
  LEDStripSetAllPixels(&cl, &color);
}

void redGreenAndBlueRun() {
  Color color;
  CONFIGURE_COLOR_SIMPLE_WP(color, colorRunFrame, 20, RED, 1, 0);
  LEDStripSetColor(&cl3[0], &color);
  CONFIGURE_COLOR_SIMPLE_WP(color, colorRunFrame, 20, GREEN, 1, 0);
  LEDStripSetColor(&cl3[1], &color);
  CONFIGURE_COLOR_SIMPLE_WP(color, colorRunFrame, 20, BLUE, 1, 0);
  LEDStripSetColor(&cl3[2], &color);
}

void flame(const RGB_t rgb) {
  Color color;
  CONFIGURE_COLOR_SIMPLE_WP(color, flameFrame, 15, rgb, 1, 0);
  LEDStripSetAllPixels(&cl, &color);
}

void rainbowWipe() {
  Color color;
  CONFIGURE_COLOR_SIMPLE_WP(color, rainbowSimpleWipeFrame, 100, TRANSPARENT, 25, 0);
  LEDStripSetColor(&cl, &color);
}

void rainbowRun() {
  Color color;
  CONFIGURE_COLOR_SIMPLE_WP(color, rainbowSimpleRunFrame, 100, TRANSPARENT, 50, 1);
  LEDStripSetColor(&cl, &color);
}

void sinColor(const RGB_t rgb) {
  Color color;
  CONFIGURE_COLOR_SIMPLE_WP(color, colorSinFrame, 10, rgb, 0, 0);
  LEDStripSetColor(&cl, &color);
}

void comet(const RGB_t rgb) {
  Color color;
  CONFIGURE_COLOR_SIMPLE_WP(color, colorCometFrame, 5, rgb, 0, 0);
  LEDStripSetColor(&cl, &color);
}

void allWhites() {
  Color color;
  CONFIGURE_COLOR_RGB(color, WHITE);
  LEDStripSetColor(&whole, &color);
}

#define DELAY 16

void loop() {
  static byte pressed = 0;
  long start = millis();
  for(int i = 0; i < ARRAY_SIZE(parts); i++) {
    LEDStripAnimate(parts[i]);
  }
  strip.show();
  long t = millis() - start;  
  if (t < DELAY) {
    delay(DELAY - t);
  }
  
  if (autoSwitch) {
    patternTime += DELAY;
    if (patternTime > 30000) {
      setPattern(random() % ARRAY_SIZE(patterns), 0);
    }
  }
} 

