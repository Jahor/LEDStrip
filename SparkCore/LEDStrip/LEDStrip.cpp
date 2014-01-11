#include "LEDStrip.h"
#include <string.h>
#include <stdlib.h>

#define MAX_WHITENESS 15
#define MAX_VALUE 17

#define SIXTH_HUE 16

#define THIRD_HUE (SIXTH_HUE * 2)
#define HALF_HUE (SIXTH_HUE * 3)
#define TWO_THIRDS_HUE (SIXTH_HUE * 4)
#define FIVE_SIXTHS_HUE (SIXTH_HUE * 5)
#define FULL_HUE (SIXTH_HUE * 6)

#ifdef SMALL_DIM
#define DIM(c, v) ((c) / (v)) 
#else 
#define DIM(c, v) ((uint8_t)(((uint16_t) (c)) * (v) / 255))
#endif

#ifndef NULL
#define NULL 0
#endif

#define RGBD(original, value) RGB(DIM(original.r, value), DIM(original.g, value), DIM(original.b, value))

#define WRAP(k, l) (((k) >= (l)) ? (k) - (l) : (k))

static const RGB_t rainbowColors[] = {RED, ORANGE, YELLOW, GREEN, SKY_BLUE, BLUE, MAGENTA};

static RGB_t hsv2rgb(HSV_t hsv) {
    if (hsv.v == 0) return BLACK;
    
    uint8_t high = hsv.v * MAX_WHITENESS;//channel with max value
    if (hsv.s == 0) return RGB(high, high, high);
    
    uint8_t W = MAX_WHITENESS - hsv.s;
    uint8_t low = hsv.v * W;//channel with min value
    uint8_t rising = low;
    uint8_t falling = high;
    
    uint8_t h_after_sixth = hsv.h % SIXTH_HUE;
    if (h_after_sixth > 0) {//not at primary color? ok, h_after_sixth = 1..sixth_hue - 1
        uint8_t z = hsv.s * (uint8_t)(hsv.v * h_after_sixth) / SIXTH_HUE;
        rising += z;
        falling -= z + 1;//it's never 255, so ok
    }
    
    uint8_t H = hsv.h;
    while (H >= FULL_HUE) H -= FULL_HUE;
    
    if (H < SIXTH_HUE) return RGB(high, rising, low);
    if (H < THIRD_HUE) return RGB(falling, high, low);
    if (H < HALF_HUE) return RGB(low, high, rising);
    if (H < TWO_THIRDS_HUE) return RGB(low, falling, high);
    if (H < FIVE_SIXTHS_HUE) return RGB(rising, low, high);
    return RGB(high, low, falling);
}


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
static RGB_t wheel(uint8_t WheelPos) {
    if(WheelPos < 85) {
        return RGB(WheelPos * 3, 255 - WheelPos * 3, 0);
    } else if(WheelPos < 170) {
        WheelPos -= 85;
        return RGB(255 - WheelPos * 3, 0, WheelPos * 3);
    } else {
        WheelPos -= 170;
        return RGB(0, WheelPos * 3, 255 - WheelPos * 3);
    }
}

bool areColorsEqual(Color* lhs, Color* rhs) {
	return lhs->frameGenerator               == rhs->frameGenerator &&
				 lhs->frameParameters.color.r      == rhs->frameParameters.color.r &&
				 lhs->frameParameters.color.g      == rhs->frameParameters.color.g &&
				 lhs->frameParameters.color.b      == rhs->frameParameters.color.b &&
				 lhs->frameParameters.color.alpha  == rhs->frameParameters.color.alpha &&
				 lhs->frameParameters.parameter1   == rhs->frameParameters.parameter1 &&
				 lhs->frameParameters.parameter2   == rhs->frameParameters.parameter2 &&
				 lhs->frameParameters.divider      == rhs->frameParameters.divider &&
				 lhs->frameParameters.inverted     == rhs->frameParameters.inverted &&
				 lhs->frameParameters.transparent  == rhs->frameParameters.transparent &&
				 lhs->startFrame                   == rhs->startFrame;
}

void LEDStripAnimate(LEDStrip* strip) {
  strip->animate(strip);
}

void LEDStripAnimateBase(LEDStrip* strip) {
	if (strip->color.frameGenerator) {
		strip->color.frameGenerator(strip, &strip->frameNumber, &strip->color.frameParameters);
	}
}

void LEDStripSetPixel(LEDStrip* strip, Length pixelN, Color* color, uint8_t inverted) {
    if (pixelN < strip->length) {
        Length n;
        if (strip->inverted ^ inverted) {
            n = (strip->length - pixelN - 1);
        } else {
            n = pixelN;
        }
        strip->setPixelColor(strip, n, color);
    }
}
#ifdef ALLOW_GET_PIXEL
void LEDStripGetPixel(LEDStrip* strip, Length pixelN, Color* color, uint8_t inverted) {
    if (pixelN < strip->length) {
        Length n;
        if (strip->inverted ^ inverted) {
            n = (strip->length - pixelN - 1);
        } else {
            n = pixelN;
        }
        strip->getPixelColor(strip, n, color);
    }
}
#endif

void LEDStripSetPixelRange(LEDStrip* strip, Length from, Length to, Color* c, uint8_t inverted) {
    if (to >= from && to < strip->length) {
        Length f;
        Length l = to - from + 1;
        if (strip->inverted ^ inverted) {
            f = (strip->length - to);
        } else {
            f = from;
        }
        for (Length i = 0; i < l; i++) {
            strip->setPixelColor(strip, f + i, c);
        }
    }
}

void LEDStripSetAllPixels(LEDStrip* strip, Color* c) {
	LEDStripSetPixelRange(strip, 0, strip->length - 1, c, false);
}

void LEDStripClear(LEDStrip* strip) {
  Color transparent;
  CONFIGURE_COLOR_TRANSPARENT(transparent);
  LEDStripSetColor(strip, &transparent);
}

void LEDStripSetColor(LEDStrip* strip, Color* color) {
  if (!areColorsEqual(color, &strip->color)) {
    memcpy(&strip->color, color, sizeof(Color));
    if (color->startFrame != MAX_FRAME_COUNT) {
      strip->frameNumber.i = color->startFrame;
      strip->frameNumber.f = 0;
    }
  }
}

void LEDStripInit(LEDStrip* strip, const Length length) {
  strip->length = length;
  strip->animate = LEDStripAnimateBase;
}

static void PartialStripSetPixelColor(LEDStrip* strip, Length pixelN, Color* color) {
    strip->baseStrip->setPixelColor(strip->baseStrip, strip->position + pixelN, color);
}

#ifdef ALLOW_GET_PIXEL
static void PartialStripGetPixelColor(LEDStrip* strip, Length pixelN, Color* color) {
    strip->baseStrip->getPixelColor(strip->baseStrip, strip->position + pixelN, color);
}
#endif

void PartialStripInit(LEDStrip* strip, LEDStrip* baseStrip, const Length position, const Length length) {
  LEDStripInit(strip, length);
  strip->baseStrip = baseStrip;
  strip->position = position;
  strip->setPixelColor = PartialStripSetPixelColor;
  #ifdef ALLOW_GET_PIXEL
  strip->getPixelColor = PartialStripGetPixelColor;
#endif
}

static void SparkStripSetPixelColor(LEDStrip* strip, Length pixelN, Color* color) {
    RGB_t rgb = color->frameParameters.color;
    if (rgb.alpha) {
      strip->pixels[pixelN * 3] = rgb.g;
      strip->pixels[pixelN * 3 + 1] = rgb.r;
      strip->pixels[pixelN * 3 + 2] = rgb.b;
    }
}

#ifdef ALLOW_GET_PIXEL
static void SparkStripGetPixelColor(LEDStrip* strip, Length pixelN, Color* color) {
    CONFIGURE_COLOR_RGB((*color), RGB(strip->pixels[pixelN * 3 + 1], strip->pixels[pixelN * 3], strip->pixels[pixelN * 3 + 2]);
}
#endif


static void SparkStripOutputInit(LEDStrip* strip) {
 GPIO_TypeDef *gpio_port = strip->gpio;
 uint16_t gpio_pin = strip->pinMask;

 GPIO_InitTypeDef GPIO_InitStructure;

 if (gpio_port == GPIOA )
 {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
 }
 else if (gpio_port == GPIOB )
 {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
 }

 GPIO_InitStructure.GPIO_Pin = gpio_pin;

 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(gpio_port, &GPIO_InitStructure);
}

void SparkStripInit(LEDStrip* strip, uint8_t* pixels, GPIO_TypeDef* gpio, const uint16_t pinMask, const Length length) {
  LEDStripInit(strip, length);
  strip->pixels = pixels;
  strip->pinMask = pinMask;
  strip->gpio = gpio;

  SparkStripOutputInit(strip);
  
  strip->setPixelColor = SparkStripSetPixelColor;
  #ifdef ALLOW_GET_PIXEL
  strip->getPixelColor = SparkStripGetPixelColor;
#endif
}

void SparkStripShow(LEDStrip* strip) {
  GPIO_TypeDef* gpio = strip->gpio;
  const uint16_t pinMask = strip->pinMask;
  
  volatile uint32_t 
    c,    // 24-bit pixel color
    mask; // 8-bit mask
  volatile uint16_t i = strip->length; // Output loop counter
  volatile uint8_t
    j,              // 8-bit inner loop counter
   *ptr = strip->pixels,   // Pointer to next byte
    g,              // Current green byte value
    r,              // Current red byte value
    b;              // Current blue byte value
  mask = 0x1000000; // reset the mask, start 1 higher than
  g = *ptr++;   // Next green byte value
  r = *ptr++;   // Next red byte value
  b = *ptr++;   // Next blue byte value
  __disable_irq();
  while(i) { // While bytes left...
    c = ((uint32_t)g << 16) | ((uint32_t)r <<  8) | b; // Pack the next 3 bytes to keep timing tight
    for (j=0; j<23; j++) { // iterate through 24-bits of next pixel, MSB to LSB.
      if (c & (mask >>= 1)) { // mask shifts first, then & with c
        // 700ns HIGH (meas. 694ns)
        gpio->BSRR = pinMask; // HIGH
        asm volatile(
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          ::: "r0", "cc", "memory");
        // 600ns LOW (meas. 598ns)
        gpio->BRR = pinMask; // LOW 
      } else {
        // 350ns HIGH (meas. 360ns)
        gpio->BSRR = pinMask; // HIGH
        asm volatile(
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          ::: "r0", "cc", "memory");
        // 800ns LOW (meas. 792ns)
        gpio->BRR = pinMask; // LOW
        asm volatile(
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          ::: "r0", "cc", "memory");
      }
        asm volatile(
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
              ::: "r0", "cc", "memory");
    }
    
    mask = 0x1000000; // reset the mask, start 1 higher than
    if (c & 1) { // mask shifts first, then & with c
      // 700ns HIGH (meas. 694ns)
      gpio->BSRR = pinMask; // HIGH
      g = *ptr++;   // Next green byte value
      r = *ptr++;   // Next red byte value
      b = *ptr++;   // Next blue byte value
      i--;
      asm volatile(
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
          "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
        ::: "r0", "cc", "memory");
      // 600ns LOW (meas. 598ns)
      gpio->BRR = pinMask; // LOW 
    } else {
      // 350ns HIGH (meas. 360ns)
      gpio->BSRR = pinMask; // HIGH
      g = *ptr++;   // Next green byte value
      r = *ptr++;   // Next red byte value
      b = *ptr++;   // Next blue byte value
      i--;
      asm volatile(
        "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
        ::: "r0", "cc", "memory");
      // 800ns LOW (meas. 792ns)
      gpio->BRR = pinMask; // LOW
      asm volatile(
        "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
        "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
        "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
        "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
        "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
        "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t" "mov r0, r0" "\n\t"
        ::: "r0", "cc", "memory");
    }
  } // end while(i)
  __enable_irq();
}

void MetaStripSetPixelColor(LEDStrip* strip, Length pixelN, Color* color) {
  LEDStripSetColor(strip->substrips[pixelN], color);
}

void MetaStripGetPixelColor(LEDStrip* strip, Length pixelN, Color* color) {
    color = &strip->substrips[pixelN]->color;
}

void MetaStripAnimate(LEDStrip* strip) {
  LEDStripAnimateBase(strip);
  for (Length i = 0; i < strip->length; i++) {
    strip->substrips[i]->animate(strip->substrips[i]);
  }
}

void MetaStripInit(LEDStrip* strip, LEDStrip** substrips, Length count) {
  LEDStripInit(strip, count);
  strip->substrips = substrips;
  strip->setPixelColor = MetaStripSetPixelColor;
  #ifdef ALLOW_GET_PIXEL
  strip->getPixelColor = MetaStripGetPixelColor;
#endif
  strip->animate = MetaStripAnimate;
}


#define SIN_TABLE_SIZE 128

static const int8_t sint[SIN_TABLE_SIZE] = {
    0,   3,   6,   9,  12,  16,  19,  22,  25,  28,  31,  34,  37,  40,  43,  46,
    49,  51,  54,  57,  60,  63,  65,  68,  71,  73,  76,  78,  81,  83,  85,  88,
    90,  92,  94,  96,  98, 100, 102, 104, 106, 107, 109, 111, 112, 113, 115, 116,
    117, 118, 120, 121, 122, 122, 123, 124, 125, 125, 126, 126, 126, 127, 127, 127,
    127, 127, 127, 127, 126, 126, 126, 125, 125, 124, 123, 122, 122, 121, 120, 118,
    117, 116, 115, 113, 112, 111, 109, 107, 106, 104, 102, 100,  98,  96,  94,  92,
    90,  88,  85,  83,  81,  78,  76,  73,  71,  68,  65,  63,  60,  57,  54,  51,
    49,  46,  43,  40,  37,  34,  31,  28,  25,  22,  19,  16,  12,   9,   6,   3 };

static int8_t Sin(uint8_t angle) {
    if (angle >= SIN_TABLE_SIZE) return - sint[angle - SIN_TABLE_SIZE];
    return sint[angle];
}


static void nextFrame(FrameNumber* frame, FrameDivider divider, FrameCount max) {
    frame->f += 1;
    if (frame->f >= divider) {
        frame->f = 0;
        frame->i += 1;
        if (frame->i >= max) {
            frame->i = 0;
        }
    }
}

static void nextFrame2(FrameNumber* frame, FrameCount step, FrameDivider divider, FrameCount max) {
    FrameCount nf = frame->f + step;
    while (nf >= divider) {
        nf -= divider;
        frame->i += 1;
        if (frame->i >= max) {
            frame->i = 0;
        }
    }
    frame->f = nf;
}

static RGB_t background(FrameParameters* data) {
    return data->transparent ? TRANSPARENT : BLACK;
}

void singleColorFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data) {
    Color color;
    CONFIGURE_COLOR_RGB(color, data->color);
    LEDStripSetAllPixels(strip, &color);
    nextFrame(frame, data->divider, 1);
}

void colorSinRainbowFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data) {
    Color color;
    for (Length i = 0; i < strip->length; i++) {
        uint16_t v = Sin(((i + frame->i) % (SIN_TABLE_SIZE / 8)) * 8 ) / 8;
        CONFIGURE_COLOR_RGB(color, hsv2rgb((HSV_t) {frame->i, v, 1}));
        LEDStripSetPixel(strip, i, &color, data->inverted);
    }
    nextFrame(frame, data->divider, FULL_HUE);
}

void colorSinFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data) {
    RGB_t c = data->color;
    Color color;
    for (Length i = 0; i < strip->length; i++) {
        uint16_t v = Sin(((i + frame->i) % (SIN_TABLE_SIZE / 8)) * 8 ) / 10;
        CONFIGURE_COLOR_RGB(color, RGB((c.r * v) / 128, (c.g * v) / 128, (c.b * v) / 128));
        LEDStripSetPixel(strip, i, &color, data->inverted);
    }
    nextFrame(frame, data->divider, SIN_TABLE_SIZE);
}

// Fill the dots one after the other with a color
void colorGlowFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data) {
    RGB_t c = data->color;
    Color color;
    if (frame->i < 128) {
        CONFIGURE_COLOR_RGB(color, RGBD(c, 
#ifdef SMALL_DIM
        255 / frame->i
#else
        frame->i
#endif          
          ));
    } else {
        CONFIGURE_COLOR_RGB(color, RGBD(c, 
#ifdef SMALL_DIM
        255 / (255 - frame->i)
#else
        255 - frame->i
#endif  
        ));
    }        
    for (Length i = 0; i < strip->length; i++) {
      LEDStripSetPixel(strip,  i, &color, data->inverted);
    }
    nextFrame2(frame, 32, data->divider, 255);
}

void colorBlinkFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data) {
    Color color;
    if (frame->i == data->parameter1) {
        CONFIGURE_COLOR_RGB(color, data->color);
    } else {
        CONFIGURE_COLOR_RGB(color, background(data));
    }
    LEDStripSetAllPixels(strip, &color);
    nextFrame2(frame, 1, data->divider, data->parameter1 + 1);
}

// Fill the dots one after the other with a color
void colorRunFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data) {
  Color bg;
  CONFIGURE_COLOR_RGB(bg, background(data));

  Color color;
  CONFIGURE_COLOR_RGB(color, data->color);

  FrameCount i = frame->i;
  LEDStripSetAllPixels(strip, &bg);
    
  for (Length j = 0; j < data->parameter1; j++, i++) {
    LEDStripSetPixel(strip, WRAP(i, strip->length), &color, data->inverted);
  }
  nextFrame(frame, data->divider, strip->length);
}

// Fill the dots one after the other with a color
void colorWipeFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data) {
  Color color;
  CONFIGURE_COLOR_RGB(color, data->color);
  Color bg;
  CONFIGURE_COLOR_RGB(bg, background(data));
  Length color_f, color_t, bg_f, bg_t;
  if (frame->i < strip->length) {
    color_f = 0; color_t = frame->i;
    bg_f = frame->i + 1; bg_t = strip->length - 1;
  } else {
    Length l = frame->i - strip->length;
    color_f = l + 1; color_t = strip->length - 1;
    bg_f = 0; bg_t = l;
  }
  LEDStripSetPixelRange(strip, color_f, color_t, &color, data->inverted);
  LEDStripSetPixelRange(strip, bg_f, bg_t, &bg, data->inverted);
  
  nextFrame(frame, data->divider, strip->length * 2);
}

void colorCometFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data) {
  Color color;
  CONFIGURE_COLOR_RGB(color, data->color);
  Color bg;
  CONFIGURE_COLOR_RGB(bg, background(data));
  Length color_f, color_t, bg_f, bg_t;
  uint8_t s = frame->i / strip->length;
  switch(s) {
    case 0:
    color_f = 0; color_t = frame->i;
    bg_f = frame->i + 1; bg_t = strip->length - 1;
    break;
    case 1: {
      Length l = frame->i - strip->length;
      color_f = l + 1; color_t = strip->length - 1;
      bg_f = 0; bg_t = l;
    } 
    break;
    case 2: {
      Length l = frame->i - strip->length * 2;
      color_f = strip->length - l; color_t = strip->length - 1;
      bg_f = 0; bg_t = strip->length - l;
    }
    break;
    case 3: {
      Length l = frame->i - strip->length * 3;
      color_f = 0; color_t = strip->length - l;
      bg_f = strip->length - l; bg_t = strip->length - 1;
    }
    break;
  }

  LEDStripSetPixelRange(strip, color_f, color_t, &color, data->inverted);
  LEDStripSetPixelRange(strip, bg_f, bg_t, &bg, data->inverted);
  
  nextFrame(frame, data->divider, strip->length * 4);
}

// Fill the dots one after the other with a color
void colorFillFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data) {
  Color color;
  CONFIGURE_COLOR_RGB(color, data->color);
  Color bg;
  CONFIGURE_COLOR_RGB(bg, background(data));
  LEDStripSetPixelRange(strip, 0, frame->i, &color, data->inverted);
  LEDStripSetPixelRange(strip, frame->i + 1, strip->length - 1, &bg, data->inverted);
  
  nextFrame(frame, data->divider, strip->length);
}

void rainbowFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data) {
    Color color;
    for (Length i=0; i< strip->length; i++) {
        CONFIGURE_COLOR_RGB(color, wheel((i + frame->i) & 255));
        LEDStripSetPixel(strip, i, &color, data->inverted);
    }
    nextFrame(frame, data->divider, 255);
}

void rainbowSimpleFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data) {
  Color color; 
  uint8_t ci = frame->i;
  for (Length i = 0; i < strip->length; i++, ci++) {
    if (ci >= ARRAY_SIZE(rainbowColors)) {
      ci = 0;
    }
    CONFIGURE_COLOR_RGB(color, rainbowColors[ci]);
      LEDStripSetPixel(strip, i, &color, data->inverted);
  }
  nextFrame(frame, data->divider, ARRAY_SIZE(rainbowColors));
}

void rainbowWholeFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data) {
  Color color;
  CONFIGURE_COLOR_RGB(color, wheel(frame->i));
  LEDStripSetAllPixels(strip, &color);
  nextFrame(frame, data->divider, 255);
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycleFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data) {
    uint8_t df = 256 / strip->length;
    FrameCount f =  frame->i;
    Color color;
    for (Length i=0; i < strip->length; i++, f += df) {
        CONFIGURE_COLOR_RGB(color, wheel(f));
        LEDStripSetPixel(strip,  i, &color, data->inverted);
    }
    nextFrame(frame, data->divider, 255);
}

void randomColorFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data) {
    Color color;
    CONFIGURE_COLOR_RGB(color, data->color);
    if (frame->i < strip->length - 5) { // Show pixels
      #ifdef ALLOW_GET_PIXEL
        uint8_t foundEmptyPlace = 0;
        do {
            Length n = rand() % strip->length;
            Color currentColor;
            LEDStripGetPixel(strip, n, &currentColor, data->inverted);
            RGB_t c = currentColor.frameParameters.color;
            if (c.r == 0 && c.g == 0 && c.b == 0) {
                foundEmptyPlace = 1;
                LEDStripSetPixel(strip,  n, &color, data->inverted);
            }
        } while (!foundEmptyPlace);
#else
        Length n = rand() % strip->length;
        LEDStripSetPixel(strip,  n, &color, data->inverted);
#endif
    } else {
        LEDStripClear(strip);
    }
    nextFrame(frame, data->divider, strip->length + 1);
}

void flameFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data) {
    if (frame->pi != frame->i) {
        frame->pi = frame->i;
        Length k = strip->length / data->parameter1;
        Color color;
        uint8_t d = data->parameter2 == 0 ? 1 : data->parameter2;
        for(Length i = 0; i < strip->length; i++) {
            uint8_t x = rand() % 60 + 67;
            RGB_t rgb;
            for (uint8_t c = 0; c < ARRAY_SIZE(data->color.grb); c++) {
              if (data->color.grb[c] == 255) {
                  uint16_t v = (uint16_t) x * ((i % k) == 0 ? 2 : 4);
                  rgb.grb[c] = (v > 255 ? 255 : v) / d;
                } else if (data->color.grb[c] == 0) {
                  rgb.grb[c] = 0;
                } else {
                  rgb.grb[c] = x / d;
                }
            }
            CONFIGURE_COLOR_RGB(color, rgb);
            LEDStripSetPixel(strip, i, &color, data->inverted);
        }
        frame->f = rand() % data->divider;
    }
    nextFrame(frame, data->divider, 2);
} 

void metaFlameFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data) {
  Color color;
  CONFIGURE_COLOR(color, flameFrame, 0, data->divider, data->color, 0, 0, data->parameter1, 0);
  for (Length i = 0; i < strip->length; i++) {
      LEDStripSetPixel(strip, i, &color, data->inverted);
  }
  nextFrame(frame, data->divider, 2);
}

void randomPixelFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data) {
  Color color;
  CONFIGURE_COLOR_RGB(color, data->color);
  Color bg;
  CONFIGURE_COLOR_RGB(bg, background(data));
  LEDStripSetAllPixels(strip, &bg);
  LEDStripSetPixel(strip, rand() % strip->length, &color, data->inverted);
  nextFrame(frame, data->divider, 2);
}

void rainbowSimpleWipeFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data) {
  Color color;
  uint8_t ci = frame->i;
  for (Length i = 0; i < strip->length; i++, ci++) {
    if (ci >= ARRAY_SIZE(rainbowColors)) {
      ci = 0;
    }
    CONFIGURE_COLOR(color, colorWipeFrame, MAX_FRAME_COUNT, data->parameter1, rainbowColors[ci], 0, 0, 0, 0);
    LEDStripSetPixel(strip, i, &color, data->inverted);
  }
  nextFrame(frame, data->divider, ARRAY_SIZE(rainbowColors));
}

void rainbowSimpleRunFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data) {
  Color color;
  uint8_t ci = frame->i;
  for (Length i = 0; i < strip->length; i++, ci++) {
    if (ci >= ARRAY_SIZE(rainbowColors)) {
      ci = 0;
    }
    CONFIGURE_COLOR(color, colorRunFrame, MAX_FRAME_COUNT, data->parameter1, rainbowColors[ci], 0, 0, data->parameter2, 0);
    LEDStripSetPixel(strip, i, &color, data->inverted);
  }
  nextFrame(frame, data->divider, ARRAY_SIZE(rainbowColors));
}

void rainbowSimpleFillFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data) {
  Color color;
  uint8_t ci = frame->i;
  for (Length i = 0; i < strip->length; i++, ci++) {
    if (ci >= ARRAY_SIZE(rainbowColors)) {
      ci = 0;
    }
    CONFIGURE_COLOR(color, colorFillFrame, MAX_FRAME_COUNT, data->parameter1, rainbowColors[ci], 1, 0, 0, 0);
    LEDStripSetPixel(strip, i, &color, data->inverted);
  }
  nextFrame(frame, data->divider, ARRAY_SIZE(rainbowColors));
}

void rainbowWipeFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data) {
  Color color;
  uint8_t ci = frame->i;
  uint8_t df = 256 / strip->length;
  FrameCount f =  frame->i;
  for (Length i = 0; i < strip->length; i++, f+=df) {
    CONFIGURE_COLOR(color, colorWipeFrame, MAX_FRAME_COUNT, data->parameter1, wheel(f), 0, 0, 0, 0);
    LEDStripSetPixel(strip, i, &color, data->inverted);
  }
  nextFrame(frame, data->divider, 255);
}

void rainbowRunFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data) {
  Color color;
  uint8_t ci = frame->i;
  uint8_t df = 256 / strip->length;
  FrameCount f =  frame->i;
  for (Length i = 0; i < strip->length; i++, f+=df) {
    CONFIGURE_COLOR(color, colorRunFrame, MAX_FRAME_COUNT, data->parameter1, wheel(f), 0, 0, data->parameter2, 0);
    LEDStripSetPixel(strip, i, &color, data->inverted);
  }
  nextFrame(frame, data->divider, 255);
}

void rainbowFillFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data) {
  Color color;
  uint8_t ci = frame->i;
  uint8_t df = 256 / strip->length;
  FrameCount f =  frame->i;
  for (Length i = 0; i < strip->length; i++, f+=df) {
    CONFIGURE_COLOR(color, colorFillFrame, MAX_FRAME_COUNT, data->parameter1, wheel(f), 1, 0, 0, 0);
    LEDStripSetPixel(strip, i, &color, data->inverted);
  }
  nextFrame(frame, data->divider, 255);
}

void policeLightsFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data) {
  Color color;
  switch (frame->i) {
    case 0:
    case 2:
    case 4:
      CONFIGURE_COLOR_RGB(color, data->color);
      break;
    default:
      CONFIGURE_COLOR_RGB(color, background(data));
      break;
  }
  LEDStripSetAllPixels(strip, &color);
  nextFrame(frame, data->divider, 32);
}
