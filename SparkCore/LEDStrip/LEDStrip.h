#ifndef __ledstrip_h__
#define __ledstrip_h__

//#define SMALL_DIM
//#define ALLOW_GET_PIXEL

#include "stm32f10x.h"

#define ARRAY_SIZE(A) (sizeof(A) / sizeof(A[0]))
#define MAX_OF(TYPE) ((1 << (sizeof(TYPE) * 8)) - 1)

typedef uint8_t Length;
typedef uint8_t FrameCount;
typedef uint8_t FrameDivider;

#define MAX_FRAME_COUNT MAX_OF(FrameCount)

typedef struct {
    FrameCount i;
    FrameCount pi;
    FrameDivider f;
} FrameNumber;


typedef struct {
    union {
        struct {
            uint8_t g;
            uint8_t r;
            uint8_t b;
        };
        uint8_t grb[3];
    };
    uint8_t alpha;
} RGB_t;

typedef struct {
    uint8_t h;
    uint8_t s;
    uint8_t v;
} HSV_t;


#define MAX_COLOR_COMPONENT 255

#define TRANSPARENT ((RGB_t){{{0,0,0}},0})
#define RGB(r,g,b) ((RGB_t){{{(g),(r),(b)}}, 255})
#define GREY(x)   RGB(x,x,x)
#define BLACK     GREY(0)

#define RED       RGB(MAX_COLOR_COMPONENT, 0, 0)
#define ORANGE    RGB(MAX_COLOR_COMPONENT, (MAX_COLOR_COMPONENT + 1) / 4,  0)
#define YELLOW    RGB(MAX_COLOR_COMPONENT, MAX_COLOR_COMPONENT, 0)
#define GREEN     RGB(0, MAX_COLOR_COMPONENT, 0)
#define SKY_BLUE  RGB((MAX_COLOR_COMPONENT + 1) / 4, (MAX_COLOR_COMPONENT + 1) / 4, MAX_COLOR_COMPONENT)
#define BLUE      RGB(0, 0, MAX_COLOR_COMPONENT)
#define MAGENTA   RGB(MAX_COLOR_COMPONENT, 0, (MAX_COLOR_COMPONENT + 1) / 2)
#define WHITE     GREY(MAX_COLOR_COMPONENT)


#define RED_FLAME RGB(255, 127, 0)
#define ORANGE_FLAME RGB(255, 127, 127)
#define YELLOW_FLAME RGB(127, 255, 0)
#define GREEN_FLAME RGB(0, 255, 127)
#define SKY_BLUE_FLAME RGB(0, 127, 255)
#define BLUE_FLAME RGB(127, 0, 255)
#define MAGENTA_FLAME RGB(255, 0, 127)
#define WHITE_BLUE_FLAME RGB(255, 255, 255)


typedef struct {
    FrameDivider divider;
    RGB_t color;
    uint8_t transparent : 1;
    uint8_t inverted : 1;
    uint8_t parameter1;
    uint8_t parameter2;
} FrameParameters;

struct _LEDStrip;

typedef void (*FrameGenerator)(struct _LEDStrip* strip, FrameNumber* frame, FrameParameters* data);

typedef struct {
	FrameGenerator frameGenerator;
	FrameCount startFrame;
	FrameParameters frameParameters;
} Color;

#define CONFIGURE_COLOR(COLOR, FG, SF, DIV, CLR, TR, INV, P1, P2) \
  COLOR.frameGenerator = FG;\
  COLOR.startFrame = SF;\
  COLOR.frameParameters.divider = DIV;\
  COLOR.frameParameters.color = CLR;\
  COLOR.frameParameters.transparent = TR;\
  COLOR.frameParameters.inverted = INV;\
  COLOR.frameParameters.parameter1 = P1;\
  COLOR.frameParameters.parameter2 = P2
  
#define CONFIGURE_COLOR_RGB(COLOR, CLR) \
CONFIGURE_COLOR(COLOR, singleColorFrame, 0, 0, CLR, 0, 0, 0, 0)

#define CONFIGURE_COLOR_SIMPLE(COLOR, FG, DIV, CLR) \
CONFIGURE_COLOR(COLOR, FG, 0, DIV, CLR, 0, 0, 0, 0)
		
#define CONFIGURE_COLOR_SIMPLE_WP(COLOR, FG, DIV, CLR, P1, P2) \
CONFIGURE_COLOR(COLOR, FG, 0, DIV, CLR, 0, 0, P1, P2)
		
#define CONFIGURE_COLOR_TRANSPARENT(COLOR) \
CONFIGURE_COLOR(COLOR, NULL, 0, 0, TRANSPARENT, 0, 0, 0, 0)


typedef struct _LEDStrip {
  union {
    struct _LEDStrip** substrips;
    struct {
      struct _LEDStrip* baseStrip;
  	  Length position;
    };
		struct {
			uint8_t* pixels;
			GPIO_TypeDef* gpio;
			uint16_t pinMask;
		};
  };
	Length length;
	Color color;
	bool inverted;
	FrameNumber frameNumber;
#ifdef ALLOW_GET_PIXEL
  void (*getPixelColor)(struct _LEDStrip* strip, Length pixelN, Color* color);
#endif
  void (*setPixelColor)(struct _LEDStrip* strip, Length pixelN, Color* color);
  void (*animate)(struct _LEDStrip* strip);
} LEDStrip;

void LEDStripSetPixel(LEDStrip* strip, Length pixelN, Color* color, uint8_t inverted);
void LEDStripBegin(const Length length);

void LEDStripSetPixel(LEDStrip* strip, Length pixelN, Color* color, uint8_t inverted);
#ifdef ALLOW_GET_PIXEL
void LEDStripGetPixel(LEDStrip* strip, Length pixelN, Color* color, uint8_t inverted);
#endif
void LEDStripSetPixelRange(LEDStrip* strip, Length from, Length to, Color* c, uint8_t inverted);
void LEDStripSetAllPixels(LEDStrip* strip, Color* c);
void LEDStripSetColor(LEDStrip* strip, Color* color);
void LEDStripClear(LEDStrip* strip);
void LEDStripAnimate(LEDStrip* strip);

void PartialStripInit(LEDStrip* strip, LEDStrip* baseStrip, const Length position, const Length length);
void SparkStripInit(LEDStrip* strip, uint8_t* pixels, const uint8_t pin, const Length length);
void MetaStripInit(LEDStrip* strip, LEDStrip** substrips, Length count);

void SparkStripShow(LEDStrip* strip);

void singleColorFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data);
void randomColorFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data);
void colorWipeFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data);
void colorFillFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data);
void colorCometFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data);
void rainbowCycleFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data);
void rainbowFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data);
void rainbowSimpleFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data);
void rainbowWholeFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data);
void colorRunFrame(LEDStrip* strip, FrameNumber* wait, FrameParameters* data);
void colorGlowFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data);
void colorBlinkFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data);
void colorSinFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data);
void colorSinRainbowFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data);
void policeLightsFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data);
void flameFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data);
void metaFlameFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data);
void randomPixelFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data);


void rainbowSimpleWipeFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data);
void rainbowSimpleRunFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data);
void rainbowSimpleFillFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data);

void rainbowWipeFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data);
void rainbowRunFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data);
void rainbowFillFrame(LEDStrip* strip, FrameNumber* frame, FrameParameters* data);

#endif
