// ESP32-S3: 1 Hz mouse nudge + ultra-smooth NeoPixel fade (HSV + gamma + easing)

#include "USB.h"
#include "USBHIDMouse.h"
#include <Adafruit_NeoPixel.h>
#include <math.h>

USBHIDMouse Mouse;

#define LED_PIN    48
#define LED_COUNT  1
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// 1 Hz cadence
const uint32_t PERIOD_MS = 1500;

// Small relative movement
const int8_t STEP_MIN = -3;
const int8_t STEP_MAX =  4;

// HSV state (16-bit hue for fine resolution)
uint32_t tStart = 0;
uint16_t hueFrom = 0;
uint16_t hueTo   = 0;
const uint8_t SAT = 255;      // full saturation
const uint8_t VAL = 10;       // overall brightness (tweak 1..255)

static inline uint16_t wrap16(int32_t x) { return (uint16_t)(x & 0xFFFF); }

// Shortest-path hue delta in signed 16-bit domain
static int16_t hueDelta(uint16_t from, uint16_t to) {
  int32_t d = (int32_t)to - (int32_t)from;
  if (d >  32767) d -= 65536;
  if (d < -32768) d += 65536;
  return (int16_t)d;
}

// Cosine ease-in-out for perceptual smoothness
static inline float easeCos(float x) {
  if (x <= 0) return 0;
  if (x >= 1) return 1;
  return 0.5f - 0.5f * cosf((float)M_PI * x);
}

static void startNewFade(uint16_t startHue) {
  hueFrom = startHue;
  hueTo   = (uint16_t)random(0, 65536);  // pick any target hue
  tStart  = millis();
}

void setup() {
  USB.begin();
  Mouse.begin();
  randomSeed(esp_random());

  strip.begin();
  strip.setBrightness(VAL);     // set once; use HSV V for overall level
  strip.show();

  hueFrom = (uint16_t)random(0, 65536);
  hueTo   = (uint16_t)random(0, 65536);
  tStart  = millis();
}

void loop() {
  const uint32_t now = millis();
  const uint32_t dt  = now - tStart;

  // Progress 0..1 over one second
  const float t = (dt >= PERIOD_MS) ? 1.0f : (float)dt / (float)PERIOD_MS;

  // Eased hue along shortest arc, 16-bit precision
  const int16_t dHue = hueDelta(hueFrom, hueTo);
  const uint16_t curHue = wrap16((int32_t)hueFrom + (int32_t)(dHue * easeCos(t)));

  // HSV -> RGB, then gamma correct for perceptual smoothness
  uint32_t rgb = strip.ColorHSV(curHue, SAT, 255);     // use full value, brightness is via setBrightness
  rgb = strip.gamma32(rgb);                            // perceptual correction
  strip.setPixelColor(0, rgb);
  strip.show();

  // 1 Hz: move mouse and start next fade
  static uint32_t lastTick = now;
  if (now - lastTick >= PERIOD_MS) {
    lastTick = now;
    Mouse.move(random(STEP_MIN, STEP_MAX), random(STEP_MIN, STEP_MAX), 0);
    startNewFade(curHue);
  }

  // Tight loop for fine granularity, very cheap for 1 pixel
  delay(1);
}
