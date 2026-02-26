#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <FastLED.h>
#include "driver/i2s_std.h"
#include "driver/gpio.h"

// ================= WIFI / WEB =================
static const char* WIFI_SSID = "Stinky";
static const char* WIFI_PASS = "Prinzessin457";
WebServer server(80);

// ================= LED =================
#define LED_PIN 5
#define NUM_LEDS 275
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

// ================= Layout / Zones =================
static constexpr int T1_S = 0,   T1_E = 33;    // left side vertical
static constexpr int B1_S = 34,  B1_E = 130;   // top horizontal
static constexpr int T2_S = 131, T2_E = 173;   // right side vertical
static constexpr int B2_S = 174, B2_E = 274;   // bottom horizontal

static constexpr int T1_LEN = (T1_E - T1_S + 1);
static constexpr int B1_LEN = (B1_E - B1_S + 1);
static constexpr int T2_LEN = (T2_E - T2_S + 1);
static constexpr int B2_LEN = (B2_E - B2_S + 1);

static constexpr int PERIM_LEN = T1_LEN + B1_LEN + T2_LEN + B2_LEN;

// ================= Controls (web adjustable) =================
volatile bool     g_enabled    = true;
volatile uint8_t  g_brightness = 165;

// audio tuning
volatile float    g_bassGate   = 0.18f;
volatile float    g_bassGamma  = 2.0f;
volatile float    g_trebleGain = 1.85f;
volatile uint8_t  g_bassSmooth = 70; // 0..100 (higher = smoother)

// Modes
// 0 Music Reactive (Bars + Peak Hold)
// 1 Solid Color
// 2 Circle Chase (bounce)
// 3 Rainbow Madness (optional beat flash + strobe)
// 4 Border Rotate
// 5 Bass Wave
// 6 VU Border (symmetric)
// 7 Corners Hit (audio-driven + beat boost)
volatile uint8_t g_mode = 0;

// Toggle between OLD flashy behavior and NEW no-flash behavior
// 0 = No Flash (new), 1 = Flashy (old)
volatile uint8_t g_flashyMode = 0;

// color modes for music-reactive
volatile uint8_t g_bassColorMode   = 1; // 0 solid, 1 gradient
volatile uint8_t g_trebleColorMode = 1; // 0 solid, 1 gradient

volatile uint8_t g_mainColor[3] = {255, 80, 10};

volatile uint8_t g_bassSolid[3] = {255,  60,   0};
volatile uint8_t g_bassLow[3]   = {  0, 255,   0};
volatile uint8_t g_bassHigh[3]  = {255,   0,   0};

volatile uint8_t g_trebleSolid[3] = {  0, 140, 255};
volatile uint8_t g_trebleLow[3]   = {  0, 255, 255};
volatile uint8_t g_trebleHigh[3]  = {200,   0, 255};

// peak-hold (music bars)
volatile uint8_t g_peakEnable   = 1;   // 0/1
volatile uint8_t g_peakFallMs   = 28;  // higher = slower fall (ms per step)

// corners hit effect
volatile uint8_t g_cornerEnable = 1;   // 0/1
volatile uint8_t g_cornerSpread = 18;  // LEDs to spread from each corner

// strobe-on-beat (intensity 0 disables)
volatile uint8_t g_strobeIntensity = 0; // 0..255

// Effects speeds
volatile uint8_t g_circleSpeedMs = 18; // circle chase speed
volatile uint8_t g_borderSpeedMs = 20; // border rotate speed
volatile uint8_t g_rainbowSpeed  = 1;  // hue increment per frame
volatile uint8_t g_vuSmoothing   = 70; // VU border smoothing (0..100)

// ================= Live debug values (/debug) =================
static volatile float g_dbg_bassEnv=0, g_dbg_trebEnv=0, g_dbg_volEnv=0;
static volatile float g_dbg_bassRaw=0, g_dbg_trebRaw=0;
static volatile uint8_t g_dbg_beatHit=0;

// ================= I2S (INMP441) =================
static constexpr gpio_num_t PIN_BCLK = GPIO_NUM_14;
static constexpr gpio_num_t PIN_WS   = GPIO_NUM_27;
static constexpr gpio_num_t PIN_DIN  = GPIO_NUM_32;

static constexpr uint32_t SAMPLE_RATE = 16000;
static constexpr int N = 256;
static int32_t samples32[N];
static i2s_chan_handle_t rx_chan = nullptr;

// ================= Forward declarations =================
static void handleRoot();
static void handleGet();
static void handleSet();
static void handleToggle();
static void handleDebug();

// ================= Helpers =================
static inline float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }
static inline uint8_t u8(float x) { x = x < 0 ? 0 : (x > 255 ? 255 : x); return (uint8_t)x; }
static inline float lerp1f(float a, float b, float t) { return a + (b - a) * t; }

static uint32_t parseHexColor(const String &hex) {
  String s = hex;
  if (s.length() == 7 && s[0] == '#') s = s.substring(1);
  if (s.length() != 6) return 0;
  return (uint32_t)strtoul(s.c_str(), nullptr, 16);
}

static String toHex(uint8_t r, uint8_t g, uint8_t b) {
  char buf[8];
  snprintf(buf, sizeof(buf), "#%02X%02X%02X", r, g, b);
  return String(buf);
}

static CRGB lerpRGB(const volatile uint8_t a[3], const volatile uint8_t b[3], float t) {
  t = clamp01(t);
  uint8_t ar = a[0], ag = a[1], ab = a[2];
  uint8_t br = b[0], bg = b[1], bb = b[2];
  return CRGB(
    (uint8_t)lerp1f((float)ar, (float)br, t),
    (uint8_t)lerp1f((float)ag, (float)bg, t),
    (uint8_t)lerp1f((float)ab, (float)bb, t)
  );
}

static CRGB mainColor() { return CRGB(g_mainColor[0], g_mainColor[1], g_mainColor[2]); }

static CRGB bassColorForLevel(float level01) {
  level01 = clamp01(level01);
  if (g_bassColorMode == 0) return CRGB(g_bassSolid[0], g_bassSolid[1], g_bassSolid[2]);
  return lerpRGB(g_bassLow, g_bassHigh, level01);
}

static CRGB trebleColorForLevel(float level01) {
  level01 = clamp01(level01);
  if (g_trebleColorMode == 0) return CRGB(g_trebleSolid[0], g_trebleSolid[1], g_trebleSolid[2]);
  return lerpRGB(g_trebleLow, g_trebleHigh, level01);
}

static int perimeterIndexToLed(int p) {
  p %= PERIM_LEN;
  if (p < 0) p += PERIM_LEN;

  if (p < T1_LEN) return T1_S + p;  // left bottom->top
  p -= T1_LEN;

  if (p < B1_LEN) return B1_S + p;  // top left->right
  p -= B1_LEN;

  if (p < T2_LEN) return T2_S + p;  // right top->bottom
  p -= T2_LEN;

  if (p < B2_LEN) return B2_S + (B2_LEN - 1 - p); // bottom right->left
  return 0;
}

static int perimBottomMiddle() {
  int led = B2_S + (B2_LEN / 2);
  int pBottom = (B2_LEN - 1) - (led - B2_S);
  return (T1_LEN + B1_LEN + T2_LEN) + pBottom;
}

static int perimTopMiddle() {
  int pTop = B1_LEN / 2;
  return T1_LEN + pTop;
}

static int ledBottomLeft()  { return B2_S + (B2_LEN - 1); }
static int ledBottomRight() { return B2_S + 0; }
static int ledTopLeft()     { return B1_S + 0; }
static int ledTopRight()    { return B1_S + (B1_LEN - 1); }

static int borderFromCornerClockwise(uint8_t corner, int off) {
  int baseP = 0;
  switch (corner) {
    case 0: baseP = PERIM_LEN - 1; break;                 // BL
    case 1: baseP = T1_LEN; break;                        // TL
    case 2: baseP = T1_LEN + B1_LEN - 1; break;           // TR
    case 3: baseP = T1_LEN + B1_LEN + T2_LEN - 1; break;  // BR
  }
  return perimeterIndexToLed(baseP + off);
}

static void clearZone(int start, int len) {
  for (int i = 0; i < len; i++) leds[start + i] = CRGB::Black;
}

// ================= I2S =================
static void setupI2S() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, nullptr, &rx_chan));

  i2s_std_config_t std_cfg = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = PIN_BCLK,
      .ws   = PIN_WS,
      .dout = I2S_GPIO_UNUSED,
      .din  = PIN_DIN,
      .invert_flags = { .mclk_inv=false, .bclk_inv=false, .ws_inv=false },
    },
  };

  ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
}

static bool readBlock() {
  size_t bytes_read = 0;
  esp_err_t err = i2s_channel_read(rx_chan, samples32, sizeof(samples32), &bytes_read, pdMS_TO_TICKS(200));
  return (err == ESP_OK && bytes_read == sizeof(samples32));
}

// ================= DSP =================
static float goertzelPower(const float *x, int n, float sampleRate, float freqHz) {
  const float k = 0.5f + (n * freqHz / sampleRate);
  const float w = (2.0f * PI * k) / n;
  const float c = 2.0f * cosf(w);

  float s0 = 0, s1 = 0, s2 = 0;
  for (int i = 0; i < n; i++) {
    s0 = x[i] + c * s1 - s2;
    s2 = s1;
    s1 = s0;
  }
  return s1*s1 + s2*s2 - c*s1*s2;
}

static void toFloatWindowed(float *x) {
  int64_t sum = 0;
  for (int i = 0; i < N; i++) sum += (samples32[i] >> 8);
  float mean = (float)sum / (float)N;

  for (int i = 0; i < N; i++) {
    float v = (float)(samples32[i] >> 8) - mean;
    float w = 0.5f - 0.5f * cosf(2.0f * PI * (float)i / (float)(N - 1));
    x[i] = v * w;
  }
}

static float updateAgc(float current, float &agc, float attack, float release, float floorVal) {
  if (current > agc) agc = agc + (current - agc) * attack;
  else              agc = agc + (current - agc) * release;
  if (agc < floorVal) agc = floorVal;
  return agc;
}

static float gateAndCurve(float x, float gate, float gamma) {
  if (x <= gate) return 0.0f;
  float y = (x - gate) / (1.0f - gate);
  y = clamp01(y);
  return powf(y, gamma);
}

// outputs: bassEnv/trebEnv/volEnv in [0..1], beatHit, boomEnv
static void computeFeatures(float &bassEnv, float &trebEnv, float &volEnv, bool &beatHit, float &boomEnv,
                            float &bassRaw01, float &trebRaw01) {
  static float x[N];
  toFloatWindowed(x);

  float subP =
      1.10f * goertzelPower(x, N, SAMPLE_RATE, 30.0f) +
      1.10f * goertzelPower(x, N, SAMPLE_RATE, 40.0f) +
      1.00f * goertzelPower(x, N, SAMPLE_RATE, 55.0f);

  float bodyP =
      goertzelPower(x, N, SAMPLE_RATE, 70.0f) +
      goertzelPower(x, N, SAMPLE_RATE, 90.0f) +
      goertzelPower(x, N, SAMPLE_RATE, 120.0f) +
      goertzelPower(x, N, SAMPLE_RATE, 180.0f) +
      0.55f * goertzelPower(x, N, SAMPLE_RATE, 220.0f) +
      0.40f * goertzelPower(x, N, SAMPLE_RATE, 320.0f);

  float punchP =
      goertzelPower(x, N, SAMPLE_RATE, 110.0f) +
      goertzelPower(x, N, SAMPLE_RATE, 150.0f) +
      goertzelPower(x, N, SAMPLE_RATE, 200.0f) +
      goertzelPower(x, N, SAMPLE_RATE, 250.0f);

  float highP =
      goertzelPower(x, N, SAMPLE_RATE,  800.0f) +
      goertzelPower(x, N, SAMPLE_RATE, 1200.0f) +
      goertzelPower(x, N, SAMPLE_RATE, 1800.0f) +
      goertzelPower(x, N, SAMPLE_RATE, 2600.0f) +
      goertzelPower(x, N, SAMPLE_RATE, 3600.0f) +
      goertzelPower(x, N, SAMPLE_RATE, 5200.0f) +
      goertzelPower(x, N, SAMPLE_RATE, 6500.0f);

  float loudP =
      goertzelPower(x, N, SAMPLE_RATE, 250.0f) +
      goertzelPower(x, N, SAMPLE_RATE, 500.0f) +
      goertzelPower(x, N, SAMPLE_RATE, 1000.0f) +
      goertzelPower(x, N, SAMPLE_RATE, 2000.0f);

  float subE   = sqrtf(fmaxf(subP, 0.0f));
  float bodyE  = sqrtf(fmaxf(bodyP, 0.0f));
  float punchE = sqrtf(fmaxf(punchP, 0.0f));
  float highE  = sqrtf(fmaxf(highP, 0.0f));
  float loudE  = sqrtf(fmaxf(loudP, 0.0f));

  static float roomLoud = 1.0f;
  roomLoud = 0.9995f * roomLoud + 0.0005f * loudE;
  if (roomLoud < 200.0f) roomLoud = 200.0f;

  float sens = clamp01(600.0f / roomLoud);
  float bassDiv = lerp1f(1.9f, 3.2f, 1.0f - sens);
  float trebDiv = lerp1f(1.8f, 3.3f, 1.0f - sens);
  float volDiv  = lerp1f(2.0f, 3.6f, 1.0f - sens);

  static float subAGC=1.0f, bodyAGC=1.0f, punchAGC=1.0f, highAGC=1.0f, loudAGC=1.0f;
  updateAgc(subE,   subAGC,   0.14f, 0.0020f,  80.0f);
  updateAgc(bodyE,  bodyAGC,  0.14f, 0.0020f, 120.0f);
  updateAgc(punchE, punchAGC, 0.18f, 0.0030f, 120.0f);
  updateAgc(highE,  highAGC,  0.12f, 0.0030f, 120.0f);
  updateAgc(loudE,  loudAGC,  0.10f, 0.0020f, 200.0f);

  float subLevel0   = clamp01(subE   / (subAGC   * bassDiv));
  float bodyLevel0  = clamp01(bodyE  / (bodyAGC  * bassDiv));
  float punchLevel0 = clamp01(punchE / (punchAGC * bassDiv));
  float bassLevel0  = clamp01(0.62f * bodyLevel0 + 0.28f * punchLevel0 + 0.15f * subLevel0);
  bassRaw01 = bassLevel0;

  float bassLevel = gateAndCurve(bassLevel0, g_bassGate, g_bassGamma);

  float highLevel = clamp01(highE / (highAGC * trebDiv));
  float loudLevel = clamp01(loudE / (loudAGC * volDiv));

  float ratio = highE / (bodyE + 1.0f);
  float ratioSoft = clamp01((ratio - 0.03f) / 0.28f);
  ratioSoft = 0.45f + 0.55f * ratioSoft;

  float trebLevel = clamp01(highLevel * ratioSoft * g_trebleGain);
  trebRaw01 = trebLevel;

  float trebLevelForEnv = clamp01(trebLevel + 0.10f * bassLevel);

  static float bassState = 0, trebState = 0, volState = 0;

  float smooth = clamp01((float)g_bassSmooth / 100.0f);
  float bassAttack  = lerp1f(0.85f, 0.18f, smooth);
  float bassRelease = lerp1f(0.70f, 0.03f, smooth);
  float bassA = (bassLevel > bassState) ? bassAttack : bassRelease;

  float trebA = (trebLevelForEnv > trebState) ? 0.62f : 0.14f;
  float volA  = (loudLevel > volState) ? 0.18f : 0.08f;

  bassState += (bassLevel - bassState) * bassA;
  trebState += (trebLevelForEnv - trebState) * trebA;
  volState  += (loudLevel - volState) * volA;

  bassEnv = bassState;
  trebEnv = trebState;
  volEnv  = volState;

  // Boom detector (FIX: make it less "always on" during calm songs)
  static float prevPunch = 0.0f;
  float punchRise = punchLevel0 - prevPunch;
  prevPunch = punchLevel0;

  static float boomState = 0.0f;
  // Raised threshold makes boom trigger only on real kicks
  float boomHit = (punchRise > 0.10f) ? 1.0f : 0.0f;
  boomState += (boomHit - boomState) * (boomHit > 0 ? 0.92f : 0.20f);
  boomEnv = boomState;

  // Beat detector
  static float prevBass = 0.0f;
  const float BEAT_THRESH = 0.60f;
  const float BEAT_RISE   = 0.08f;
  static uint32_t lastBeat = 0;

  bool rawBeat = (bassEnv > BEAT_THRESH) && ((bassEnv - prevBass) > BEAT_RISE);
  bool cooldownOk = (millis() - lastBeat) > 80;
  beatHit = rawBeat && cooldownOk;
  if (beatHit) lastBeat = millis();
  prevBass = bassEnv;
}

// ================= Music reactive render =================
static void renderTrebleForward(int start, int len, float trebEnv, const CRGB &c, float &barPos) {
  float target = trebEnv * (float)len;
  barPos += (target - barPos) * 0.62f;

  int lit = (int)barPos;
  float frac = barPos - (float)lit;

  clearZone(start, len);
  uint8_t baseV = u8(10 + trebEnv * 225.0f);

  for (int i = 0; i < lit && i < len; i++) {
    leds[start + i] = c;
    leds[start + i].nscale8_video(baseV);
  }
  if (lit >= 0 && lit < len) {
    uint8_t topV = u8((float)baseV * clamp01(frac));
    leds[start + lit] = c;
    leds[start + lit].nscale8_video(topV);
  }
}

static void renderTrebleReverse(int start, int len, float trebEnv, const CRGB &c, float &barPos) {
  float target = trebEnv * (float)len;
  barPos += (target - barPos) * 0.62f;

  int lit = (int)barPos;
  float frac = barPos - (float)lit;

  clearZone(start, len);
  uint8_t baseV = u8(10 + trebEnv * 225.0f);

  for (int i = 0; i < lit && i < len; i++) {
    int ledIndex = start + (len - 1 - i);
    leds[ledIndex] = c;
    leds[ledIndex].nscale8_video(baseV);
  }
  if (lit >= 0 && lit < len) {
    int ledIndex = start + (len - 1 - lit);
    uint8_t topV = u8((float)baseV * clamp01(frac));
    leds[ledIndex] = c;
    leds[ledIndex].nscale8_video(topV);
  }
}

// IMPORTANT: updated to allow either "flashy" (old) or "no flash" (new)
static void renderBassMirror(int start, int len, float bassEnv, const CRGB &c, float beatBoost, float boomEnv, bool flashy) {
  // calmer baseline (reduces constant flashing on calm music)
  float level = clamp01(bassEnv);

  if (flashy) {
    // Old behavior: beat boost + additive boom (flashy)
    level = clamp01(level + beatBoost + 0.75f * boomEnv);
  } else {
    // New behavior: NO beat boost; boom only spikes when it is stronger than the baseline
    level = max(level, boomEnv);
  }

  int half = len / 2;
  int fill = (int)(level * (float)half);

  clearZone(start, len);

  uint8_t v = u8(10 + level * 235.0f);
  CRGB cc = c;
  cc.nscale8_video(v);

  for (int i = 0; i < fill; i++) {
    leds[start + i] = cc;
    leds[start + (len - 1 - i)] = cc;
  }
}

static void drawPeakHoldForward(int start, int len, float env, float &peakPos, uint32_t &lastFallMs) {
  if (!g_peakEnable) return;

  float pos = env * (float)(len - 1);
  if (pos > peakPos) peakPos = pos;

  uint32_t now = millis();
  if (now - lastFallMs >= (uint32_t)g_peakFallMs) {
    lastFallMs = now;
    peakPos -= 1.0f;
    if (peakPos < 0) peakPos = 0;
  }

  int idx = start + (int)peakPos;
  idx = constrain(idx, start, start + len - 1);
  leds[idx] = CRGB::White;
}

static void drawPeakHoldReverse(int start, int len, float env, float &peakPos, uint32_t &lastFallMs) {
  if (!g_peakEnable) return;

  float pos = env * (float)(len - 1);
  if (pos > peakPos) peakPos = pos;

  uint32_t now = millis();
  if (now - lastFallMs >= (uint32_t)g_peakFallMs) {
    lastFallMs = now;
    peakPos -= 1.0f;
    if (peakPos < 0) peakPos = 0;
  }

  int local = (int)peakPos;
  int idx = start + (len - 1 - local);
  idx = constrain(idx, start, start + len - 1);
  leds[idx] = CRGB::White;
}

// ================= Startup animation =================
static void runStartupAnimation() {
  const CRGB c = mainColor();
  FastLED.setBrightness(g_brightness);

  for (int lap = 0; lap < 2; lap++) {
    for (int p = 0; p < PERIM_LEN; p++) {
      fadeToBlackBy(leds, NUM_LEDS, 35);
      leds[perimeterIndexToLed(p)] += c;
      leds[perimeterIndexToLed(p - 1)] += CRGB(c.r / 3, c.g / 3, c.b / 3);
      leds[perimeterIndexToLed(p - 2)] += CRGB(c.r / 8, c.g / 8, c.b / 8);
      FastLED.show();
      delay(10);
    }
  }

  for (int v = 0; v <= 255; v += 8) {
    CRGB cc = c;
    cc.nscale8_video((uint8_t)v);
    fill_solid(leds, NUM_LEDS, cc);
    FastLED.show();
    delay(12);
  }

  for (int k = 0; k < 18; k++) {
    fadeToBlackBy(leds, NUM_LEDS, 35);
    FastLED.show();
    delay(15);
  }
  FastLED.clear(true);
}

// ================= Effect modes =================
static void modeSolid() {
  fill_solid(leds, NUM_LEDS, mainColor());
}

static void modeCircleChaseBounce() {
  static int phase = 0;
  static int dir = 1;
  static uint32_t last = 0;

  uint32_t now = millis();
  if (now - last > g_circleSpeedMs) { last = now; phase += dir; }

  const int startP = perimBottomMiddle();
  const int endP   = perimTopMiddle();
  const int halfTrip = (endP - startP + PERIM_LEN) % PERIM_LEN;
  const int fullPhase = 2 * halfTrip;

  if (phase >= fullPhase - 1) { phase = fullPhase - 1; dir = -1; }
  if (phase <= 0)             { phase = 0;             dir = +1; }

  int prog = phase;
  if (prog > halfTrip) prog = fullPhase - prog;

  fadeToBlackBy(leds, NUM_LEDS, 55);

  int pCW  = startP + prog;
  int pCCW = startP - prog;

  for (int k = 0; k < 20; k++) {
    float tt = 1.0f - (float)k / 20.0f;
    uint8_t v = u8(255 * tt);
    CRGB c = CHSV((uint8_t)(millis() / 10), 255, v);
    leds[perimeterIndexToLed(pCW - k)]  += c;
    leds[perimeterIndexToLed(pCCW + k)] += c;
  }

  if (prog >= halfTrip - 1) leds[perimeterIndexToLed(endP)] += CRGB::White;
  if (prog <= 1)            leds[perimeterIndexToLed(startP)] += CRGB::White;
}

static void modeRainbowMadness(bool beatHit) {
  static uint8_t hue = 0;
  hue += g_rainbowSpeed;

  for (int i = 0; i < NUM_LEDS; i++) leds[i] = CHSV(hue + i * 2, 255, 255);

  // Beat flash is OPTIONAL and controlled by g_flashyMode
  if (g_flashyMode && beatHit) {
    for (int i = 0; i < B1_LEN; i++) leds[B1_S + i] = CRGB::White;
    for (int i = 0; i < B2_LEN; i++) leds[B2_S + i] = CRGB::White;
  }

  if (beatHit && g_strobeIntensity > 0) {
    CRGB st = CRGB::White;
    st.nscale8_video(g_strobeIntensity);
    for (int i = 0; i < NUM_LEDS; i++) leds[i] += st;
  }
}

static void modeBorderRotate() {
  static uint16_t head = 0;
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last > g_borderSpeedMs) { last = now; head++; }

  fadeToBlackBy(leds, NUM_LEDS, 55);

  for (int k = 0; k < 20; k++) {
    int p = (int)head - k;
    float tt = 1.0f - (float)k / 20.0f;
    CRGB c = CHSV((uint8_t)(millis() / 12 + p * 2), 255, u8(255 * tt));
    leds[perimeterIndexToLed(p)] += c;
  }
}

static void modeBassWave(float bassEnv, float trebEnv) {
  fadeToBlackBy(leds, NUM_LEDS, 35);

  float amp = 0.10f + 0.90f * clamp01(bassEnv);
  uint8_t baseHue = (uint8_t)(millis() / 18);

  for (int i = 0; i < B1_LEN; i++) {
    float phase = (float)i / (float)B1_LEN * 2.0f * PI;
    float w = 0.5f + 0.5f * sinf(phase + millis() * 0.004f);
    uint8_t v = u8(255.0f * clamp01(w * amp));
    leds[B1_S + i] = CHSV(baseHue + (uint8_t)(i * 2), 255, v);
  }

  for (int i = 0; i < B2_LEN; i++) {
    float phase = (float)i / (float)B2_LEN * 2.0f * PI;
    float w = 0.5f + 0.5f * sinf(phase + millis() * 0.004f + PI);
    uint8_t v = u8(255.0f * clamp01(w * amp));
    leds[B2_S + i] = CHSV(baseHue + (uint8_t)(i * 2), 255, v);
  }

  float shimmer = clamp01(trebEnv);
  uint8_t sideV = u8(40 + shimmer * 215.0f);

  for (int i = 0; i < T1_LEN; i++) {
    uint8_t tw = sin8((uint8_t)(millis() / 3 + i * 8));
    uint8_t v = scale8(tw, sideV);
    leds[T1_S + i] += CHSV(baseHue + 96, 220, v);
  }
  for (int i = 0; i < T2_LEN; i++) {
    uint8_t tw = sin8((uint8_t)(millis() / 3 + i * 8 + 128));
    uint8_t v = scale8(tw, sideV);
    leds[T2_S + i] += CHSV(baseHue + 160, 220, v);
  }
}

static void modeVUBorder(float bassEnv, float trebEnv) {
  static float b=0, t=0;
  float s = clamp01((float)g_vuSmoothing / 100.0f);
  float a = lerp1f(0.75f, 0.25f, s);

  b += (bassEnv - b) * a;
  t += (trebEnv - t) * a;

  float bV = powf(clamp01(b), 0.55f);
  float tV = powf(clamp01(t), 0.60f);

  fill_solid(leds, NUM_LEDS, CRGB::Black);

  const int topHalf = B1_LEN / 2;
  const int botHalf = B2_LEN / 2;

  int topFill = (int)roundf(bV * (float)topHalf);
  int botFill = (int)roundf(bV * (float)botHalf);

  CRGB bc = bassColorForLevel(bV);
  bc.nscale8_video(u8(40 + 215 * bV));

  const int topC = B1_S + topHalf;
  const int botC = B2_S + botHalf;

  for (int i = 0; i <= topFill; i++) {
    int L = topC - i; if (L >= B1_S) leds[L] = bc;
    int R = topC + i; if (R <= B1_E) leds[R] = bc;
  }
  for (int i = 0; i <= botFill; i++) {
    int L = botC - i; if (L >= B2_S) leds[L] = bc;
    int R = botC + i; if (R <= B2_E) leds[R] = bc;
  }

  if (topFill > 0) {
    int L = max(B1_S, topC - topFill);
    int R = min(B1_E, topC + topFill);
    leds[L] = CRGB::White;
    leds[R] = CRGB::White;
  }
  if (botFill > 0) {
    int L = max(B2_S, botC - botFill);
    int R = min(B2_E, botC + botFill);
    leds[L] = CRGB::White;
    leds[R] = CRGB::White;
  }

  int leftFill  = (int)roundf(tV * (float)T1_LEN);
  int rightFill = (int)roundf(tV * (float)T2_LEN);

  CRGB tc = trebleColorForLevel(tV);
  tc.nscale8_video(u8(30 + 225 * tV));

  for (int i = 0; i < leftFill; i++) leds[T1_S + i] = tc;
  for (int i = 0; i < rightFill; i++) leds[T2_E - i] = tc;

  if (leftFill > 0)  leds[T1_S + leftFill - 1] = CRGB::White;
  if (rightFill > 0) leds[T2_E - (rightFill - 1)] = CRGB::White;

  if (tV > 0.05f && random8() < (uint8_t)(10 + 60 * tV)) {
    int side = random8(0, 2);
    if (side == 0 && leftFill > 0) leds[T1_S + random16(leftFill)] += CRGB::White;
    if (side == 1 && rightFill > 0) leds[T2_E - (int)random16(rightFill)] += CRGB::White;
  }
}

static void modeCornersHit(bool beatHit, float bassEnv, float trebEnv) {
  static float energy = 0.0f;

  float drive = clamp01(0.65f * bassEnv + 0.35f * trebEnv);
  if (beatHit) drive = clamp01(drive + 0.65f);

  energy += (drive - energy) * 0.55f;
  energy *= 0.92f;

  uint8_t e = u8(energy * 255.0f);

  fadeToBlackBy(leds, NUM_LEDS, 35);

  if (!g_cornerEnable || e < 5) return;

  CRGB c = mainColor();
  c.nscale8_video(e);

  int spread = (int)g_cornerSpread;
  if (spread < 0) spread = 0;

  for (int k = 0; k <= spread; k++) {
    uint8_t fall = (spread == 0) ? 255 : (uint8_t)(255 - (k * 255) / spread);
    uint8_t v = scale8(e, fall);

    CRGB cc = c;
    cc.nscale8_video(v);

    leds[borderFromCornerClockwise(0, k)] += cc;
    leds[borderFromCornerClockwise(1, k)] += cc;
    leds[borderFromCornerClockwise(2, k)] += cc;
    leds[borderFromCornerClockwise(3, k)] += cc;
  }

  if (beatHit) {
    leds[ledBottomLeft()]  += CRGB(80,80,80);
    leds[ledBottomRight()] += CRGB(80,80,80);
    leds[ledTopLeft()]     += CRGB(80,80,80);
    leds[ledTopRight()]    += CRGB(80,80,80);
  }
}

// ================= Web UI: multi-page =================
static String pageStyle() {
  return R"CSS(
body{font-family:Arial;margin:16px;background:#000;color:#eee}
.row{margin:12px 0;padding:12px;border:1px solid #333;border-radius:10px;background:#0b0b0b}
label{display:block;margin:8px 0}
small{color:#aaa}
button{padding:10px 14px;font-size:16px;background:#222;color:#eee;border:1px solid #444;border-radius:8px;margin:4px}
select,input[type=color]{font-size:16px;background:#111;color:#eee;border:1px solid #444;border-radius:6px;padding:4px}
input[type=range]{width:100%}
hr{border:0;border-top:1px solid #333}
.mono{font-family:ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace; white-space:pre; font-size:12px}
.hidden{display:none}
)CSS";
}

static void handleRoot() {
  // NOTE: Fix for "button didn't work" => Flashy toggle is now a BUTTON that calls /toggleFlashy
  String html =
R"HTML(<!doctype html><html><head>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<style>)HTML" + String(pageStyle()) + R"HTML(</style>
</head><body>
<h2>ESP32 Rectangle LED Controller</h2>

<div class='row'>
<label>Page:
<select id=page>
  <option value="main">Main</option>
  <option value="music">Music</option>
  <option value="effects">Effects</option>
  <option value="presets">Presets</option>
  <option value="debug">Debug</option>
</select></label>
</div>

<div id=panel_main class='row'>
<h3>Main</h3>
<label>Enabled: <b id=ev></b></label>
<button id=toggle></button>

<label>Flash Style: <b id=fsv></b></label>
<button id=toggleFlashy></button>

<label>Mode:
<select id=mode>
  <option value="0">Music Reactive (Bars)</option>
  <option value="1">Solid Color</option>
  <option value="2">Circle Chase (bounce)</option>
  <option value="3">Rainbow Madness</option>
  <option value="4">Border Rotate</option>
  <option value="5">Bass Wave</option>
  <option value="6">VU Border</option>
  <option value="7">Corners Hit</option>
</select></label>

<label>Brightness: <span id=bv></span>
<input id=b type=range min=10 max=255></label>

<label>Main Color:
<input id=main type=color></label>
</div>

<div id=panel_music class='row hidden'>
<h3>Music</h3>

<label>Bass Smooth: <span id=bsv></span>
<input id=bsmooth type=range min=0 max=100></label>

<label>Bass Gate: <span id=gv></span>
<input id=g type=range min=0 max=40></label>

<label>Bass Gamma: <span id=gamv></span>
<input id=gam type=range min=10 max=40></label>

<label>Treble Gain: <span id=tgv></span>
<input id=tg type=range min=10 max=300></label>

<hr>
<label>Peak Hold:
<select id=peaken>
  <option value="0">Off</option>
  <option value="1">On</option>
</select></label>

<label>Peak Fall Speed: <span id=peakv></span>
<input id=peak type=range min=5 max=80></label>

<hr>
<label>Bass Color mode:
<select id=bmode>
  <option value="0">Solid</option>
  <option value="1">Gradient by level</option>
</select></label>
<label>Bass Solid: <input id=bcs type=color></label>
<label>Bass Low: <input id=bcl type=color></label>
<label>Bass High: <input id=bch type=color></label>

<hr>
<label>Treble Color mode:
<select id=tmode>
  <option value="0">Solid</option>
  <option value="1">Gradient by level</option>
</select></label>
<label>Treble Solid: <input id=tcs type=color></label>
<label>Treble Low: <input id=tcl type=color></label>
<label>Treble High: <input id=tch type=color></label>
</div>

<div id=panel_effects class='row hidden'>
<h3>Effects</h3>

<label>Strobe intensity (0 = off): <span id=strov></span>
<input id=strobe type=range min=0 max=255></label>

<label>Circle Chase speed (ms): <span id=cspeedv></span>
<input id=cspeed type=range min=8 max=50></label>

<label>Border Rotate speed (ms): <span id=bspeedv></span>
<input id=bspeed type=range min=8 max=60></label>

<label>Rainbow speed: <span id=rspeedv></span>
<input id=rspeed type=range min=1 max=10></label>

<label>VU smoothing: <span id=vusv></span>
<input id=vus type=range min=0 max=100></label>

<hr>
<label>Corners Hit enabled:
<select id=corneren>
  <option value="0">Off</option>
  <option value="1">On</option>
</select></label>

<label>Corner spread: <span id=spreadv></span>
<input id=spread type=range min=0 max=60></label>
</div>

<div id=panel_presets class='row hidden'>
<h3>Presets</h3>
<button onclick="preset('neon')">Neon</button>
<button onclick="preset('fire')">Fire</button>
<button onclick="preset('ice')">Ice</button>
<button onclick="preset('cyber')">Cyberpunk</button>
<button onclick="preset('sunset')">Sunset</button>
<button onclick="preset('matrix')">Matrix</button>
<button onclick="preset('purple')">Purple Haze</button>
<p><small>Presets set Main + Bass/Treble gradients. You can still tweak afterwards.</small></p>
</div>

<div id=panel_debug class='row hidden'>
<h3>Debug</h3>
<div class=mono id=dbg>loading...</div>
</div>

<script>
const $=id=>document.getElementById(id);

function showPage(name){
  ['main','music','effects','presets','debug'].forEach(p=>{
    $('panel_'+p).classList.toggle('hidden', p!==name);
  });
}
$('page').addEventListener('change', ()=>showPage($('page').value));
showPage('main');

function setEnabledLabel(v){
  $('ev').textContent = v ? "ON" : "OFF";
  $('toggle').textContent = v ? "Turn OFF" : "Turn ON";
}

function setFlashyLabel(v){
  $('fsv').textContent = v ? "Flashy (old)" : "No Flash (new)";
  $('toggleFlashy').textContent = v ? "Switch to No Flash" : "Switch to Flashy";
}

function updateLabels(){
  $('bv').textContent = $('b').value;
  $('gv').textContent = (parseInt($('g').value)/100).toFixed(2);
  $('gamv').textContent = (parseInt($('gam').value)/10).toFixed(1);
  $('tgv').textContent = (parseInt($('tg').value)/100).toFixed(2);
  $('bsv').textContent = $('bsmooth').value;
  $('peakv').textContent = $('peak').value;

  $('strov').textContent = $('strobe').value;
  $('cspeedv').textContent = $('cspeed').value;
  $('bspeedv').textContent = $('bspeed').value;
  $('rspeedv').textContent = $('rspeed').value;
  $('vusv').textContent = $('vus').value;

  $('spreadv').textContent = $('spread').value;
}

function setFromDevice(s){
  $('mode').value = s.mode;
  $('b').value = s.b;
  $('main').value = s.main;

  $('bsmooth').value = s.bsmooth;
  $('g').value = s.g;
  $('gam').value = s.gam;
  $('tg').value = s.tg;

  $('peaken').value = s.peaken;
  $('peak').value = s.peak;

  $('bmode').value = s.bmode;
  $('tmode').value = s.tmode;
  $('bcs').value = s.bcs;
  $('bcl').value = s.bcl;
  $('bch').value = s.bch;
  $('tcs').value = s.tcs;
  $('tcl').value = s.tcl;
  $('tch').value = s.tch;

  $('strobe').value = s.strobe;
  $('cspeed').value = s.cspeed;
  $('bspeed').value = s.bspeed;
  $('rspeed').value = s.rspeed;
  $('vus').value = s.vus;

  $('corneren').value = s.corneren;
  $('spread').value = s.spread;

  setEnabledLabel(s.en);
  setFlashyLabel(s.flashy);
  updateLabels();
}

// Debounced send (no flashy here; flashy is changed by the button)
let tmr=null;
function send(){
  if(tmr) clearTimeout(tmr);
  tmr=setTimeout(()=>{
    const q = new URLSearchParams({
      mode:$('mode').value,

      b:$('b').value,
      main:$('main').value,

      bsmooth:$('bsmooth').value,
      g:$('g').value,
      gam:$('gam').value,
      tg:$('tg').value,

      peaken:$('peaken').value,
      peak:$('peak').value,

      bmode:$('bmode').value,
      tmode:$('tmode').value,
      bcs:$('bcs').value, bcl:$('bcl').value, bch:$('bch').value,
      tcs:$('tcs').value, tcl:$('tcl').value, tch:$('tch').value,

      strobe:$('strobe').value,
      cspeed:$('cspeed').value,
      bspeed:$('bspeed').value,
      rspeed:$('rspeed').value,
      vus:$('vus').value,

      corneren:$('corneren').value,
      spread:$('spread').value,
    });

    fetch('/set?'+q.toString())
      .then(()=>fetch('/get')).then(r=>r.json()).then(setFromDevice);

    updateLabels();
  }, 120);
}

// sliders: send on change; selects/colors: instant
const sliderIds = ['b','bsmooth','g','gam','tg','peak','strobe','cspeed','bspeed','rspeed','vus','spread'];
const instantIds = ['mode','main','peaken','bmode','tmode','bcs','bcl','bch','tcs','tcl','tch','corneren','page'];

sliderIds.forEach(id=>$(id).addEventListener('change', send));
instantIds.forEach(id=>{
  $(id).addEventListener('input', send);
  $(id).addEventListener('change', send);
});

$('toggle').addEventListener('click', ()=>{
  fetch('/toggle')
    .then(()=>fetch('/get')).then(r=>r.json()).then(setFromDevice);
});

$('toggleFlashy').addEventListener('click', ()=>{
  fetch('/toggleFlashy')
    .then(()=>fetch('/get')).then(r=>r.json()).then(setFromDevice);
});

function preset(name){
  const P = {
    neon:   { main:"#00FFC8", bL:"#00FF00", bH:"#FF0066", tL:"#00FFFF", tH:"#A000FF" },
    fire:   { main:"#FF5A00", bL:"#FF0000", bH:"#FFD000", tL:"#FF6A00", tH:"#FFFFFF" },
    ice:    { main:"#00A0FF", bL:"#0018FF", bH:"#00FFFF", tL:"#00FFFF", tH:"#FFFFFF" },
    cyber:  { main:"#FF00FF", bL:"#00FF9A", bH:"#FF00FF", tL:"#00D5FF", tH:"#FF4D00" },
    sunset: { main:"#FF2D6A", bL:"#FF7A00", bH:"#FF2D6A", tL:"#7A00FF", tH:"#00D5FF" },
    matrix: { main:"#00FF00", bL:"#003300", bH:"#00FF00", tL:"#00AA00", tH:"#CCFFCC" },
    purple: { main:"#B000FF", bL:"#3C00FF", bH:"#FF00A8", tL:"#00D5FF", tH:"#B000FF" },
  }[name];
  if(!P) return;
  $('main').value = P.main;
  $('bcl').value = P.bL;
  $('bch').value = P.bH;
  $('tcl').value = P.tL;
  $('tch').value = P.tH;
  $('bmode').value = 1;
  $('tmode').value = 1;
  send();
}

async function pollDebug(){
  try{
    const r = await fetch('/debug');
    const j = await r.json();
    $('dbg').textContent =
`bassEnv : ${j.bassEnv.toFixed(3)}
trebEnv : ${j.trebEnv.toFixed(3)}
volEnv  : ${j.volEnv.toFixed(3)}
bassRaw : ${j.bassRaw.toFixed(3)}
trebRaw : ${j.trebRaw.toFixed(3)}
beatHit : ${j.beatHit}
flashy  : ${j.flashy}`;
  }catch(e){}
}
setInterval(pollDebug, 200);

fetch('/get').then(r=>r.json()).then(setFromDevice);
</script>
</body></html>)HTML";

  server.send(200, "text/html", html);
}

static void handleGet() {
  String json = "{";
  json += "\"en\":" + String(g_enabled ? 1 : 0) + ",";
  json += "\"mode\":" + String(g_mode) + ",";
  json += "\"flashy\":" + String(g_flashyMode) + ",";

  json += "\"b\":" + String(g_brightness) + ",";
  json += "\"main\":\"" + toHex(g_mainColor[0], g_mainColor[1], g_mainColor[2]) + "\",";

  json += "\"bsmooth\":" + String(g_bassSmooth) + ",";
  json += "\"g\":" + String((int)(g_bassGate*100)) + ",";
  json += "\"gam\":" + String((int)(g_bassGamma*10)) + ",";
  json += "\"tg\":" + String((int)(g_trebleGain*100)) + ",";

  json += "\"peaken\":" + String(g_peakEnable) + ",";
  json += "\"peak\":" + String(g_peakFallMs) + ",";

  json += "\"bmode\":" + String(g_bassColorMode) + ",";
  json += "\"tmode\":" + String(g_trebleColorMode) + ",";
  json += "\"bcs\":\"" + toHex(g_bassSolid[0], g_bassSolid[1], g_bassSolid[2]) + "\",";
  json += "\"bcl\":\"" + toHex(g_bassLow[0], g_bassLow[1], g_bassLow[2]) + "\",";
  json += "\"bch\":\"" + toHex(g_bassHigh[0], g_bassHigh[1], g_bassHigh[2]) + "\",";
  json += "\"tcs\":\"" + toHex(g_trebleSolid[0], g_trebleSolid[1], g_trebleSolid[2]) + "\",";
  json += "\"tcl\":\"" + toHex(g_trebleLow[0], g_trebleLow[1], g_trebleLow[2]) + "\",";
  json += "\"tch\":\"" + toHex(g_trebleHigh[0], g_trebleHigh[1], g_trebleHigh[2]) + "\",";

  json += "\"strobe\":" + String(g_strobeIntensity) + ",";
  json += "\"cspeed\":" + String(g_circleSpeedMs) + ",";
  json += "\"bspeed\":" + String(g_borderSpeedMs) + ",";
  json += "\"rspeed\":" + String(g_rainbowSpeed) + ",";
  json += "\"vus\":" + String(g_vuSmoothing) + ",";

  json += "\"corneren\":" + String(g_cornerEnable) + ",";
  json += "\"spread\":" + String(g_cornerSpread);

  json += "}";
  server.send(200, "application/json", json);
}

static void handleDebug() {
  String json = "{";
  json += "\"bassEnv\":" + String((float)g_dbg_bassEnv, 4) + ",";
  json += "\"trebEnv\":" + String((float)g_dbg_trebEnv, 4) + ",";
  json += "\"volEnv\":"  + String((float)g_dbg_volEnv, 4) + ",";
  json += "\"bassRaw\":" + String((float)g_dbg_bassRaw, 4) + ",";
  json += "\"trebRaw\":" + String((float)g_dbg_trebRaw, 4) + ",";
  json += "\"beatHit\":" + String((int)g_dbg_beatHit) + ",";
  json += "\"flashy\":" + String((int)g_flashyMode);
  json += "}";
  server.send(200, "application/json", json);
}

static void handleSet() {
  if (server.hasArg("mode")) g_mode = (uint8_t)server.arg("mode").toInt();

  if (server.hasArg("b")) g_brightness = (uint8_t)server.arg("b").toInt();
  FastLED.setBrightness(g_brightness);

  if (server.hasArg("main")) {
    uint32_t rgb = parseHexColor(server.arg("main"));
    g_mainColor[0] = (rgb >> 16) & 0xFF;
    g_mainColor[1] = (rgb >> 8) & 0xFF;
    g_mainColor[2] = (rgb) & 0xFF;
  }

  if (server.hasArg("bsmooth")) g_bassSmooth = (uint8_t)server.arg("bsmooth").toInt();
  if (server.hasArg("g"))   g_bassGate   = server.arg("g").toFloat() / 100.0f;
  if (server.hasArg("gam")) g_bassGamma  = server.arg("gam").toFloat() / 10.0f;
  if (server.hasArg("tg"))  g_trebleGain = server.arg("tg").toFloat() / 100.0f;

  if (server.hasArg("peaken")) g_peakEnable = (uint8_t)server.arg("peaken").toInt();
  if (server.hasArg("peak"))   g_peakFallMs = (uint8_t)server.arg("peak").toInt();

  if (server.hasArg("bmode")) g_bassColorMode = (uint8_t)server.arg("bmode").toInt();
  if (server.hasArg("tmode")) g_trebleColorMode = (uint8_t)server.arg("tmode").toInt();

  auto loadRGB = [&](const char* key, volatile uint8_t dst[3]) {
    if (!server.hasArg(key)) return;
    uint32_t rgb = parseHexColor(server.arg(key));
    dst[0] = (rgb >> 16) & 0xFF;
    dst[1] = (rgb >> 8)  & 0xFF;
    dst[2] = (rgb)       & 0xFF;
  };

  loadRGB("bcs", g_bassSolid);
  loadRGB("bcl", g_bassLow);
  loadRGB("bch", g_bassHigh);

  loadRGB("tcs", g_trebleSolid);
  loadRGB("tcl", g_trebleLow);
  loadRGB("tch", g_trebleHigh);

  if (server.hasArg("strobe")) g_strobeIntensity = (uint8_t)server.arg("strobe").toInt();
  if (server.hasArg("cspeed")) g_circleSpeedMs = (uint8_t)server.arg("cspeed").toInt();
  if (server.hasArg("bspeed")) g_borderSpeedMs = (uint8_t)server.arg("bspeed").toInt();
  if (server.hasArg("rspeed")) g_rainbowSpeed = (uint8_t)server.arg("rspeed").toInt();
  if (server.hasArg("vus"))    g_vuSmoothing = (uint8_t)server.arg("vus").toInt();

  if (server.hasArg("corneren")) g_cornerEnable = (uint8_t)server.arg("corneren").toInt();
  if (server.hasArg("spread"))   g_cornerSpread = (uint8_t)server.arg("spread").toInt();

  server.send(200, "text/plain", "OK");
}

static void handleToggle() {
  g_enabled = !g_enabled;
  if (!g_enabled) {
    FastLED.clear(true);
  } else {
    runStartupAnimation();
  }
  server.send(200, "text/plain", "OK");
}

static void handleToggleFlashy() {
  g_flashyMode = (g_flashyMode == 0) ? 1 : 0;
  server.send(200, "text/plain", "OK");
}

// ================= Setup / Loop =================
void setup() {
  Serial.begin(115200);
  delay(200);

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(g_brightness);
  FastLED.clear(true);

  WiFi.mode(WIFI_STA);
  WiFi.setHostname("AudioBarsESP32");
  WiFi.disconnect(true, true);
  delay(200);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) delay(250);

  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/get", handleGet);
  server.on("/set", handleSet);
  server.on("/toggle", handleToggle);
  server.on("/toggleFlashy", handleToggleFlashy);
  server.on("/debug", handleDebug);
  server.begin();

  setupI2S();
  runStartupAnimation();
}

void loop() {
  server.handleClient();

  if (!g_enabled) {
    delay(10);
    return;
  }

  bool beatHit = false;
  float bassEnv=0, trebEnv=0, volEnv=0, boomEnv=0;
  float bassRaw01=0, trebRaw01=0;

  if (g_mode == 0 || g_mode == 3 || g_mode == 5 || g_mode == 6 || g_mode == 7) {
    if (readBlock()) {
      computeFeatures(bassEnv, trebEnv, volEnv, beatHit, boomEnv, bassRaw01, trebRaw01);
    }
  }

  // publish debug values
  g_dbg_bassEnv = bassEnv;
  g_dbg_trebEnv = trebEnv;
  g_dbg_volEnv  = volEnv;
  g_dbg_bassRaw = bassRaw01;
  g_dbg_trebRaw = trebRaw01;
  g_dbg_beatHit = beatHit ? 1 : 0;

  // Quieter music = dimmer LEDs (always on, with more smoothing so it doesn't "flash")
  static float volSm = 0.0f;
  volSm += (volEnv - volSm) * 0.06f;
  float v = powf(clamp01(volSm), 0.75f);
  uint8_t dynB = (uint8_t)lerp1f((float)g_brightness * 0.18f, (float)g_brightness, v);
  FastLED.setBrightness(dynB);

  // beat envelope only used when Flashy is enabled (and also smoothed)
  static float beatEnv = 0.0f;
  float beatTarget = beatHit ? 0.50f : 0.0f;
  beatEnv += (beatTarget - beatEnv) * (beatHit ? 0.70f : 0.22f);

  FastLED.clear(false);

  switch (g_mode) {
    case 0: { // Music Reactive (Bars + Peak Hold)
      CRGB bassC   = bassColorForLevel(powf(clamp01(bassRaw01), 0.70f));
      CRGB trebleC = trebleColorForLevel(powf(clamp01(trebRaw01), 0.60f));

      static float t1Bar = 0.0f;
      static float t2Bar = 0.0f;

      renderTrebleForward(T1_S, T1_LEN, trebEnv, trebleC, t1Bar);

      renderBassMirror(B1_S, B1_LEN, bassEnv, bassC, beatEnv, boomEnv, g_flashyMode != 0);

      renderTrebleReverse(T2_S, T2_LEN, trebEnv, trebleC, t2Bar);

      renderBassMirror(B2_S, B2_LEN, bassEnv, bassC, beatEnv, boomEnv, g_flashyMode != 0);

      static float peakT1=0, peakT2=0;
      static uint32_t lastFall1=0, lastFall2=0;
      drawPeakHoldForward(T1_S, T1_LEN, trebEnv, peakT1, lastFall1);
      drawPeakHoldReverse(T2_S, T2_LEN, trebEnv, peakT2, lastFall2);
      break;
    }

    case 1:
      modeSolid();
      break;

    case 2:
      modeCircleChaseBounce();
      break;

    case 3:
      modeRainbowMadness(beatHit);
      break;

    case 4:
      modeBorderRotate();
      break;

    case 5:
      modeBassWave(bassEnv, trebEnv);
      break;

    case 6:
      modeVUBorder(bassEnv, trebEnv);
      break;

    case 7:
      modeCornersHit(beatHit, bassEnv, trebEnv);
      break;

    default:
      modeSolid();
      break;
  }

  FastLED.show();
}