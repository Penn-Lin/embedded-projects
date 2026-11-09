/**
 * 磁流体智能音响 — 完整版
 * Core 0: 蓝牙 A2DP 音频接收
 * Core 1: 磁流体视觉引擎 + 陀螺仪
 *
 * I2S:  BCK=27  DIN=26  LCK=25
 * SPI:  MOSI=23 SCLK=18 CS=5  DC=17 RES=16
 * I2C:  SDA=21  SCL=22
 * XSMT: GPIO14 (PCM5102A 静音控制)
 */

#include "AudioTools.h"
#include "BluetoothA2DPSink.h"
#include <TFT_eSPI.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ══════════════════════════════════════════
// 音频对象
// ══════════════════════════════════════════
I2SStream i2s;
BluetoothA2DPSink a2dp_sink(i2s);

// ── 音频分析缓冲 ──────────────────────────
#define SAMPLE_SIZE 256
int16_t  sampleBuf[SAMPLE_SIZE];
volatile int  sampleCount = 0;
volatile bool newSamples  = false;

// ── XSMT 静音控制 ─────────────────────────
#define PIN_XSMT 14
volatile bool btPlaying   = false;
uint32_t      lastAudioTime = 0;

// ══════════════════════════════════════════
// 屏幕 / 陀螺仪
// ══════════════════════════════════════════
TFT_eSPI    tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);
Adafruit_MPU6050 mpu;

#define SCREEN_W 240
#define SCREEN_H 240
#define CX       120.0f
#define CY       120.0f

// ══════════════════════════════════════════
// 磁流体参数
// ══════════════════════════════════════════
#define NUM_ARMS      5
#define ARM_R_BASE    18.0f
#define ARM_DIST_BASE 52.0f
#define CORE_R_BASE   28.0f

float armAngle[NUM_ARMS];
float armAngleTarget[NUM_ARMS];
float armDist[NUM_ARMS];
float armRadius[NUM_ARMS];
float armPhase[NUM_ARMS];

float coreRadius       = CORE_R_BASE;
float coreRadiusTarget = CORE_R_BASE;

float ringRadius       = 95.0f;
float ringRadiusTarget = 95.0f;
float ringColorPhase   = 0.0f;

// ── 音频能量 ──────────────────────────────
float bassEnergy = 0.3f;
float midEnergy  = 0.0f;
float highEnergy = 0.0f;

// ── 陀螺仪 & 状态 ─────────────────────────
float    gravX = 0.0f, gravY = 0.0f;
float    lastGx = 0,   lastGy = 0;
bool     deviceStill    = false;
uint32_t stillStartTime = 0;
float    stateTransition = 0.0f;   // 0=动态 1=静止

#define STILL_THRESHOLD 0.08f
#define STILL_DELAY     2000

// ══════════════════════════════════════════
// 液滴系统
// ══════════════════════════════════════════
#define NUM_DROPLETS 30
float    dlX[NUM_DROPLETS], dlY[NUM_DROPLETS];
float    dlVx[NUM_DROPLETS], dlVy[NUM_DROPLETS];
bool     dlActive[NUM_DROPLETS];
uint32_t lastSpawnTime = 0;

// ── FPS ───────────────────────────────────
uint32_t frameCount = 0, lastFpsTime = 0;

// ══════════════════════════════════════════
// 前向声明
// ══════════════════════════════════════════
void renderRing();
void renderFluid();
void renderStandby();
void renderDroplets();
void updatePhysics();
void analyzeAudio();
void updateDroplets();
void spawnDroplet(float x, float y, float vx, float vy);

// ══════════════════════════════════════════
// 音频回调（Core 0）
// ══════════════════════════════════════════
void audioDataCallback(const uint8_t *data, uint32_t len) {
  // 收到数据时解除静音
  if (!btPlaying) {
    digitalWrite(PIN_XSMT, HIGH);
    btPlaying = true;
  }
  lastAudioTime = millis();

  int16_t *samples = (int16_t*)data;
  int count = len / 4;
  for (int i = 0; i < count && sampleCount < SAMPLE_SIZE; i++) {
    sampleBuf[sampleCount++] = (samples[i*2] / 2 + samples[i*2+1] / 2);
  }
  if (sampleCount >= SAMPLE_SIZE) newSamples = true;
}

// ══════════════════════════════════════════
// 音频能量分析
// ══════════════════════════════════════════
void analyzeAudio() {
  if (!newSamples) return;
  newSamples  = false;
  sampleCount = 0;

  float bass = 0, mid = 0, high = 0;
  int bassEnd = SAMPLE_SIZE / 8;
  int midEnd  = SAMPLE_SIZE / 2;

  for (int i = 0; i < SAMPLE_SIZE; i++) {
    float v = fabsf((float)sampleBuf[i]) / 32768.0f;
    if      (i < bassEnd) bass += v;
    else if (i < midEnd)  mid  += v;
    else                  high += v;
  }

  bass /= bassEnd;
  mid  /= (midEnd - bassEnd);
  high /= (SAMPLE_SIZE - midEnd);

  bassEnergy = bassEnergy * 0.6f + bass * 0.4f * 4.0f;
  midEnergy  = midEnergy  * 0.6f + mid  * 0.4f * 3.0f;
  highEnergy = highEnergy * 0.6f + high * 0.4f * 3.0f;

  bassEnergy = constrain(bassEnergy, 0.0f, 1.0f);
  midEnergy  = constrain(midEnergy,  0.0f, 1.0f);
  highEnergy = constrain(highEnergy, 0.0f, 1.0f);
}

// ══════════════════════════════════════════
// 发光环颜色：蓝→紫→粉
// ══════════════════════════════════════════
uint16_t ringColor(float t, float brightness) {
  t = constrain(t, 0.0f, 1.0f);
  brightness = constrain(brightness, 0.0f, 1.0f);
  uint8_t r, g, b;
  if (t < 0.5f) {
    float s = t / 0.5f;
    r = (uint8_t)(80  + 100 * s);
    g = (uint8_t)(60  * (1.0f - s * 0.5f));
    b = (uint8_t)(220 - 20  * s);
  } else {
    float s = (t - 0.5f) / 0.5f;
    r = (uint8_t)(180 + 60  * s);
    g = (uint8_t)(30  + 80  * s);
    b = (uint8_t)(200 - 60  * s);
  }
  return spr.color565(
    (uint8_t)(r * brightness),
    (uint8_t)(g * brightness),
    (uint8_t)(b * brightness)
  );
}

// ══════════════════════════════════════════
// 物理更新
// ══════════════════════════════════════════
void updatePhysics() {
  uint32_t t = millis();

  ringRadiusTarget = 88.0f + bassEnergy * 18.0f;
  ringRadius += (ringRadiusTarget - ringRadius) * 0.12f;

  ringColorPhase += midEnergy * 0.008f + 0.002f;
  if (ringColorPhase > 1.0f) ringColorPhase -= 1.0f;

  coreRadiusTarget = CORE_R_BASE + bassEnergy * 12.0f;
  coreRadius += (coreRadiusTarget - coreRadius) * 0.18f;

  for (int i = 0; i < NUM_ARMS; i++) {
    float noise =
      sinf(t * 0.004f + armPhase[i]) * 0.6f +
      sinf(t * 0.009f + armPhase[i] * 1.7f) * 0.4f;

    armAngleTarget[i] = (float)i / NUM_ARMS * 2.0f * M_PI
                        + noise * 0.35f
                        + t * 0.0003f;
    armAngle[i] += (armAngleTarget[i] - armAngle[i]) * 0.06f;

    float distNoise = sinf(t * 0.006f + armPhase[i] * 2.3f) * 0.3f + 0.7f;
    armDist[i] = ARM_DIST_BASE * distNoise + bassEnergy * 22.0f;

    float sizeNoise = sinf(t * 0.015f + armPhase[i]) * 0.2f + 0.8f;
    armRadius[i] = ARM_R_BASE * sizeNoise + bassEnergy * 8.0f;

    armPhase[i] += 0.002f;
  }
}

// ══════════════════════════════════════════
// 液滴系统
// ══════════════════════════════════════════
void spawnDroplet(float x, float y, float vx, float vy) {
  for (int i = 0; i < NUM_DROPLETS; i++) {
    if (!dlActive[i]) {
      dlX[i] = x;  dlY[i] = y;
      dlVx[i] = vx; dlVy[i] = vy;
      dlActive[i] = true;
      return;
    }
  }
}

void updateDroplets() {
  uint32_t now = millis();

  if (stateTransition < 0.5f) {
    // 动态模式：随机生成，跟随重力落下
    if (now - lastSpawnTime > 80) {
      lastSpawnTime = now;
      float spawnX = random(10, 230);
      float spawnY = random(10, 230);
      float spd = 1.5f + random(0, 20) * 0.15f;
      spawnDroplet(spawnX, spawnY,
                   gravX * spd + random(-10, 10) * 0.1f,
                   gravY * spd + random(-10, 10) * 0.1f);
    }
  } else {
    // 待机模式：从中心圆边缘向外喷出再落下
    if (now - lastSpawnTime > 120) {
      lastSpawnTime = now;
      float angle = random(0, 628) * 0.01f;
      float edge  = coreRadius + 5.0f;
      float sx = CX + cosf(angle) * edge;
      float sy = CY + sinf(angle) * edge;
      float outSpd = 0.8f + random(0, 10) * 0.1f;
      spawnDroplet(sx, sy,
                   cosf(angle) * outSpd,
                   sinf(angle) * outSpd);
    }
  }

  for (int i = 0; i < NUM_DROPLETS; i++) {
    if (!dlActive[i]) continue;

    dlVx[i] += gravX * 0.4f;
    dlVy[i] += gravY * 0.4f;
    dlVx[i] *= 0.97f;
    dlVy[i] *= 0.97f;
    dlX[i]  += dlVx[i];
    dlY[i]  += dlVy[i];

    if (dlX[i] < 0 || dlX[i] > 240 ||
        dlY[i] < 0 || dlY[i] > 240) {
      dlActive[i] = false;
    }
  }
}

void renderDroplets() {
  for (int i = 0; i < NUM_DROPLETS; i++) {
    if (!dlActive[i]) continue;
    float spd = sqrtf(dlVx[i]*dlVx[i] + dlVy[i]*dlVy[i]);
    uint16_t c     = spr.color565(80, 160, 255);
    uint16_t cTail = spr.color565(30, 80,  180);
    spr.fillCircle((int)dlX[i], (int)dlY[i], 2, c);
    if (spd > 1.2f) {
      spr.fillCircle(
        (int)(dlX[i] - dlVx[i]),
        (int)(dlY[i] - dlVy[i]),
        1, cTail
      );
    }
  }
}

// ══════════════════════════════════════════
// 渲染：发光圆环背景
// ══════════════════════════════════════════
void renderRing() {
  float pulse = 0.85f + 0.15f * sinf(millis() * 0.004f);
  spr.fillCircle((int)CX, (int)CY, (int)(ringRadius * 1.18f),
    ringColor(ringColorPhase, 0.10f * pulse));
  spr.fillCircle((int)CX, (int)CY, (int)(ringRadius * 1.08f),
    ringColor(ringColorPhase, 0.25f * pulse));
  spr.fillCircle((int)CX, (int)CY, (int)(ringRadius * 1.00f),
    ringColor(ringColorPhase, 0.55f));
  spr.fillCircle((int)CX, (int)CY, (int)(ringRadius * 0.90f),
    ringColor(ringColorPhase + 0.1f, 0.80f));
  // 内部深色背景
  spr.fillCircle((int)CX, (int)CY, (int)(ringRadius * 0.78f),
    spr.color565(8, 8, 15));
}

// ══════════════════════════════════════════
// 渲染：磁流体主体
// ══════════════════════════════════════════
void renderFluid() {
  uint16_t fluidMid   = spr.color565(60,  100, 220);
  uint16_t fluidLight = spr.color565(120, 170, 255);
  uint16_t fluidShine = spr.color565(210, 230, 255);

  // 触角
  for (int i = 0; i < NUM_ARMS; i++) {
    float ax = CX + cosf(armAngle[i]) * armDist[i];
    float ay = CY + sinf(armAngle[i]) * armDist[i];
    int   ar = (int)armRadius[i];

    // 颈部连接
    float nx   = ax - CX, ny = ay - CY;
    float dist = sqrtf(nx*nx + ny*ny);
    float ux = nx/dist, uy = ny/dist;
    int neckSteps = (int)(dist / 5.0f);
    for (int s = 1; s < neckSteps; s++) {
      float tt = (float)s / neckSteps;
      float nr = coreRadius * (1.0f - tt) + armRadius[i] * tt;
      nr *= 0.65f;
      float pinch = 1.0f - sinf(tt * M_PI) * 0.3f;
      nr *= pinch;
      int nx2 = (int)(CX + ux * dist * tt);
      int ny2 = (int)(CY + uy * dist * tt);
      spr.fillCircle(nx2, ny2, max(3, (int)nr), fluidMid);
    }

    // 触角球
    spr.fillCircle((int)ax, (int)ay, ar, fluidMid);
    int hlX = (int)(ax - ar * 0.3f);
    int hlY = (int)(ay - ar * 0.3f);
    spr.fillCircle(hlX, hlY, max(2, (int)(ar * 0.35f)), fluidLight);
    spr.fillCircle(hlX, hlY, max(1, (int)(ar * 0.15f)), fluidShine);
  }

  // 核心球
  int cr = (int)coreRadius;
  spr.fillCircle((int)CX, (int)CY, cr, fluidMid);
  int chlX = (int)(CX - cr * 0.28f);
  int chlY = (int)(CY - cr * 0.28f);
  spr.fillCircle(chlX, chlY, (int)(cr * 0.38f), fluidLight);
  spr.fillCircle(chlX, chlY, (int)(cr * 0.18f), fluidShine);
  spr.fillCircle(chlX, chlY, (int)(cr * 0.08f), TFT_WHITE);
}

// ══════════════════════════════════════════
// 渲染：待机呼吸球
// ══════════════════════════════════════════
void renderStandby() {
  float breath = 0.5f + 0.5f * sinf(millis() * 0.0012f);
  float r = 35.0f + 20.0f * breath;

  spr.fillCircle((int)CX, (int)CY, (int)(r * 2.0f),  spr.color565(0,  5,  25));
  spr.fillCircle((int)CX, (int)CY, (int)(r * 1.7f),  spr.color565(0, 15,  55));
  spr.fillCircle((int)CX, (int)CY, (int)(r * 1.4f),  spr.color565(0, 35, 100));
  spr.fillCircle((int)CX, (int)CY, (int)(r * 1.15f), spr.color565(0, 70, 170));
  spr.fillCircle((int)CX, (int)CY, (int)r,            spr.color565(40, 130, 240));
  spr.fillCircle((int)CX, (int)CY, (int)(r * 0.5f),  spr.color565(140, 200, 255));
  spr.fillCircle((int)CX, (int)CY, (int)(r * 0.2f),  TFT_WHITE);
}

// ══════════════════════════════════════════
// Setup
// ══════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("=== 磁流体音响启动 ===");

  // XSMT 默认静音
  pinMode(PIN_XSMT, OUTPUT);
  digitalWrite(PIN_XSMT, LOW);

  // 陀螺仪
  Wire.begin(21, 22);
  if (!mpu.begin(0x68, &Wire))
    Serial.println("MPU6050 未找到");
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);

  // ① 屏幕和 Sprite 先初始化（必须在蓝牙之前）
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  spr.setColorDepth(8);
  spr.createSprite(SCREEN_W, SCREEN_H);
  Serial.printf("Sprite 创建后空闲堆: %d bytes\n", ESP.getFreeHeap());

  // 初始化触角
  for (int i = 0; i < NUM_ARMS; i++) {
    armAngle[i]       = (float)i / NUM_ARMS * 2.0f * M_PI;
    armAngleTarget[i] = armAngle[i];
    armDist[i]        = ARM_DIST_BASE;
    armRadius[i]      = ARM_R_BASE;
    armPhase[i]       = (float)i * 1.27f;
  }
  memset(dlActive, 0, sizeof(dlActive));

  // ② 蓝牙后启动
  auto cfg = i2s.defaultConfig();
  cfg.pin_bck  = 27;
  cfg.pin_ws   = 25;
  cfg.pin_data = 26;
  i2s.begin(cfg);

  a2dp_sink.set_stream_reader(audioDataCallback, true);
  a2dp_sink.start("流沙音响");

  lastFpsTime = millis();
  Serial.println("启动完成，等待蓝牙连接...");
}

// ══════════════════════════════════════════
// Loop
// ══════════════════════════════════════════
void loop() {
  // 读陀螺仪
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  gravX = -a.acceleration.x * 0.1f;
  gravY =  a.acceleration.y * 0.1f;

  float gDelta = fabsf(gravX - lastGx) + fabsf(gravY - lastGy);
  lastGx = gravX;
  lastGy = gravY;

  if (gDelta > STILL_THRESHOLD) {
    stillStartTime = millis();
    deviceStill = false;
  } else if (millis() - stillStartTime > STILL_DELAY) {
    deviceStill = true;
  }

  if (deviceStill) stateTransition = min(stateTransition + 0.03f, 1.0f);
  else             stateTransition = max(stateTransition - 0.08f, 0.0f);

  // 超过 2 秒没有音频数据自动静音
  if (btPlaying && millis() - lastAudioTime > 2000) {
    digitalWrite(PIN_XSMT, LOW);
    btPlaying = false;
  }

  analyzeAudio();
  updatePhysics();
  updateDroplets();

  spr.fillSprite(TFT_BLACK);

  if (stateTransition > 0.95f) {
    renderStandby();
  } else {
    renderRing();
    renderFluid();
  }

  renderDroplets();
  spr.pushSprite(0, 0);

  // FPS 串口输出
  frameCount++;
  uint32_t now = millis();
  if (now - lastFpsTime >= 1000) {
    float fps = frameCount * 1000.0f / (now - lastFpsTime);
    frameCount  = 0;
    lastFpsTime = now;
    Serial.printf("FPS:%.1f bass:%.2f mid:%.2f high:%.2f bt:%s\n",
                  fps, bassEnergy, midEnergy, highEnergy,
                  btPlaying ? "播放中" : "静音");
  }
}
