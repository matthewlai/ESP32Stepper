// Libraries:
// Adafruit Neopixel 1.10.0x

#include <Adafruit_NeoPixel.h>

#include <SPI.h>

#include "ble.h"
#include "config.h"
#include "crash.h"
#include "delay_inserter.h"
#include "homing_controller.h"
#include "stepper_control.h"

// Pin definitions ------------------------------------------------
constexpr int kLedData = 21;

#ifdef HAS_PAIR_SWITCH
constexpr int kPairSw = 22;
#endif
// ----------------------------------------------------------------

// Other Constants ------------------------------------------------
// ----------------------------------------------------------------

// Globals --------------------------------------------------------
Adafruit_NeoPixel g_rgb_led(/*count=*/1, kLedData);

MotionController g_motion_controller;
// ----------------------------------------------------------------

// Example Motion Update Commands ---------------------------------

void MoveToMiddle(MotionController* motion_controller, int32_t range) {
  int32_t target = range * 0.5f;
  if (fabs(motion_controller->GetCurrentPosition() - target) > kPositionTolerance * range) {
    motion_controller->UpdateTargetSpeedByPosition(target, kMaxSpeed);
  } else {
    motion_controller->SetTargetSpeed(0.0f);
  }
}

void MoveAtConstantSpeed(MotionController* motion_controller, float v) {
  motion_controller->SetTargetSpeed(v);
}

// Oscillate as fast as possible between x1 and x2 positions.
void Oscillate(MotionController* motion_controller, int32_t range, int32_t x1, int32_t x2) {
  static int32_t current_target = x1;
  auto current_position = motion_controller->GetCurrentPosition();
  if (fabs(current_position - current_target) < kPositionTolerance * range) {
    float dist_to_x1 = fabs(current_position - x1);
    float dist_to_x2 = fabs(current_position - x2);
    if (dist_to_x1 > dist_to_x2) {
      current_target = x1;
    } else {
      current_target = x2;
    }
  }
  motion_controller->UpdateTargetSpeedByPosition(current_target, kMaxSpeed);
}

// Time based velocity oscillation without homing.
void Oscillate(MotionController* motion_controller, float v, float period_s) {
  float time_now = float(micros()) / 1000000.0f;
  float scale = sin(2.0f * 3.14f / period_s * time_now);
  motion_controller->SetTargetSpeed(v * scale);
}

// Switching direction as fast as possible.
void AbruptOscillate(MotionController* motion_controller, float v, float period_s) {
  float time_now = float(micros()) / 1000000.0f;
  float mod = fmod(time_now, period_s);
  float dir = (mod > (period_s / 2)) ? 1.0f : -1.0f;
  motion_controller->SetTargetSpeed(dir * v);
}

// ----------------------------------------------------------------

void SetRGBLedColour(float r, float g, float b) {
  static int last_r = 0;
  static int last_g = 0;
  static int last_b = 0;
  int new_r = int(r * 255);
  int new_g = int(g * 255);
  int new_b = int(b * 255);
  if (last_r != new_r || last_g != new_g || last_b != new_b) {
    g_rgb_led.setPixelColor(0, int(r * 255), int(g * 255), int(b * 255));
    g_rgb_led.show();
    last_r = new_r;
    last_g = new_g;
    last_b = new_b;
  }
}

void CrashHandler(const String& msg) {
  g_motion_controller.Stop();
  for (;;) {
    Serial.println(msg);
    SetRGBLedColour(0.1f, 0.0f, 0.0f);
    delay(500);
    SetRGBLedColour(0.0f, 0.0f, 0.0f);
    delay(500);
  }
}

void setup() {
  pinMode(kLedData, OUTPUT);

  #ifdef HAS_PAIR_SWITCH
  pinMode(kPairSw, INPUT);
  #endif
  
  g_rgb_led.begin();
  Serial.begin(115200);

  delay(100);

  Serial.println("Setting up");

  g_motion_controller.Begin();
}

void loop() {
  auto t = millis();

  #ifdef HAS_PAIR_SWITCH
  if (digitalRead(kPairSw) == LOW) {
    g_motion_controller.Stop();
    for (;;) {}
  }
  #endif

  constexpr float kPeriodMs = 1000.0f;
  constexpr float kBrightness = 0.1f;
  constexpr float kCoeff = (2 * M_PI) / kPeriodMs;
  float r = sin(kCoeff * t) * 0.5f + 0.5f;
  float g = sin(kCoeff * t + (0.33f * 2.0f * M_PI)) * 0.5f + 0.5f;
  float b = sin(kCoeff * t + (0.66f * 2.0f * M_PI)) * 0.5f + 0.5f;
  SetRGBLedColour(kBrightness * r, kBrightness * g, kBrightness * b);

  static WoodpeckerBLEServer ble_server;

  static bool last_run_setting = false;
  bool run_setting = ble_server.Started();

  if (last_run_setting != run_setting) {
    Serial.print("Started=");
    Serial.println(run_setting);
    last_run_setting = run_setting;
  }

  static HomingController homing_controller(&g_motion_controller);

  if (kUseStallGuardHoming && !homing_controller.Done()) {
    homing_controller.Update();
  } else {
    Oscillate(&g_motion_controller, homing_controller.Range(), homing_controller.Range() * 0.3f, homing_controller.Range() * 0.7f);
    //MoveToMiddle(&g_motion_controller, homing_controller.Range());
    //MoveAtConstantSpeed(&g_motion_controller, 60.0f);
    //Oscillate(&g_motion_controller, kMaxSpeed, 2.0f);
    //AbruptOscillate(&g_motion_controller, kMaxSpeed, 5.0f);
  }

  // Run the control loop at approx 200 Hz.
  // This is only for velocity updates. Stepping happens asynchronously.
  // We put the delay inserter here because this is the most crucial part
  // for loop timing.
  static DelayInserter<5000> delay_inserter;
  delay_inserter.Sync();
  g_motion_controller.Update();
}
