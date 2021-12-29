// Libraries:
// Adafruit Neopixel 1.10.0x

#include <Adafruit_NeoPixel.h>

#include <SPI.h>

#include "ble.h"
#include "config.h"
#include "delay_inserter.h"
#include "homing_controller.h"
#include "stepper_control.h"

// Pin definitions ------------------------------------------------
constexpr int kLedData = 21;
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

void setup() {
  pinMode(kLedData, OUTPUT);
  
  g_rgb_led.begin();
  Serial.begin(115200);

  delay(100);

  Serial.println("Setting up");

  g_motion_controller.Begin();
}

void loop() {
  auto t = millis();

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

  if (homing_controller.Done()) {
    MoveToMiddle(&g_motion_controller, homing_controller.Range());
  } else {
    homing_controller.Update();
  }

  // Run the control loop at approx 200 Hz.
  // This is only for velocity updates. Stepping happens asynchronously.
  // We put the delay inserter here because this is the most crucial part
  // for loop timing.
  static DelayInserter<5000> delay_inserter;
  delay_inserter.Sync();
  g_motion_controller.Update();
}
