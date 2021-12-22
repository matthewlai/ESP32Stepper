// Libraries:
// Adafruit Neopixel 1.10.0x

#include <Adafruit_NeoPixel.h>

#include <SPI.h>

#include "ble.h"
#include "config.h"
#include "delay_inserter.h"
#include "stepper_control.h"

// Pin definitions ------------------------------------------------
constexpr int kLedData = 21;
// ----------------------------------------------------------------

// Other Constants ------------------------------------------------
// ----------------------------------------------------------------

// Globals --------------------------------------------------------
Adafruit_NeoPixel g_rgb_led(/*count=*/1, kLedData);

TMC2590Controller g_stepper_controller;
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

  g_stepper_controller.Begin();
}

enum class HomingPhase {
  kStart,
  kForward,
  kBackward,
  kDone
};

void loop() {
  auto t = millis();

  constexpr float kPeriodMs = 1000.0f;
  constexpr float kBrightness = 0.1f;
  constexpr float kCoeff = (2 * M_PI) / kPeriodMs;
  float r = sin(kCoeff * t) * 0.5f + 0.5f;
  float g = sin(kCoeff * t + (0.33f * 2.0f * M_PI)) * 0.5f + 0.5f;
  float b = sin(kCoeff * t + (0.66f * 2.0f * M_PI)) * 0.5f + 0.5f;
  SetRGBLedColour(kBrightness * r, kBrightness * g, kBrightness * b);

  static HomingPhase homing_phase = HomingPhase::kStart;

  static WoodpeckerBLEServer ble_server;

  static bool last_run_setting = false;
  bool run_setting = ble_server.Started();

  if (last_run_setting != run_setting) {
    Serial.print("Start is now: ");
    Serial.println(run_setting);
    last_run_setting = run_setting;
  }

  static int32_t forward_position = 0;
  static int32_t backward_position = 0;
  static int32_t range = 0;

  switch (homing_phase) {
   case HomingPhase::kStart:
    // Disable stallguard filtering for faster response.
    g_stepper_controller.SetStallGuardFiltering(false);
    g_stepper_controller.SetAccelerationLimit(kHomingAcceleration);
    g_stepper_controller.SetJerkLimit(kHomingMaxJerk);
    g_stepper_controller.SetMotorCurrent(kHomingCurrent);
    homing_phase = HomingPhase::kForward;
    break;
   case HomingPhase::kForward:
    g_stepper_controller.SetTargetSpeedRPM(kHomingSpeedRPM);
    if ((g_stepper_controller.GetCurrentSpeedRPM() > (kHomingSpeedRPM * 0.99f)) &&
        g_stepper_controller.IsStalled()) {
      forward_position = g_stepper_controller.GetCurrentPosition();
      g_stepper_controller.SetTargetSpeedRPM(-kHomingSpeedRPM);
      homing_phase = HomingPhase::kBackward;
    }
    break;
   case HomingPhase::kBackward:
   g_stepper_controller.SetTargetSpeedRPM(-kHomingSpeedRPM);
    if ((g_stepper_controller.GetCurrentSpeedRPM() < -(kHomingSpeedRPM * 0.99f)) &&
        g_stepper_controller.IsStalled()) {
      backward_position = g_stepper_controller.GetCurrentPosition();
      range = forward_position - backward_position;
      g_stepper_controller.SetTargetSpeedRPM(0.0f);
      g_stepper_controller.SetCurrentPosition(0);
      g_stepper_controller.SetStallGuardFiltering(true);
      g_stepper_controller.SetAccelerationLimit(kMaxAcceleration);
      g_stepper_controller.SetJerkLimit(kMaxJerk);
      g_stepper_controller.SetMotorCurrent(kMotorCurrentLimit);
      homing_phase = HomingPhase::kDone;
      Serial.println("Homing Done!");
      Serial.print("Forward Position: ");
      Serial.println(forward_position);
      Serial.print("Backward Position: ");
      Serial.println(backward_position);
    }
    break;
   case HomingPhase::kDone:
    break;
  }

  if (homing_phase == HomingPhase::kDone) {
    g_stepper_controller.SetTargetSpeed(0.0f);

    int32_t target = range * 0.5f;
    if (g_stepper_controller.GetCurrentPosition() < (target + range * 0.01f) && g_stepper_controller.GetCurrentPosition() > (target - range * 0.01f)) {
      g_stepper_controller.SetTargetSpeed(0.0f);
    } else {
      g_stepper_controller.UpdateTargetSpeedByPosition(target, kMaxSpeed);
    }
  }

  /*
  if (homing_phase == HomingPhase::kDone) {
    // Once the homing is done, the valid range is (0, range).
  
    static int32_t low_position_target = 3.5f * range / 8;
    static int32_t high_position_target = 4.5f * range / 8;

    static int32_t low_position_threshold = 3.6f * range / 8;
    static int32_t high_position_threshold = 4.4f * range / 8;

    static int32_t current_target = low_position_target;

    int32_t current_position = g_stepper_controller.GetCurrentPosition();

    if (current_position > high_position_threshold) {
      current_target = low_position_target;
    } else if (current_position < low_position_threshold) {
      current_target = high_position_target;
    }

     g_stepper_controller.UpdateTargetSpeedByPosition(current_target, kMaxSpeed);
  }
  */

  // Run the control loop at approx 1000 Hz.
  // This is only for velocity updates. Stepping happens asynchronously.
  // We put the delay inserter here because this is the most crucial part
  // for loop timing.
  static DelayInserter<1000> delay_inserter;
  delay_inserter.Sync();
  g_stepper_controller.Update();
}
