// Libraries:
// Adafruit Neopixel 1.10.0x

#include <Adafruit_NeoPixel.h>

#include <SPI.h>

#include "config.h"
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
  kAccelerating,
  kDrivingForward,
  kAcceleratingBackward,
  kDrivingBackward,
  kDone
};

void loop() {
  constexpr float kPeriodMs = 1000.0f;
  constexpr float kBrightness = 0.1f;
  constexpr float kCoeff = (2 * M_PI) / kPeriodMs;
  auto t = millis();
  float r = sin(kCoeff * t) * 0.5f + 0.5f;
  float g = sin(kCoeff * t + (0.33f * 2.0f * M_PI)) * 0.5f + 0.5f;
  float b = sin(kCoeff * t + (0.66f * 2.0f * M_PI)) * 0.5f + 0.5f;
  SetRGBLedColour(kBrightness * r, kBrightness * g, kBrightness * b);

  static HomingPhase homing_phase = HomingPhase::kStart;

  static int32_t forward_position = 0;
  static int32_t backward_position = 0;
  static int32_t range = 0;

  static int32_t stall_detection_delay = 10;

  switch (homing_phase) {
   case HomingPhase::kStart:
    // Disable stallguard filtering for faster response.
    g_stepper_controller.SetStallGuardFiltering(false);
    homing_phase = HomingPhase::kAccelerating;
    break;
   case HomingPhase::kAccelerating:
    g_stepper_controller.SetTargetSpeedRPM(kHomingSpeed);
    if (g_stepper_controller.GetCurrentSpeed() > (kHomingSpeed * 0.9f)) {
      --stall_detection_delay;
      if (stall_detection_delay <= 0) {
        homing_phase = HomingPhase::kDrivingForward;
        stall_detection_delay = 10;
      }
    }
    break;
   case HomingPhase::kDrivingForward:
    if (g_stepper_controller.IsStalled()) {
      forward_position = g_stepper_controller.GetCurrentPosition();
      g_stepper_controller.SetTargetSpeedRPM(-kHomingSpeed);
      homing_phase = HomingPhase::kAcceleratingBackward;
    }
    break;
   case HomingPhase::kAcceleratingBackward:
    if (g_stepper_controller.GetCurrentSpeed() < -(kHomingSpeed * 0.9f)) {
      --stall_detection_delay;
      if (stall_detection_delay <= 0) {
        homing_phase = HomingPhase::kDrivingBackward;
        stall_detection_delay = 10;
      }
    }
    break;
   case HomingPhase::kDrivingBackward:
    if (g_stepper_controller.IsStalled()) {
      backward_position = g_stepper_controller.GetCurrentPosition();
      range = forward_position - backward_position;
      g_stepper_controller.SetTargetSpeedRPM(0.0f);
      g_stepper_controller.SetCurrentPosition(0);
      g_stepper_controller.SetStallGuardFiltering(true);
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
    // Once the homing is done, the valid range is (0, range).
  
    static int32_t low_position_target = 3 * range / 8;
    static int32_t high_position_target = 5 * range / 8;

    int32_t current_position = g_stepper_controller.GetCurrentPosition();

    static float motor_speed = 0.0f;

    if (current_position < low_position_target) {
      motor_speed = 600.0f;
    } else if (current_position > high_position_target) {
      motor_speed = -600.0f;
    }
  
    g_stepper_controller.SetTargetSpeedRPM(motor_speed);
  }

  Serial.println(g_stepper_controller.ReadStallGuardValue());

  // Run the control loop at approx 100 Hz.
  // This is only for velocity updates. Stepping happens asynchronously.
  delay(10);
}
