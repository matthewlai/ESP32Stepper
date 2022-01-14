#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include <cstring>

#if defined(TMC2590)
#include "tmc2590.h"
#elif defined(TMC2160)
#include "tmc2160.h"
#else
#error "No driver selected"
#endif

// Stepper ISR ----------------------------------------------------
// This is the ISR for the timer used for stepping. Note that no
// floating point operations are allowed in ISRs, so they must be
// converted to fixed point by the controller.

// Current position kept track of by the ISR. Output to main thread.
volatile int32_t g_current_position = 0;

// Current direction (1 or -1). Input from main thread.
// This is only used to update g_current_position.
volatile int32_t g_current_dir_sign = 1;

// Whether the next edge should be rising or falling. We use double
// edge stepping (DEDGE=1 in DRVCTRL) to cut down our ISR
// requirements by half. Only used internally in the ISR.
volatile bool g_current_edge = false;

// Semaphore for atomic updates.
portMUX_TYPE g_stepper_timer_mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR StepperTimerHandler(void) {
  portENTER_CRITICAL_ISR(&g_stepper_timer_mux);
  digitalWrite(kTmcStep, g_current_edge);
  g_current_edge = !g_current_edge;

  g_current_position += g_current_dir_sign;

  DriverStepHandler();
  portEXIT_CRITICAL_ISR(&g_stepper_timer_mux);
}
// ----------------------------------------------------------------

class MotionController {
 public:
  MotionController();

  void Begin();
  void Stop();

  void SetTargetSpeed(float speed_rps);
  void SetTargetSpeedRPM(float speed_rpm) { SetTargetSpeed(speed_rpm / 60.0f); }

  // Use built-in planner to update target speed to get to target
  // position without overshooting.
  // Note that this class is fundamentally a velocity-based controller,
  // so this convenience function needs to be called before every update to update target 
  // velocity. This is NOT done automatically as part of Update();
  void UpdateTargetSpeedByPosition(int32_t target_position, float max_speed);

  void Update();

  int32_t GetCurrentPosition();
  void SetCurrentPosition(int32_t new_position);
  float GetCurrentSpeed() { return current_speed_; }
  float GetCurrentSpeedRPM() { return current_speed_ * 60.0f; }

  void SetAccelerationLimit(float acceleration_limit) { acceleration_limit_ = acceleration_limit; }
  void SetJerkLimit(float jerk_limit) { jerk_limit_ = jerk_limit; }

  TMCDriver* Driver() { return driver_; }

 private:
  TMCDriver* driver_;
 
  // Speed we are ramping to (Rev/s).
  float target_speed_;

  // Speed we are currently commanding.
  float current_speed_;

  float current_acceleration_;

  float acceleration_limit_;
  float jerk_limit_;

  // Last time the control loop was run.
  uint32_t last_update_time_us_;

  hw_timer_t* step_timer_;
};

MotionController::MotionController()
  : target_speed_(0.0f),
    current_speed_(0.0f),
    current_acceleration_(0.0f),
    acceleration_limit_(kMaxAcceleration),
    jerk_limit_(kMaxJerk),
    last_update_time_us_(0),
    step_timer_(nullptr) {
#if defined(TMC2590)
  driver_ = new TMC2590Driver;
#elif defined(TMC2160)
  driver_ = new TMC2160Driver;
#else
#error "No driver selected"
#endif
}

void MotionController::Begin() {
  driver_->Begin();  
  step_timer_ = timerBegin(3, 80 /* APB_CLK / 80 = 1MHz*/, /*countUp=*/true);
  timerAttachInterrupt(step_timer_, &StepperTimerHandler, 1);
}

void MotionController::Stop() {
  if (step_timer_) {
    timerAlarmDisable(step_timer_);
  }
  driver_->Stop();
}

void MotionController::SetTargetSpeed(float speed_rps) {
  target_speed_ = speed_rps;
}

void MotionController::UpdateTargetSpeedByPosition(
    int32_t target_position, float max_speed) {
  int32_t current_position = GetCurrentPosition();
  
  float position_error = fabs((target_position - current_position) * kRevsPerMicroStep);

  // Given the current position error and our max acceleration,
  // how fast can we be at this point and not overshoot?
  // Note that this calculation ignores jerk limit, so it's not
  // 100% accurate. This also doesn't take into account the current
  // speed (it assumes we are accelerating to the target velocity
  // at infinite acceleration), so it's not optimally efficient.

  // Displacement under constant acceleration is:
  // d = v0 * t + (1/2) * a * t^2
  //
  // d = position_error
  // a = -acceleration_limit_ (assuming v0 > 0, d > 0)
  // t = v0 / -a
  // d = v0 * v0 / -a + (1/2) * a * (v0^2 / a^2)
  //   = -(v0^2 / a) + (1/2) * v0^2 / a
  //   = -(1/2) v0^2 / a
  //   = (1/2) v0^2 / acceleration_limit_
  //
  // |v0| = sqrt(2 * d * acceleration_limit_)

  // TODO: Figure out how we have to drop the 2x to stop overshooting.
  float v_abs = sqrt(position_error * acceleration_limit_);
  float v = 0.0f;

  if (current_position < target_position) {
    v = min(max_speed, v_abs);
  } else {
    v = max(-max_speed, -v_abs);
  }

  SetTargetSpeed(v);
}

void MotionController::Update() {
  uint32_t time_now = micros();

  // Skip first call so we don't get a huge step on first update.
  if (last_update_time_us_ == 0) {
    last_update_time_us_ = time_now;
    return;
  }
  
  float time_delta = (time_now - last_update_time_us_) / 1000000.0f;
  float reciprocal_time_delta = 1.0f / time_delta;
  last_update_time_us_ = time_now;

  // Update acceleration.
  float velocity_error = target_speed_ - current_speed_;

  // First start with target acceleration that would take us to target velocity in one step.
  float acceleration_target = velocity_error * reciprocal_time_delta;

  // Constrain the acceleration to avoid overshoot due to jerk limit.
  // TODO: Figure out how we have to drop the 2x to stop overshooting.
  float a_max = sqrt(jerk_limit_ * fabs(target_speed_ - current_speed_));

  if (target_speed_ > current_speed_) {
    if (acceleration_target > a_max) {
      acceleration_target = a_max;
    }
  } else {
    if (acceleration_target < -a_max) {
      acceleration_target = -a_max;
    }
  }

  float acceleration_error = acceleration_target - current_acceleration_;

  // Calculate actual acceleration update subject to jerk limit.
  float acceleration_update = constrain(acceleration_error, -jerk_limit_ * time_delta, jerk_limit_ * time_delta);
  current_acceleration_ = constrain(current_acceleration_ + acceleration_update, -acceleration_limit_, acceleration_limit_);

  // Update speed.
  float new_speed = current_speed_ + current_acceleration_ * time_delta;

  if (kStepperDebugPlotting) {
    Serial.print("A:");
    Serial.print(current_acceleration_);
    Serial.print("\t");
    Serial.print("V:");
    Serial.print(new_speed * 60.0f);
    Serial.print("\t");
    Serial.print("Vt:");
    Serial.print(target_speed_ * 60.0f);
    Serial.print("\t");
    Serial.print("P:");
    Serial.print(GetCurrentPosition() / 3.0f);
    Serial.print("\t");
    Serial.print("SG:");
    Serial.println(driver_->ReadStallGuardValue());
  }

  current_speed_ = new_speed;

  uint32_t delay_us = 1000000.0f / (max(fabs(current_speed_), 0.000001f) * kFullStepsPerRev * kMicrostepsPerFullStep);

  driver_->SetStallGuardThreshold(kStallGuardThreshold[(current_speed_ > 0 ? 0 : 1)]);

  timerAlarmWrite(step_timer_, delay_us, /*autoreload=*/true);
  
  portENTER_CRITICAL_ISR(&g_stepper_timer_mux);
  if (current_speed_ > 0) {
    g_current_dir_sign = 1;
    digitalWrite(kTmcDir, kFlipDrivingDirection ? LOW : HIGH);
  } else {
    g_current_dir_sign = -1;
    digitalWrite(kTmcDir, kFlipDrivingDirection ? HIGH : LOW);
  }
  portEXIT_CRITICAL_ISR(&g_stepper_timer_mux);

  if (fabs(current_speed_) > kMinVelocity) {
    timerAlarmEnable(step_timer_);
  } else {
    timerAlarmDisable(step_timer_);
  }
}

int32_t MotionController::GetCurrentPosition() {
  int32_t ret;
  portENTER_CRITICAL_ISR(&g_stepper_timer_mux);
  ret = g_current_position;
  portEXIT_CRITICAL_ISR(&g_stepper_timer_mux);
  return ret;
}

void MotionController::SetCurrentPosition(int32_t new_position) {
  portENTER_CRITICAL_ISR(&g_stepper_timer_mux);
  g_current_position = new_position;
  portEXIT_CRITICAL_ISR(&g_stepper_timer_mux);
}

#endif  // STEPPER_CONTROL_H
