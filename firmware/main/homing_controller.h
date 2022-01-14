#ifndef HOMING_CONTROLLER_H
#define HOMING_CONTROLLER_H

#include "config.h"
#include "stepper_control.h"
#include "tmc_driver.h"

class HomingController {
 public:
  HomingController(MotionController* motion_controller);

  void Update();
  bool Done() const { return phase_ == HomingPhase::kDone; }

  // The motion controller is calibrated to [0, Range()] once homing is done.
  // Only valid once Done() returns true.
  float Range() const { return range_; }

 private:
  enum class HomingPhase {
    kStart,
    kForward,
    kBackward,
    kDone
  };
 
  MotionController* motion_controller_;
  HomingPhase phase_;

  int32_t forward_position_;
  int32_t backward_position_;
  int32_t range_;
};

HomingController::HomingController(MotionController* motion_controller)
  : motion_controller_(motion_controller), phase_(HomingPhase::kStart) {}

void HomingController::Update() {
  switch (phase_) {
   case HomingPhase::kStart:
    // Disable stallguard filtering for faster response.
    motion_controller_->Driver()->SetStallGuardFiltering(false);
    motion_controller_->SetAccelerationLimit(kHomingAcceleration);
    motion_controller_->SetJerkLimit(kHomingMaxJerk);
    motion_controller_->Driver()->SetMotorCurrent(kHomingCurrent);
    phase_ = HomingPhase::kForward;
    motion_controller_->Driver()->ClearStallFlag();
    break;
   case HomingPhase::kForward:
    motion_controller_->SetTargetSpeedRPM(kHomingSpeedRPM);
    if ((motion_controller_->GetCurrentSpeedRPM() > (kHomingSpeedRPM * 0.99f)) &&
        motion_controller_->Driver()->IsStalled()) {
      forward_position_ = motion_controller_->GetCurrentPosition();
      motion_controller_->SetTargetSpeedRPM(-kHomingSpeedRPM);
      phase_ = HomingPhase::kBackward;
      Serial.println("Forward done");
    }
    break;
   case HomingPhase::kBackward:
   motion_controller_->SetTargetSpeedRPM(-kHomingSpeedRPM);
    if ((motion_controller_->GetCurrentSpeedRPM() < -(kHomingSpeedRPM * 0.99f)) &&
        motion_controller_->Driver()->IsStalled()) {
      backward_position_ = motion_controller_->GetCurrentPosition();
      range_ = forward_position_ - backward_position_;
      motion_controller_->SetTargetSpeedRPM(0.0f);
      motion_controller_->SetCurrentPosition(0);
      motion_controller_->Driver()->SetStallGuardFiltering(true);
      motion_controller_->SetAccelerationLimit(kMaxAcceleration);
      motion_controller_->SetJerkLimit(kMaxJerk);
      motion_controller_->Driver()->SetMotorCurrent(kMotorCurrentLimit);
      phase_ = HomingPhase::kDone;
      Serial.println("Homing Done!");
      Serial.print("Forward Position: ");
      Serial.println(forward_position_);
      Serial.print("Backward Position: ");
      Serial.println(backward_position_);
    }
    motion_controller_->Driver()->ClearStallFlag();
    break;
   case HomingPhase::kDone:
    break;
  }
}

#endif  // HOMING_CONTROLLER_H
