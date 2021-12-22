#ifndef CONFIG_H
#define CONFIG_H

// Stepper config -------------------------------------------------------------
// Motor current limit in Amps(rms). Maximum supported by this board is 4.6A.
constexpr float kMotorCurrentLimit = 1.6f;

// Flip all directions.
constexpr bool kFlipDrivingDirection = true;

constexpr int kFullStepsPerRev = 200;

// Maximum speed in Revs/s.
constexpr float kMaxSpeed = 800;

// Maximum acceleration in Revs/s^2.
constexpr float kMaxAcceleration = 700.0f;

// Maximum jerk in Rev/s^3
constexpr float kMaxJerk = 100000.0f;

constexpr float kHomingSpeedRPM = 100.0f;
constexpr float kHomingCurrent = 0.6f;
constexpr float kHomingAcceleration = 50.0f;

// Effectively disable max jerk limit during homing, so we don't try
// driving hard into the end.
constexpr float kHomingMaxJerk = 1000000.0f;

// StallGuard 2 sensitivity setting. See 5. StallGuard2 Load Measurement in
// the datasheet. Higher values = less sensitive. (63 to -64).
// Value per direcion [forward, backward].
constexpr int8_t kStallGuardThreshold[2] = { 4, 0 };

// StallGuard doesn't work at very low velocities due to weak back-emf.
constexpr float kStallGuardMinSpeed = 60.0f;

// ----------------------------------------------------------------------------

#endif  // CONFIG_H
