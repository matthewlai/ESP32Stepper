#ifndef CONFIG_H
#define CONFIG_H

// Board config ---------------------------------------------------------------

// First revision boards have TMC2590
#define TMC2590

// Second revision boards have TMC2160(A)
//#define TMC2160

// ----------------------------------------------------------------------------

// Stepper config -------------------------------------------------------------
// Motor current limit in Amps(rms). Maximum supported by this board is 4.6A.
constexpr float kMotorCurrentLimit = 1.6f;

// Flip all directions.
constexpr bool kFlipDrivingDirection = true;

constexpr int kFullStepsPerRev = 200;

// Maximum speed in Revs/s.
constexpr float kMaxSpeed = 800;

// Maximum acceleration in Revs/s^2.
constexpr float kMaxAcceleration = 500.0f;

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

// Motion planning config -----------------------------------------------------

// Position controller will consider target reached when we get this close to
// target position as a fraction of total range.
constexpr float kPositionTolerance = 0.01f;

// Velocities below this are ignored by the driver (assumed to be rounding
// errors).
constexpr float kMinVelocity = 0.0001f;

// ----------------------------------------------------------------------------

// Program Config -------------------------------------------------------------

// Output all stepper / trajectory parameters at each control step for plotting.
// (Can cause step skipping due to the volume of data printed).
constexpr bool kStepperDebugPlotting = false;

// ----------------------------------------------------------------------------

#endif  // CONFIG_H
