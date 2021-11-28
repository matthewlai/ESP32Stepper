#ifndef CONFIG_H
#define CONFIG_H

// Stepper config -------------------------------------------------------------
// Motor current limit in Amps(rms). Maximum supported by this board is 4.6A.
constexpr float kMotorCurrentLimit = 1.6f;

constexpr int kFullStepsPerRev = 200;

// Maximum acceleration in RPM/s.
constexpr float kMaxAcceleration = 6000.0f;

// Maximum jerk in RPM/s^2
constexpr float kMaxJerk = 80000.0f;

constexpr float kHomingSpeed = 100.0f;

// StallGuard 2 sensitivity setting. See 5. StallGuard2 Load Measurement in
// the datasheet. Higher values = less sensitive. (63 to -64).
// Value per direcion.
constexpr int8_t kStallGuardThreshold[2] = { 1, 6 };

// StallGuard doesn't work at very low velocities due to weak back-emf.
constexpr float kStallGuardMinSpeed = 60.0f;

// ----------------------------------------------------------------------------

#endif  // CONFIG_H
