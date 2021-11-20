#ifndef CONFIG_H
#define CONFIG_H

// Stepper config -------------------------------------------------------------
// Motor current limit in Amps(rms). Maximum supported by this board is 4.6A.
constexpr float kMotorCurrentLimit = 1.8f;

constexpr int kFullStepsPerRev = 200;

// Maximum acceleration in RPM/s.
constexpr float kMaxAcceleration = 6000.0f;

constexpr float kHomingSpeed = 100.0f;

// StallGuard 2 sensitivity setting. See 5. StallGuard2 Load Measurement in
// the datasheet. Higher values = less sensitive. (63 to -64).
constexpr int8_t kStallGuardThreshold = 2;

// StallGuard doesn't work at very low velocities due to weak back-emf.
constexpr float kStallGuardMinSpeed = 60.0f;

// ----------------------------------------------------------------------------

#endif  // CONFIG_H
