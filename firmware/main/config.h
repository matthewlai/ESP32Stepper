#ifndef CONFIG_H
#define CONFIG_H

// Board selection ------------------------------------------------------------

// Uncomment one of the following:

// First revision boards with TMC2590
//#define HW_REV 1

// Second revision boards with TMC2160(A)
//#define HW_REV 2

// Third revision boards with TMC2160(A) and ESP32-PICO-D4
#define HW_REV 3

// ----------------------------------------------------------------------------

// Stepper config -------------------------------------------------------------
// Motor current limit in Amps(rms). Maximum supported by this board is 4.6A.
// The driver will never exceed this limit.
constexpr float kMotorCurrentLimit = 1.6f;

// Flip all directions.
constexpr bool kFlipDrivingDirection = false;

constexpr int kFullStepsPerRev = 200;

// Maximum speed in Revs/s.
constexpr float kMaxSpeed = 13.3f; // 800 RPM

// Maximum acceleration in Revs/s^2.
constexpr float kMaxAcceleration = 500.0f;

// Maximum jerk in Rev/s^3
constexpr float kMaxJerk = 10000.0f;

constexpr float kHomingSpeedRPM = 100.0f;
constexpr float kHomingCurrent = 0.6f;
constexpr float kHomingAcceleration = 50.0f;

// Effectively disable max jerk limit during homing, so we don't try
// driving hard into the end.
constexpr float kHomingMaxJerk = 1000000.0f;

// StallGuard 2 sensitivity setting. See 5. StallGuard2 Load Measurement in
// the datasheet. Higher values = less sensitive. (63 to -64).
// Value per direcion [forward, backward].
constexpr int8_t kStallGuardThreshold[2] = { 3, 0 };

// StallGuard doesn't work at very low velocities due to weak back-emf.
// This is the speed in Rev/s below which StallGuard should not be trusted.
// On TMC2160 this is done automatically on the driver side with TCOOLTHRS.
// 1 RPS should work for most motors, but may need to be higher for motors
// smaller than NEMA17.
constexpr float kStallGuardMinSpeed = 1.0f;

// ----------------------------------------------------------------------------

// Motion planning config -----------------------------------------------------
constexpr bool kUseStallGuardHoming = true;

// Position controller will consider target reached when we get this close to
// target position as a fraction of total range.
constexpr float kPositionTolerance = 0.01f;

// Velocities below this (Rev/s) are ignored by the driver (assumed to be
// rounding errors).
constexpr float kMinVelocity = 0.0001f;

// ----------------------------------------------------------------------------

// Program config -------------------------------------------------------------

// Output all stepper / trajectory parameters at each control step for plotting.
// (Can cause step skipping due to the volume of data printed).
constexpr bool kStepperDebugPlotting = false;

// ----------------------------------------------------------------------------

// Board definitions ----------------------------------------------------------
#if HW_REV == 1
#define TMC2590
#else
#define TMC2160
#endif

#if HW_REV == 1

constexpr int kLedData = 21;

constexpr int kTmcEn = 4;
constexpr int kTmcStep = 25;
constexpr int kTmcDir = 26;
constexpr int kTmcSgTst = 27;

#elif HW_REV == 2

constexpr int kLedData = 21;

// Second revision has pair switch (currently used for stopping the driver).
#define HAS_PAIR_SWITCH
constexpr int kPairSw = 22;

#define HAS_VSENSE
constexpr int kVSenseADCPin = 36;
constexpr float kVSenseScale = (10000.0f + 470.0f) / 470.0f;
constexpr float kVsOvervoltageThreshold = 38.0f;

constexpr int kTmcEn = 4;
constexpr int kTmcStep = 25;
constexpr int kTmcDir = 26;

constexpr int kTmcDiag0 = 15;
constexpr int kTmcDiag1 = 13;

constexpr int kTmcDco = 16;
constexpr int kTmcDcEn = 17;

#elif HW_REV == 3

constexpr int kLedData = 21;

// Second revision has pair switch (currently used for stopping the driver).
#define HAS_PAIR_SWITCH
constexpr int kPairSw = 22;

#define HAS_VSENSE
constexpr int kVSenseADCPin = 36;
constexpr float kVSenseScale = (10000.0f + 150.0f) / 150.0f;
constexpr float kVsOvervoltageThreshold = 38.0f;

constexpr int kTmcEn = 15;
constexpr int kTmcStep = 25;
constexpr int kTmcDir = 26;

constexpr int kTmcDiag0 = 4;
constexpr int kTmcDiag1 = 2;

constexpr int kTmcDco = 9;
constexpr int kTmcDcEn = 10;

#endif

// ----------------------------------------------------------------------------

#endif  // CONFIG_H
