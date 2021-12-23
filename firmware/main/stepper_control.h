#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include <cstring>

// Pin definitions ------------------------------------------------
// Drive low to enable motor drivers
constexpr int kTmcEn = 4;

// Stepping and direction outputs
constexpr int kTmcStep = 25;
constexpr int kTmcDir = 26;

// StallGuard input, high indicates stall at "sufficient velocity"
constexpr int kTmcSgTst = 27;
// ----------------------------------------------------------------

// Other Constants ------------------------------------------------
// 4 MHz maximum speed with the TMC2590 on internal oscillator
// MSB first, and SPI mode 3 (7.2 Bus Timing)
const SPISettings kTmcSpiSettings(4 * 1000 * 1000, MSBFIRST,
                                  SPI_MODE3);

// Motor current limit. 1.0 = 4.6A.
constexpr float kMotorCurrentLimitMax = 4.6f;

// 0x0 -> 256
// 0x1 -> 128
// 0x2 -> 64
// 0x3 -> 32
// 0x4 -> 16
// 0x5 -> 8
// 0x6 -> 4
// 0x7 -> 2
// 0x8 -> 1 (full step)

// Since we need to service an interrupt every step for pulse
// counting, we have to choose a stepping setting that doesn't
// result in excessive interrupts. 10kHz seems like a good number
// to aim for. MicroPlyer will make sure we have smooth motion
// anyways.
// Maximum speed = 1000 RPM = 16 RPS
// 16 RPS * 200 steps/rev = 3200 steps/s
// Using 4 microsteps, we have ~12.8kHz interrupt rate. 

constexpr byte kMicrostepsSetting = 0x6;
constexpr byte kMicrostepsPerFullStep = 4;

// Speed in RPM below which we consider the motor stopped.
constexpr byte kSpeedEpsilon = 0.01f;

constexpr float kRevsPerMicroStep = 1.0f / (kFullStepsPerRev * kMicrostepsPerFullStep);
// ----------------------------------------------------------------

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
  digitalWrite(kTmcStep, g_current_edge);
  g_current_edge = !g_current_edge;

  portENTER_CRITICAL_ISR(&g_stepper_timer_mux);
  g_current_position += g_current_dir_sign;
  portEXIT_CRITICAL_ISR(&g_stepper_timer_mux);
}
// ----------------------------------------------------------------

class TMC2590Controller {
 public:
  TMC2590Controller();

  void Begin();

  void SetTargetSpeed(float speed_rps);
  void SetTargetSpeedRPM(float speed_rpm) { SetTargetSpeed(speed_rpm / 60.0f); }

  // Use built-in planner to update target speed to get to target
  // position without overshooting.
  // Note that this class is fundamentally a velocity-based controller,
  // so this convenience function needs to be called before every update to update target 
  // velocity. This is NOT done automatically as part of Update();
  void UpdateTargetSpeedByPosition(int32_t target_position, float max_speed_rpm);

  void Update();

  bool IsStallGuardValid() { return fabs(current_speed_) > kStallGuardMinSpeed; }
  bool IsStalled() { return ReadStallGuardValue() == 0; }
  uint32_t ReadStallGuardValue();
  void SetStallGuardFiltering(bool filter_on);

  int32_t GetCurrentPosition();
  void SetCurrentPosition(int32_t new_position);
  float GetCurrentSpeed() { return current_speed_; }
  float GetCurrentSpeedRPM() { return current_speed_ * 60.0f; }

  void SetMotorCurrent(float new_current_setting);
  void SetAccelerationLimit(float acceleration_limit) { acceleration_limit_ = acceleration_limit; }
  void SetJerkLimit(float jerk_limit) { jerk_limit_ = jerk_limit; }

 private:
  uint32_t DoTransaction(uint32_t x);

  // TMC2590 is on the VSPI bus default pins.
  SPIClass* tmc_spi_;

  // Configuration registers (including the address bits)
  uint32_t chopconf_;
  uint32_t sgcsconf_;
  uint32_t drvconf_;
  uint32_t drvctrl_;

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

TMC2590Controller::TMC2590Controller()
  : tmc_spi_(nullptr),
    target_speed_(0.0f),
    current_speed_(0.0f),
    current_acceleration_(0.0f),
    acceleration_limit_(kMaxAcceleration),
    jerk_limit_(kMaxJerk),
    last_update_time_us_(0),
    step_timer_(nullptr) {}

void TMC2590Controller::Begin() {
  // Disable all motors.
  pinMode(kTmcEn, OUTPUT);
  digitalWrite(kTmcEn, HIGH);

  pinMode(kTmcStep, OUTPUT);
  pinMode(kTmcDir, OUTPUT);
  pinMode(kTmcSgTst, INPUT);
  
  tmc_spi_ = new SPIClass(VSPI);
  tmc_spi_->begin();
  pinMode(tmc_spi_->pinSS(), OUTPUT);
  
  // This is based on the example initialisation sequence given in
  // 7.11 Device Initialization

  // 1001 0000 0001 1011 0100
  // CHOPCONF
  // TBL=10 (36 clocks)
  // Chopper mode = 0 (SpreadCycle)
  // Random TOFF time off
  // Hysteresis decrement period 00 (16 clocks)
  // Hysteresis end (low) value 0011 (0)
  // Hysteresis start value 011 (4)
  // Off time 0100 (Nclk = 24 + (32 * 4) = 152 clocks)
  chopconf_ = 0x901b4;
  DoTransaction(chopconf_);

  // 1101 0000 0000 0000 0000
  // SGCSCONF
  // SFILT = 1 (filtered mode)
  // SGT = 0 (StallGuard 2 threshold = 0)
  sgcsconf_ = 0xd0000;

  DoTransaction(sgcsconf_);

  // 1110 1111 1000 0011 0011
  // DRVCONF
  // TST = 0
  // SLPH = 11
  // SLPL = 11
  // SLP2 = 1
  // DIS_S2G = 0
  // TS2G = 00
  // SDOFF = 0
  // RDSEL = 01 (StallGuard2 readback)
  // OTSENS = 0
  // SHRTSENS = 0
  // EN_PFD = 1
  // EN_S2VS = 1
  drvconf_ = 0xea813;
  
  DoTransaction(drvconf_);

  // 0000 0000 0001 0000 0000
  // DRVCTRL (SDOFF=0)
  // INTPOL = 0 (no step pulse interpolation)
  // DEDGE = 1 (step on both rising and falling edges)
  drvctrl_ = 0x00100;
  if (kMicrostepsSetting != 0) {
    // Enable MicroPlyer if we are not doing 256 steps microstepping.
    drvctrl_ |= 0x00200;
    drvctrl_ |= kMicrostepsSetting;
  }
  DoTransaction(drvctrl_);

  SetMotorCurrent(kMotorCurrentLimit);

  // Enable motor drivers.
  digitalWrite(kTmcEn, LOW);
}

void TMC2590Controller::SetTargetSpeed(float speed_rps) {
  target_speed_ = speed_rps;
}

void TMC2590Controller::UpdateTargetSpeedByPosition(
    int32_t target_position, float max_speed_rpm) {
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
    v = min(max_speed_rpm / 60.0f, v_abs);
  } else {
    v = max(-max_speed_rpm / 60.0f, -v_abs);
  }

  SetTargetSpeed(v);
}

void TMC2590Controller::Update() {
  uint32_t time_now = micros();
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
    Serial.println(ReadStallGuardValue());
  }

  current_speed_ = new_speed;

  uint32_t delay_us = 1000000.0f / (max(fabs(current_speed_), 0.000001f) * kFullStepsPerRev * kMicrostepsPerFullStep);

  // Update StallGuard threshold since it seems to depend on the direction.
  sgcsconf_ &= 0xffffff00ff;

  // Convert 8-bit two's complement to 7-bit by dropping the high bit
  uint8_t sgt_uint;
  std::memcpy(&sgt_uint, &kStallGuardThreshold[(current_speed_ > 0 ? 0 : 1)], 1);
  sgcsconf_ |= static_cast<uint32_t>(sgt_uint & 0x7f) << 8;
  DoTransaction(sgcsconf_);
  
  portENTER_CRITICAL_ISR(&g_stepper_timer_mux);
  if (current_speed_ > 0) {
    g_current_dir_sign = 1;
    digitalWrite(kTmcDir, kFlipDrivingDirection ? LOW : HIGH);
  } else {
    g_current_dir_sign = -1;
    digitalWrite(kTmcDir, kFlipDrivingDirection ? HIGH : LOW);
  }
  
  if (!step_timer_) {
    step_timer_ = timerBegin(3, 80 /* APB_CLK / 80 = 1MHz*/, /*countUp=*/true);
    timerAttachInterrupt(step_timer_, &StepperTimerHandler, 1);
  }

  timerAlarmWrite(step_timer_, delay_us, /*autoreload=*/true);

  if (!timerAlarmEnabled(step_timer_)) {
    timerAlarmEnable(step_timer_);
  }
  portEXIT_CRITICAL_ISR(&g_stepper_timer_mux);
}

uint32_t TMC2590Controller::ReadStallGuardValue() {
  // We have to send any register to get the value. It doesn't matter
  // which one.
  uint32_t ret = DoTransaction(drvctrl_);
  // With RDSEL=%01, bit 19-10 are the stallguard value.
  ret >>= 10;
  ret &= 0x3ff;
  return ret;
}

void TMC2590Controller::SetStallGuardFiltering(bool filter_on) {
  sgcsconf_ &= ~(1 << 16);
  if (filter_on) {
    sgcsconf_ |= 1 << 16;
  }
  DoTransaction(sgcsconf_);
}

int32_t TMC2590Controller::GetCurrentPosition() {
  int32_t ret;
  portENTER_CRITICAL_ISR(&g_stepper_timer_mux);
  ret = g_current_position;
  portEXIT_CRITICAL_ISR(&g_stepper_timer_mux);
  return ret;
}

void TMC2590Controller::SetCurrentPosition(int32_t new_position) {
  portENTER_CRITICAL_ISR(&g_stepper_timer_mux);
  g_current_position = new_position;
  portEXIT_CRITICAL_ISR(&g_stepper_timer_mux);
}

void TMC2590Controller::SetMotorCurrent(float new_current_setting) {
  float scaled_current_limit =
    constrain(new_current_setting / kMotorCurrentLimitMax, 0.0f, 1.0f);

  // If our scaled current limit is less than 0.5, use VSENSE=1 for better resolution.
  bool use_vsense_1 = scaled_current_limit <= 0.5f;
  uint32_t cs = (use_vsense_1 ? (scaled_current_limit * 2.0f) : scaled_current_limit) * 0x1f;

  sgcsconf_ &= ~0x1f;
  sgcsconf_ |= cs;
  DoTransaction(sgcsconf_);
  
  if (use_vsense_1) {
    drvconf_ |= 0x00040;
  } else {
    drvconf_ &= ~0x00040;
  }
  
  DoTransaction(drvconf_);
}

// TMC2590 requires 20-bit writes, and return 20-bit status. ESP32
// only supports multiples of 8-bits in transactions, so we use 24
// (4 MSBs ignored).
uint32_t TMC2590Controller::DoTransaction(uint32_t x) {
  uint32_t ret = 0;
  tmc_spi_->beginTransaction(kTmcSpiSettings);
  digitalWrite(tmc_spi_->pinSS(), LOW);
  uint32_t read_byte = tmc_spi_->transfer((x >> 16) & 0xFF);
  ret |= read_byte << 16;
  read_byte = tmc_spi_->transfer((x >> 8) & 0xFF);
  ret |= read_byte << 8;
  read_byte = tmc_spi_->transfer(x & 0xFF);
  ret |= read_byte;
  tmc_spi_->endTransaction();
  digitalWrite(tmc_spi_->pinSS(), HIGH);

  // We read 24 bits, but only the first (most significant)
  // 20 bits are valid, so drop the last 4 bits.
  return ret >> 4;
}

#endif  // STEPPER_CONTROL_H
