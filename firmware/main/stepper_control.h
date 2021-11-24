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
constexpr float kMotorCurrentLimitScale =
    constrain(kMotorCurrentLimit / kMotorCurrentLimitMax, 0.0f, 1.0f);

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

  void SetTargetSpeedRPM(float speed_rpm);

  void Update();

  bool IsStallGuardValid() { return fabs(current_speed_) > kStallGuardMinSpeed; }
  bool IsStalled() { return ReadStallGuardValue() == 0; }
  uint32_t ReadStallGuardValue();
  void SetStallGuardFiltering(bool filter_on);

  int32_t GetCurrentPosition();
  void SetCurrentPosition(int32_t new_position);
  float GetCurrentSpeed() { return current_speed_; }

 private:
  uint32_t DoTransaction(uint32_t x);

  // TMC2590 is on the VSPI bus default pins.
  SPIClass* tmc_spi_;

  // Configuration registers (including the address bits)
  uint32_t chopconf_;
  uint32_t sgcsconf_;
  uint32_t drvconf_;
  uint32_t drvctrl_;

  // Speed we are ramping to.
  float target_speed_rpm_;

  // Speed we are currently commanding.
  float current_speed_;

  // Last time the control loop was run.
  uint32_t last_update_time_us_;

  hw_timer_t* step_timer_;
};

TMC2590Controller::TMC2590Controller()
  : tmc_spi_(nullptr), target_speed_rpm_(0.0f), last_update_time_us_(0),
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

  // If our current limit scale is less than 0.5, use VSENSE=1 for better resolution.
  bool use_vsense_1 = kMotorCurrentLimitScale <= 0.5f;
  uint32_t cs = (use_vsense_1 ? (kMotorCurrentLimitScale * 2.0f) : kMotorCurrentLimitScale) * 0x1f;

  sgcsconf_ |= cs;
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

  if (use_vsense_1) {
    drvconf_ |= 0x00040;
  }
  
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

  // Enable motor drivers.
  digitalWrite(kTmcEn, LOW);
}

void TMC2590Controller::SetTargetSpeedRPM(float speed_rpm) {
  target_speed_rpm_ = speed_rpm;
  Update();
}

void TMC2590Controller::Update() {
  uint32_t time_now = micros();
  uint32_t time_since_last_run = time_now - last_update_time_us_;
  last_update_time_us_ = time_now;

  float max_acceleration = kMaxAcceleration;

  float max_speed_delta = time_since_last_run / 1000000.0f * max_acceleration;
  float speed_to_ramp = target_speed_rpm_ - current_speed_;

  float speed_delta = constrain(speed_to_ramp, -max_speed_delta, max_speed_delta);

  // Don't bother if we are basically at the target speed already (otherwise we
  // will be changing the timer period unnecessarily all the time).
  if (fabs(speed_delta) < kSpeedEpsilon) {
    return;
  }

  float new_speed = current_speed_ + speed_delta;

  current_speed_ = new_speed;

  uint32_t delay_us = 1000000.0f / (fabs(current_speed_) / 60.0f * kFullStepsPerRev * kMicrostepsPerFullStep);

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
    digitalWrite(kTmcDir, HIGH);
  } else {
    g_current_dir_sign = -1;
    digitalWrite(kTmcDir, LOW);
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
