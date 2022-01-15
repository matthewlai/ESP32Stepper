#ifndef TMC2590_H
#define TMC2590_H

#ifdef TMC2590

#include "tmc_driver.h"

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

constexpr float kRevsPerMicroStep = 1.0f / (kFullStepsPerRev * kMicrostepsPerFullStep);
// ----------------------------------------------------------------

// TODO: Implement latching stall detection.
void IRAM_ATTR DriverStepHandler(void) {}

class TMC2590Driver : public TMCDriver {
 public:
  TMC2590Driver();

  void Begin() override;
  void Stop() override;

  uint32_t ReadStallGuardValue() override;
  void SetStallGuardFiltering(bool filter_on) override;
  void SetStallGuardThreshold(int8_t threshold) override;

  void SetMotorCurrent(float new_current_setting) override;

  int StepsPerRev() override { return kMicrostepsPerFullStep * kFullStepsPerRev; }

 private:
  uint32_t DoTransaction(uint32_t x);

  // TMC2590 is on the VSPI bus default pins.
  SPIClass* tmc_spi_;

  // Configuration registers (including the address bits)
  uint32_t chopconf_;
  uint32_t sgcsconf_;
  uint32_t drvconf_;
  uint32_t drvctrl_;
};

TMC2590Driver::TMC2590Driver()
  : tmc_spi_(nullptr) {}

void TMC2590Driver::Begin() {
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

void TMC2590Driver::Stop() {
  // Disable all motors.
  digitalWrite(kTmcEn, HIGH);
}

uint32_t TMC2590Driver::ReadStallGuardValue() {
  // We have to send any register to get the value. It doesn't matter
  // which one.
  uint32_t ret = DoTransaction(drvctrl_);
  // With RDSEL=%01, bit 19-10 are the stallguard value.
  ret >>= 10;
  ret &= 0x3ff;
  return ret;
}

void TMC2590Driver::SetStallGuardFiltering(bool filter_on) {
  sgcsconf_ &= ~(1 << 16);
  if (filter_on) {
    sgcsconf_ |= 1 << 16;
  }
  DoTransaction(sgcsconf_);
}

void TMC2590Driver::SetStallGuardThreshold(int8_t threshold) {
  // Update StallGuard threshold since it seems to depend on the direction.
  sgcsconf_ &= 0xffffff00ff;

  // Convert 8-bit two's complement to 7-bit by dropping the high bit
  uint8_t sgt_uint;
  std::memcpy(&sgt_uint, &threshold, 1);
  sgcsconf_ |= static_cast<uint32_t>(sgt_uint & 0x7f) << 8;
  DoTransaction(sgcsconf_);
}

void TMC2590Driver::SetMotorCurrent(float new_current_setting) {
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
uint32_t TMC2590Driver::DoTransaction(uint32_t x) {
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

#endif  // TMC2590
#endif  // TMC2590_H
