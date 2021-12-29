#ifndef TMC2160_H
#define TMC2160_H

#ifdef TMC2160

#include "float_utils.h"
#include "tmc_driver.h"

// Pin definitions ------------------------------------------------
// Drive low to enable motor drivers
constexpr int kTmcEn = 4;

// Stepping and direction outputs
constexpr int kTmcStep = 25;
constexpr int kTmcDir = 26;

constexpr int kTmcDiag0 = 15;
constexpr int kTmcDiag1 = 13;

constexpr int kTmcDco = 16;
constexpr int kTmcDcen = 17;
// ----------------------------------------------------------------

// Other Constants ------------------------------------------------
// 4 MHz maximum speed with the TMC2160 on internal oscillator
// MSB first, and SPI mode 3 (4.3 Timing)
const SPISettings kTmcSpiSettings(4 * 1000 * 1000, MSBFIRST,
                                  SPI_MODE3);

// Full scale sense voltage (325mV typ)
constexpr float kVFullScale = 0.325f;
constexpr float kRSense = 0.05f;

constexpr float kSqrt2 = 1.41421356237f;

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

// Registers and Fields -------------------------------------------
template <uint8_t kAddr, uint8_t kBitStart, uint8_t kBitLen> struct RegField {};

constexpr uint8_t kGCONF = 0x00;
constexpr uint8_t kGLOBALSCALER = 0x0b;
constexpr uint8_t kIHOLD_IRUN = 0x10;
constexpr uint8_t kTPOWERDOWN = 0x11;
constexpr uint8_t kTPWMTHRS = 0x13;
constexpr uint8_t kCHOPCONF = 0x6c;
// ----------------------------------------------------------------

class TMC2160Driver : public TMCDriver {
 public:
  TMC2160Driver();

  void Begin() override;

  uint32_t ReadStallGuardValue() override;
  void SetStallGuardFiltering(bool filter_on) override;
  void SetStallGuardThreshold(int8_t threshold) override;

  void SetMotorCurrent(float new_current_setting) override;

  int StepsPerRev() override { return kMicrostepsPerFullStep * kFullStepsPerRev; }

 private:
  // This sets the max motor current to kMotorCurrentLimit using GLOBALSCALER.
  // We prefer using GLOBALSCALER as much as possible (instead of CS) because
  // it's an analog scaling on the amplifier and preserves microstepping
  // resolution.
  // This should only be called on init, as changing GLOBALSCALER at runtime breaks
  // auto tuning.
  void SetMaxMotorCurrent();
 
  template <uint8_t kAddr>
  void WriteReg(uint32_t x);

  template <uint8_t kAddr>
  uint32_t ReadReg();

  template <uint8_t kAddr, uint8_t kBitStart, uint8_t kBitLen>
  void WriteRegPart(uint32_t x);
  template <RegField<uint8_t kAddr, uint8_t kBitStart, uint8_t kBitLen> kField>>
  void WriteRegPart(uint32_t x) { WriteRegPart<kAddr, kBitStart, kBitLen>(x); }

  template <uint8_t kAddr, uint8_t kBitStart, uint8_t kBitLen>
  uint32_t ReadRegPart();
  template <RegField<uint8_t kAddr, uint8_t kBitStart, uint8_t kBitLen> kField>>
  uint32_t ReadRegPart(uint32_t x) { return ReadRegPart<kAddr, kBitStart, kBitLen>(); }

  // TMC2160 is on the VSPI bus default pins.
  SPIClass* tmc_spi_;

  // This is the SPI status returned by the TMC2160 after each transaction.
  // It is updated after each transaction (read or write).
  uint8_t spi_status_;

  // This is the actual maximum current after GLOBALSCALER is applied. It should
  // be close to kMotorCurrentLimit, modulo rounding error, and applied minimum (due to
  // minimum restriction on GLOBALSCALER). When SetMotorCurrent is called we use this
  // to compute CS to set to achieve the requested current as a ratio of the actual
  // scaled current.
  float scaled_max_current_;
};

TMC2160Driver::TMC2160Driver()
  : tmc_spi_(nullptr), spi_status_(0) {}

void TMC2160Driver::Begin() {
  // Disable all motors.
  pinMode(kTmcEn, OUTPUT);
  digitalWrite(kTmcEn, HIGH);

  pinMode(kTmcStep, OUTPUT);
  pinMode(kTmcDir, OUTPUT);

  pinMode(kTmcDiag0, INPUT);
  pinMode(kTmcDiag1, INPUT);

  pinMode(kTmcDco, INPUT);
  pinMode(kTmcDcEn, OUTPUT);

  digitalWrite(kTmcDcEn, LOW);
  
  tmc_spi_ = new SPIClass(VSPI);
  tmc_spi_->begin();
  pinMode(tmc_spi_->pinSS(), OUTPUT);
  
  // This is based on the example initialisation sequence given in
  // 19.1 Initialization Examples
  WriteReg<kCHOPCONF>(0x000100c3);
  WriteReg<kIHOLD_IRUN>(0x00061f0a);
  WriteReg<kTPOWERDOWN>(0x0000000a);
  WriteReg<kGCONF>(0x00000004);
  WriteReg<kTPWMTHRS>(0x000001f4);

  SetMotorCurrent(kMotorCurrentLimit);

  // Enable motor drivers.
  digitalWrite(kTmcEn, LOW);
}

uint32_t TMC2160Driver::ReadStallGuardValue() {
  // TODO
  return 0;
}

void TMC2160Driver::SetStallGuardFiltering(bool filter_on) {
  // TODO
}

void TMC2160Driver::SetStallGuardThreshold(int8_t threshold) {
  // TODO
}

void TMC2160Driver::SetMotorCurrent(float new_current_setting) {
  // RMS current = GLOBALSCALER / 256 * (CS + 1) / 32 * Vfs / Rsense * 1 / sqrt(2)
  
}

void TMC2160Driver::SetMaxMotorCurrent() {
  // RMS current = GLOBALSCALER / 256 * (CS + 1) / 32 * Vfs / Rsense * 1 / sqrt(2)
  float full_scale_current_limit = kVFullScale / kRSense / kSqrt2;
  float scaler_f = 256.0f * kMotorCurrentLimit / full_scale_current_limit;

  // Minimum allowed value is 32.
  uint32_t clamped_scaler = RoundWithClamping(scaler_f, 32, 256);
  WriteReg<kGLOBALSCALER>((clamped_scaler == 256) ? 0, clamped_scaler);

  scaled_max_current_ = float(clamped_scaler) / 256.0f * full_scale_current_limit;

  Serial.printf("Requested max current: %f\n", kMotorCurrentLimit);
  Serial.printf("GLOBALSCALER set to: %u\n", clamped_scaler);
  Serial.printf("Actual max current: %f\n", scaled_max_current_);
}

// TMC2160 requires 40-bit transactions (8-bit addr + 32-bit value), and return 20-bit status.
template <uint8_t kAddr>
void TMC2160Driver::WriteReg(uint32_t x) {
  tmc_spi_->beginTransaction(kTmcSpiSettings);
  digitalWrite(tmc_spi_->pinSS(), LOW);
  spi_status_ = tmc_spi_->transfer(kAddr | 0x80);
  tmc_spi_->write32(x);
  tmc_spi_->endTransaction();
  digitalWrite(tmc_spi_->pinSS(), HIGH);
}

template <uint8_t kAddr>
uint32_t TMC2160Driver::ReadReg(uint8_t addr) {
  // For reads the register value is returned 1 transaction later, so we have to do 2
  // transactions. If higher bus efficiency is necessary we can pipeline transactions
  // to avoid having to do this, but for now let's keep it simple.
  uint32_t ret;
  for (int i = 0; i < 2; ++i) {
    tmc_spi_->beginTransaction(kTmcSpiSettings);
    digitalWrite(tmc_spi_->pinSS(), LOW);
    spi_status_ = tmc_spi_->transfer(kAddr & ~0x80);
    ret = tmc_spi_->transfer32(0);
    tmc_spi_->endTransaction();
    digitalWrite(tmc_spi_->pinSS(), HIGH);
  }
  return ret;
}

namespace {  
constexpr uint32_t ComputeMask(uint8_t bit_start, uint8_t bit_len) {
  return (bit_len >= 32 ? 0xffffffff : ((1UL << bit_len) - 1)) << bit_start;
}
}

template <uint8_t kAddr, uint8_t kBitStart, uint8_t kBitLen>
void TMC2160Driver::WriteRegPart(uint32_t x) {
  static_assert((uint32_t(kBitStart) + kBitLen) <= 32, "Field >32 wide?");
  
  uint32_t new_val = ReadReg<kAddr>();
  uint32_t mask = ComputeMask(kBitStart, kBitLen);
  new_val &= ~mask;
  new_val |= (x << kBitStart) & mask;
  WriteReg<kAddr>(new_val);
}

template <uint8_t kAddr, uint8_t kBitStart, uint8_t kBitLen>
uint32_t TMC2160Driver::ReadRegPart() {
  static_assert((uint32_t(kBitStart) + kBitLen) <= 32, "Field >32 wide?");
  
  uint32_t val = ReadReg<kAddr>();
  uint32_t mask = ComputeMask(kBitStart, kBitLen);
  return (val & mask) >> kBitStart;
}

#endif  // TMC2160
#endif  // TMC2160_H
