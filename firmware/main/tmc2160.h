#ifndef TMC2160_H
#define TMC2160_H

#ifdef TMC2160

#include "float_utils.h"
#include "tmc_driver.h"

#include <type_traits>

// Pin definitions ------------------------------------------------
// Drive low to enable motor drivers
constexpr int kTmcEn = 4;

// Stepping and direction outputs
constexpr int kTmcStep = 25;
constexpr int kTmcDir = 26;

constexpr int kTmcDiag0 = 15;
constexpr int kTmcDiag1 = 13;

constexpr int kTmcDco = 16;
constexpr int kTmcDcEn = 17;
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
struct RegFieldBase {};
template <uint8_t kAddr, uint8_t kBitStart, uint8_t kBitLen> struct RegField {
  constexpr static uint8_t Addr() { return kAddr; }
  constexpr static uint8_t BitStart() { return kBitStart; }
  constexpr static uint8_t BitLen() { return kBitLen; }
};

constexpr uint8_t kGCONF = 0x00;
constexpr uint8_t kDRV_CONF = 0x0a;
constexpr uint8_t kGLOBALSCALER = 0x0b;
constexpr uint8_t kIHOLD_IRUN = 0x10;
constexpr uint8_t kTPOWERDOWN = 0x11;
constexpr uint8_t kTPWMTHRS = 0x13;
constexpr uint8_t kTCOOLTHRS = 0x14;
constexpr uint8_t kCHOPCONF = 0x6c;
constexpr uint8_t kCOOLCONF = 0x6d;
constexpr uint8_t kDRV_STATUS = 0x6f;

using kDiag0Stall = RegField<kGCONF, 7, 1>;
using kDiag0IntPushPull = RegField<kGCONF, 12, 1>;

// Break before make delay. 0=100ns, 16=200ns, 24=375ns.
using kBBMTIME = RegField<kDRV_CONF, 0, 5>;
// Break before make in clock cycles (83ns * BBMCLKS).
// Highest of BBMCLKS and BBMTIME is used.
using kBBMCLKS = RegField<kDRV_CONF, 8, 4>;
using kDRVSTRENGTH = RegField<kDRV_CONF, 18, 2>; // Drive strength (0 = weakest, 3 = strongest).
using kFILT_ISENSE = RegField<kDRV_CONF, 20, 2>; // Filter time constant.

using kIHOLD = RegField<kIHOLD_IRUN, 0, 5>;
using kIRUN = RegField<kIHOLD_IRUN, 8, 5>;
using kIHOLDDELAY = RegField<kIHOLD_IRUN, 16, 4>;

using kDEDGE = RegField<kCHOPCONF, 29, 1>; // Step on both edges.
using kINTPOL = RegField<kCHOPCONF, 28, 1>; // MicroPlyer interpolation.
using kMRES = RegField<kCHOPCONF, 24, 4>; // Microstep resolution.
using kTPFD = RegField<kCHOPCONF, 20, 4>; // Passive fast decay time.
using kTBL = RegField<kCHOPCONF, 15, 2>; // TBL blanking time select.
using kHEND = RegField<kCHOPCONF, 7, 4>; // Hysteresis low value.
using kHSTART = RegField<kCHOPCONF, 4, 3>; // Hysteresis start value.
using kTOFF = RegField<kCHOPCONF, 0, 4>; // Off time and driver enable.

using kSFILT = RegField<kCOOLCONF, 24, 1>; // SG2 filter enable.
using kSGT = RegField<kCOOLCONF, 16, 7>; // SG2 threshold.

using kStandStillIndicator = RegField<kDRV_STATUS, 31, 1>;
using kOvertempPreWarning = RegField<kDRV_STATUS, 26, 1>;
using kOverTemp = RegField<kDRV_STATUS, 25, 1>;
using kCSActual = RegField<kDRV_STATUS, 16, 5>;
using kFullStepActive = RegField<kDRV_STATUS, 15, 1>;
using kStealth = RegField<kDRV_STATUS, 14, 1>;
using kSGResult = RegField<kDRV_STATUS, 0, 10>;
// ----------------------------------------------------------------

volatile bool g_stall_detected = false;

void IRAM_ATTR DriverStepHandler(void) {
  g_stall_detected |= digitalRead(kTmcDiag0);
}

namespace {
// Calculate TSTEP value from Rev/s. This is used to set TCOOLTHRS
// and THIGH. See "5.2 Velocity Dependent Driver Feature Control
// Register Set" for more info.
uint32_t CalculateTStepValue(float rps) {
  constexpr float kInternalClockRate = 12000000.0f; // 12 MHz
  // TStep is time between 2 1/256 microsteps in internal clock
  // cycles, regardless of actual microstepping setting.
  constexpr float k256MicrostepsPerRev = kFullStepsPerRev * 256;
  constexpr uint32_t kMaxValue = (1UL << 20) - 1;
  if (rps < kMinVelocity) {
    return kMaxValue;
  } else {
    float microstep_rate = rps * k256MicrostepsPerRev;
    uint32_t ret = static_cast<uint32_t>(kInternalClockRate / microstep_rate);
    if (ret > kMaxValue) {
      return kMaxValue;
    } else {
      return ret;
    }
  }
}
};

class TMC2160Driver : public TMCDriver {
 public:
  TMC2160Driver();

  void Begin() override;
  void Stop() override;

  uint32_t ReadStallGuardValue() override;
  void SetStallGuardFiltering(bool filter_on) override;
  void SetStallGuardThreshold(int8_t threshold) override;

  void SetMotorCurrent(float new_current_setting) override;

  int StepsPerRev() override { return kMicrostepsPerFullStep * kFullStepsPerRev; }

  void PrintDebugInfo() override;

  bool IsStalled() override;
  void ClearStallFlag() override;

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

  template <typename Field>
  uint32_t ReadRegPart();

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

  // These are caches for write-only registers.
  uint32_t coolconf_;
};

namespace {

constexpr uint32_t ComputeMask(uint8_t bit_start, uint8_t bit_len) {
  return (bit_len >= 32 ? 0xffffffff : ((1UL << bit_len) - 1)) << bit_start;
}

// Write a field to val (Field addr ignored).
template <typename Field>
uint32_t WriteField(uint32_t old_val, uint32_t new_val) {
  static_assert((uint32_t(Field::BitStart()) + Field::BitLen()) <= 32, "Field >32 wide?");
  uint32_t mask = ComputeMask(Field::BitStart(), Field::BitLen());
  return (old_val & ~mask) | ((new_val << Field::BitStart()) & mask);
}

// Extract a field from val (Field addr ignored).
template <typename Field>
uint32_t ExtractPart(uint32_t val) {
  static_assert((uint32_t(Field::BitStart()) + Field::BitLen()) <= 32, "Field >32 wide?");
  uint32_t mask = ComputeMask(Field::BitStart(), Field::BitLen());
  return (val & mask) >> Field::BitStart();
}

}

TMC2160Driver::TMC2160Driver()
  : tmc_spi_(nullptr), spi_status_(0), coolconf_(0) {}

void TMC2160Driver::Begin() {
  Serial.println("TMC2160 init");
  
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

  uint32_t gconf = 0x0;
  // Enable diag0 on stall in push-pull mode.
  gconf = WriteField<kDiag0Stall>(gconf, 1);
  gconf = WriteField<kDiag0IntPushPull>(gconf, 1);

  uint32_t chopconf = 0x000100c3;
  chopconf = WriteField<kDEDGE>(chopconf, 1);
  chopconf = WriteField<kINTPOL>(chopconf, 1);
  chopconf = WriteField<kMRES>(chopconf, kMicrostepsSetting);
  chopconf = WriteField<kTOFF>(chopconf, 5);
  chopconf = WriteField<kTBL>(chopconf, 2);

  // Effective hysteresis = 4 (Page 58 example).
  chopconf = WriteField<kHSTART>(chopconf, 2);
  chopconf = WriteField<kHEND>(chopconf, 4);

  uint32_t drvconf = 0;
  drvconf = WriteField<kBBMTIME>(drvconf, 0);
  drvconf = WriteField<kBBMCLKS>(drvconf, 0);
  drvconf = WriteField<kDRVSTRENGTH>(drvconf, 0);
  drvconf = WriteField<kFILT_ISENSE>(drvconf, 1);

  WriteReg<kGCONF>(gconf);
  WriteReg<kCHOPCONF>(chopconf);
  WriteReg<kDRV_CONF>(drvconf);
  WriteReg<kIHOLD_IRUN>(0x00061f0a);
  WriteReg<kTPOWERDOWN>(0x0000000a);
  WriteReg<kTPWMTHRS>(0x000001f4);
  WriteReg<kTCOOLTHRS>(CalculateTStepValue(kStallGuardMinSpeed));

  // Sanity check to make sure SPI is working. Unlike most other
  // registers, CHOPCONF is RW.
  for (;;) {
    uint32_t read_value = ReadReg<kCHOPCONF>();
    if (read_value == chopconf) {
      break;
    } else {
      Serial.printf("Unexpected CHOPCONF readback: %u\n", read_value);
      WriteReg<kCHOPCONF>(chopconf);
    }
    delay(1000);
  }

  SetMaxMotorCurrent();

  // Enable motor drivers.
  digitalWrite(kTmcEn, LOW);

  Serial.println("TMC2160 init done");
}

void TMC2160Driver::Stop() {
  // Disable motor drivers.
  digitalWrite(kTmcEn, HIGH);
}

uint32_t TMC2160Driver::ReadStallGuardValue() {
  return ReadRegPart<kSGResult>();
}

void TMC2160Driver::SetStallGuardFiltering(bool filter_on) {
  coolconf_ = WriteField<kSFILT>(coolconf_, filter_on ? 1 : 0);
  WriteReg<kCOOLCONF>(coolconf_);
}

void TMC2160Driver::SetStallGuardThreshold(int8_t threshold) {
  // Convert 8-bit two's complement to 7-bit by dropping the high bit
  uint8_t sgt_uint;
  std::memcpy(&sgt_uint, &threshold, 1);  
  coolconf_ = WriteField<kSGT>(coolconf_, sgt_uint & 0x7f);
  WriteReg<kCOOLCONF>(coolconf_);
}

void TMC2160Driver::SetMotorCurrent(float new_current_setting) {
  float current_ratio = new_current_setting / scaled_max_current_;
  uint32_t clamped_scaler = RoundWithClamping(current_ratio * 32.0f, 1, 32);
  
  Serial.printf("IHOLD_IRUN before: %u\n", ReadReg<kIHOLD_IRUN>());
  uint32_t new_val = 0;
  new_val = WriteField<kIRUN>(new_val, clamped_scaler - 1);
  new_val = WriteField<kIHOLD>(new_val, clamped_scaler - 1);
  new_val = WriteField<kIHOLDDELAY>(new_val, 0x7);
  WriteReg<kIHOLD_IRUN>(new_val);
}

void TMC2160Driver::PrintDebugInfo() {
  auto drv_status = ReadReg<kDRV_STATUS>();
  Serial.println("TMC2160 DRV_STATUS ==============================");
  Serial.printf("DRV_STATUS: %x\n", drv_status);
  Serial.printf("Stand still: %u\n", ExtractPart<kStandStillIndicator>(drv_status));
  Serial.printf("Overtemp pre-warning: %u\n", ExtractPart<kOvertempPreWarning>(drv_status));
  Serial.printf("Overtemp: %u\n", ExtractPart<kOverTemp>(drv_status));

  uint32_t cs_actual = ExtractPart<kCSActual>(drv_status);
  Serial.printf("CSActual (drv current / 32): %u (%fA)\n", cs_actual, scaled_max_current_ * (cs_actual + 1) / 32.0f);
  Serial.printf("Full step active: %u\n", ExtractPart<kFullStepActive>(drv_status));
  Serial.printf("StealthChop: %u\n", ExtractPart<kStealth>(drv_status));
  Serial.printf("StallGuard: %u\n", ExtractPart<kSGResult>(drv_status));
  Serial.println("=================================================");
}

void TMC2160Driver::SetMaxMotorCurrent() {
  // RMS current = GLOBALSCALER / 256 * (CS + 1) / 32 * Vfs / Rsense * 1 / sqrt(2)
  float full_scale_current_limit = kVFullScale / kRSense / kSqrt2;
  float scaler_f = 256.0f * kMotorCurrentLimit / full_scale_current_limit;

  // Minimum allowed value is 32.
  uint32_t clamped_scaler = RoundWithClamping(scaler_f, 32, 256);
  WriteReg<kGLOBALSCALER>((clamped_scaler == 256) ? 0 : clamped_scaler);

  scaled_max_current_ = float(clamped_scaler) / 256.0f * full_scale_current_limit;

  Serial.printf("Requested max current: %f\n", kMotorCurrentLimit);
  Serial.printf("GLOBALSCALER set to: %u\n", clamped_scaler);
  Serial.printf("Actual max current: %f\n", scaled_max_current_);

  // Apply the remaining correction using CS.
  SetMotorCurrent(kMotorCurrentLimit);
}

bool TMC2160Driver::IsStalled() {
  return g_stall_detected;
}

void TMC2160Driver::ClearStallFlag() {
  g_stall_detected = false;
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
uint32_t TMC2160Driver::ReadReg() {
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

template <typename Field>
uint32_t TMC2160Driver::ReadRegPart() {
  static_assert((uint32_t(Field::BitStart()) + Field::BitLen()) <= 32, "Field >32 wide?");
  
  uint32_t val = ReadReg<Field::Addr()>();
  uint32_t mask = ComputeMask(Field::BitStart(), Field::BitLen());
  return (val & mask) >> Field::BitStart();
}

#endif  // TMC2160
#endif  // TMC2160_H
