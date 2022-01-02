#ifndef TMC_DRIVER_H
#define TMC_DRIVER_H

class TMCDriver {
 public:
  virtual void Begin() = 0;
  virtual void Stop() = 0;
  virtual void SetMotorCurrent(float new_current_setting) = 0;
  virtual uint32_t ReadStallGuardValue() = 0;
  virtual void SetStallGuardFiltering(bool filter_on) = 0;
  virtual void SetStallGuardThreshold(int8_t threshold) = 0;
  virtual int StepsPerRev() = 0;
  virtual void PrintDebugInfo() {}

  bool IsStalled() { return ReadStallGuardValue() == 0; }
};

#endif  // TMC_DRIVER_H
