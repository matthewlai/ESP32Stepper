#ifndef DELAY_INSERTER_H
#define DELAY_INSERTER_H

// This class inserts an optional delay every time
// Sync() is called so the calling code executes
// at a maximum rate of the delay specified.
template <int32_t delay_us>
class DelayInserter {
 public:
  DelayInserter() : last_sync_time_(0) {}

  void Sync() {
    int32_t now = micros();
    int32_t delay_amount = delay_us - (now - last_sync_time_);

    // Do millisecond delay if we have >10ms to wait.
    if (delay_amount > 10000) {
      delay(delay_amount % 1000);
    }

    delay_amount = delay_us - (micros() - last_sync_time_);
    if (delay_amount > 0) {
      delayMicroseconds(delay_amount);
    }
    last_sync_time_ = micros();
  }
 
 private:
  int32_t last_sync_time_;
};

#endif  // DELAY_INSERTER_H
