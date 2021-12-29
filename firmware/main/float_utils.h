#ifndef FLOAT_UTILS_H
#define FLOAT_UTILS_H

inline int32_t RoundWithClamping(float x, int32_t min_v, int32_t max_v) {
  bool neg = x < 0;
  int32_t ret = uint32_t(fabs(x) + 0.5f);
  if (neg) {
    ret *= -1;
  }
  return constrain(ret, min_v, max_v);
}

#endif  // FLOAT_UTILS_H
