#pragma once

// Host shim only. Production config.h and control modules are compiled unchanged.
#include <cstdint>

template <typename T>
constexpr T constrain(T value, T low, T high) {
  return value < low ? low : (value > high ? high : value);
}
