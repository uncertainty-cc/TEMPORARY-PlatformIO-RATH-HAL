

#include "gd32vf1xx_hal_time.h"     



uint32_t HAL_getTickW() {
  return N200_getMtimeLo();
}

uint64_t HAL_getTick() {
  while (1) {
    uint32_t hi = N200_getMtimeHi();
    uint32_t lo = N200_getMtimeLo();
    if (hi == N200_getMtimeHi())
      return (((uint64_t)hi) << 32) | lo;
  }
}

uint32_t HAL_getTimeW() {
  // convert the clock tick to time in ms
  return LL_tick_2_time(HAL_getTickW());
}

uint64_t HAL_getTime() {
  // convert the clock tick to time in ms
  return LL_tick_2_time(HAL_getTick());
}

void HAL_delay(uint64_t time) {
  // Don't start measuruing until we see an mtime tick
  uint64_t start_mtime = HAL_getTick();
  uint64_t current_mtime = HAL_getTick();
  while (current_mtime == start_mtime) {
    current_mtime = HAL_getTick();
  }
  uint64_t target_mtime = current_mtime + LL_time_2_tick(time);
  while (HAL_getTick() < target_mtime) {}
}
