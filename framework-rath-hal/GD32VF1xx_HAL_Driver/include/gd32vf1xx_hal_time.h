/* include guard */
#ifndef __GD32VF1XX_HAL_TIME_H
#define __GD32VF1XX_HAL_TIME_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gd32vf1xx_hal_def.h"     

#define LL_getTimerFreq()                         (HAL_RCU_getAHBClockFreq() >> 2)
#define LL_tick_2_time(tick)                      ((tick * 1000UL) / LL_getTimerFreq());
#define LL_time_2_tick(time)                      ((time * LL_getTimerFreq()) / 1000UL);


/**
 * @brief Get the current system tick
 * 
 * @return uint32_t current system tick
 */
uint32_t HAL_getTickW();

/**
 * @brief Get the current system tick
 * 
 * @return uint64_t current system tick
 */
uint64_t HAL_getTick();

/**
 * @brief Get the current system time
 * 
 * @return uint32_t current time, in milliseconds
 */
uint32_t HAL_getTimeW();

/**
 * @brief Get the current system time
 * 
 * @return uint64_t current time, in milliseconds
 */
uint64_t HAL_getTime();

/**
 * @brief Halt the processor for a given amount of time
 * 
 * @param time amount of time to sleep, in milliseconds
 */
void HAL_delay(uint64_t time);





#ifdef __cplusplus
}
#endif

#endif  // __GD32VF1XX_HAL_TIME_H
