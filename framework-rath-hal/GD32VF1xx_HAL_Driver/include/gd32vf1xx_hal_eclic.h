/**
 * @file gd32vf1xx_hal_eclic.h
 * @author -T.K.- (t_k_233@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2021-02-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/* include guard */
#ifndef __GD32VF1XX_HAL_ECLIC_H
#define __GD32VF1XX_HAL_ECLIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gd32vf1xx_hal_def.h"

#include "riscv_encoding.h"

void HAL_ECLIC_enableGlobalInterrupt(void);

void HAL_ECLIC_setPriorityGroup(uint32_t prigroup);

void HAL_ECLIC_enableIRQ(uint32_t source, uint8_t lvl_abs, uint8_t priority);



#ifdef __cplusplus
}
#endif

#endif  // __GD32VF1XX_HAL_ECLIC_H
