/**
 * @file gd32vf1xx_hal_def.h
 * @author -T.K.- (t_k_233@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2021-02-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/* include guard */
#ifndef __GD32VF1XX_HAL_DEF_H
#define __GD32VF1XX_HAL_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gd32vf103xb.h"

#define FLAG_OFFSET             0x10UL
#define FLAG_MASK               0xFFFF

enum {
  LOW   = RESET,
  HIGH  = SET,
};

typedef enum {
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

/**
  * @brief  HAL Lock structures definition
  */
typedef enum {
  HAL_UNLOCKED = 0x00U,
  HAL_LOCKED   = 0x01U
} HAL_LockTypeDef;



#ifdef __cplusplus
}
#endif

#endif  // __GD32VF1XX_HAL_DEF_H
