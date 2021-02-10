/**
 * @file system_gd32vf1xx.h
 * @author -T.K.- (t_k_233@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2021-01-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */


/* include guard */
#ifndef __SYSTEM_GD32VF1XX_H
#define __SYSTEM_GD32VF1XX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "riscv_encoding.h"
#include "n200_func.h"
#include "gd32vf103xb.h"

extern uint32_t disable_mcycle_minstret();

void _init();
void _fini();


#ifdef __cplusplus
}
#endif

#endif  // __SYSTEM_GD32VF1XX_H
