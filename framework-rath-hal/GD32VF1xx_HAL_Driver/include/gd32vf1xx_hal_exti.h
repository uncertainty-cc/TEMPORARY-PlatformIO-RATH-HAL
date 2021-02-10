/**
 * @file gd32vf1xx_hal_exti.h
 * @author -T.K.- (t_k_233@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2021-02-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/* include guard */
#ifndef __GD32VF1XX_HAL_EXTI_H
#define __GD32VF1XX_HAL_EXTI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gd32vf1xx_hal_def.h"

typedef enum {
    EXTI_0 =  0b0000000000000000001U,                                          /*!< EXTI line 0 */
    EXTI_1 =  0b0000000000000000010U,                                          /*!< EXTI line 1 */
    EXTI_2 =  0b0000000000000000100U,                                          /*!< EXTI line 2 */
    EXTI_3 =  0b0000000000000001000U,                                          /*!< EXTI line 3 */
    EXTI_4 =  0b0000000000000010000U,                                          /*!< EXTI line 4 */
    EXTI_5 =  0b0000000000000100000U,                                          /*!< EXTI line 5 */
    EXTI_6 =  0b0000000000001000000U,                                          /*!< EXTI line 6 */
    EXTI_7 =  0b0000000000010000000U,                                          /*!< EXTI line 7 */
    EXTI_8 =  0b0000000000100000000U,                                          /*!< EXTI line 8 */
    EXTI_9 =  0b0000000001000000000U,                                          /*!< EXTI line 9 */
    EXTI_10 = 0b0000000010000000000U,                                        /*!< EXTI line 10 */
    EXTI_11 = 0b0000000100000000000U,                                        /*!< EXTI line 11 */
    EXTI_12 = 0b0000001000000000000U,                                        /*!< EXTI line 12 */
    EXTI_13 = 0b0000010000000000000U,                                        /*!< EXTI line 13 */
    EXTI_14 = 0b0000100000000000000U,                                        /*!< EXTI line 14 */
    EXTI_15 = 0b0001000000000000000U,                                        /*!< EXTI line 15 */
    EXTI_16 = 0b0010000000000000000U,                                        /*!< EXTI line 16 */
    EXTI_17 = 0b0100000000000000000U,                                        /*!< EXTI line 17 */
    EXTI_18 = 0b1000000000000000000U,                                        /*!< EXTI line 18 */
} EXTI_Line_TypeDef;


/* external interrupt and event  */
typedef enum {
    EXTI_INTERRUPT = 0,                                       /*!< EXTI interrupt mode */
    EXTI_EVENT                                                /*!< EXTI event mode */
} exti_mode_enum;

/* interrupt trigger mode */
typedef enum {
    EXTI_TRIG_RISING = 0,                                     /*!< EXTI rising edge trigger */
    EXTI_TRIG_FALLING,                                        /*!< EXTI falling edge trigger */
    EXTI_TRIG_BOTH,                                           /*!< EXTI rising edge and falling edge trigger */
    EXTI_TRIG_NONE                                            /*!< without rising edge or falling edge trigger */
} exti_trig_type_enum;


void HAL_EXTI_setup(EXTI_Line_TypeDef linex, exti_mode_enum mode, exti_trig_type_enum trig_type) {
    /* reset the EXTI line x */
    EXTI->INTEN &= ~(uint32_t) linex;
    EXTI->EVEN &= ~(uint32_t) linex;
    EXTI->RTEN &= ~(uint32_t) linex;
    EXTI->FTEN &= ~(uint32_t) linex;

    /* set the EXTI mode and enable the interrupts or events from EXTI line x */
    switch (mode) {
    case EXTI_INTERRUPT:
        EXTI->INTEN |= (uint32_t) linex;
        break;
    case EXTI_EVENT:
        EXTI->EVEN |= (uint32_t) linex;
        break;
    default:
        break;
    }

    /* set the EXTI trigger type */
    switch (trig_type) {
    case EXTI_TRIG_RISING:
        EXTI->RTEN |= (uint32_t) linex;
        EXTI->FTEN &= ~(uint32_t) linex;
        break;
    case EXTI_TRIG_FALLING:
        EXTI->RTEN &= ~(uint32_t) linex;
        EXTI->FTEN |= (uint32_t) linex;
        break;
    case EXTI_TRIG_BOTH:
        EXTI->RTEN |= (uint32_t) linex;
        EXTI->FTEN |= (uint32_t) linex;
        break;
    case EXTI_TRIG_NONE:
    default:
        break;
    }
}


void exti_interrupt_flag_clear(EXTI_Line_TypeDef linex) {
    EXTI->PD = (uint32_t) linex;
}


void exti_interrupt_enable(EXTI_Line_TypeDef linex)
{
    EXTI->INTEN |= (uint32_t) linex;
}

void exti_interrupt_disable(EXTI_Line_TypeDef linex)
{
    EXTI->INTEN &= ~(uint32_t) linex;
}


uint32_t exti_flag_get(EXTI_Line_TypeDef linex)
{
    if (RESET != (EXTI->PD & (uint32_t) linex)) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      get EXTI lines flag when the interrupt flag is set
    \param[in]  linex: EXTI line number, refer to exti_line_enum
                only one parameter can be selected which is shown as below:
      \arg        EXTI_x (x=0..18): EXTI line x
    \param[out] none
    \retval     FlagStatus: status of flag (RESET or SET)
*/
uint32_t exti_interrupt_flag_get(EXTI_Line_TypeDef linex)
{
    uint32_t flag_left, flag_right;

    flag_left = EXTI->PD & (uint32_t) linex;
    flag_right = EXTI->INTEN & (uint32_t) linex;

    if ((RESET != flag_left) && (RESET != flag_right)) {
        return SET;
    } else {
        return RESET;
    }
}

#ifdef __cplusplus
}
#endif

#endif  // __GD32VF1XX_HAL_EXTI_H
