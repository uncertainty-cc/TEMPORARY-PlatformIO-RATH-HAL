/* include guard */

#ifndef __GD32VF1XX_HAL_TIMER_H
#define __GD32VF1XX_HAL_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gd32vf1xx_hal_def.h"

typedef struct {
  uint16_t prescaler;                                      /*!< prescaler value */
  uint16_t align_mode;                                    /*!< aligned mode */
  uint16_t counter_direction;                               /*!< counter direction */
  uint32_t period;                                         /*!< period value */
  uint16_t clock_division;                                  /*!< clock division value */
  uint8_t  repetition_counter;                              /*!< the counter repetition value */
} TIMER_InitTypeDef;

typedef struct {
    uint16_t output_state;                                    /*!< channel output state */
    uint16_t outputn_state;                                   /*!< channel complementary output state */
    uint16_t oc_polarity;                                     /*!< channel output polarity */
    uint16_t ocn_polarity;                                    /*!< channel complementary output polarity */
    uint16_t oc_idlestate;                                    /*!< idle state of channel output */
    uint16_t ocn_idlestate;                                   /*!< idle state of channel complementary output */
} TIMER_OCInitTypeDef;



/* TIMER channel n(n=0,1,2,3) */
#define TIMER_CH_0                              ((uint16_t)0x0000U)                     /*!< TIMER channel 0(TIMERx(x=0..4)) */
#define TIMER_CH_1                              ((uint16_t)0x0001U)                     /*!< TIMER channel 1(TIMERx(x=0..4)) */
#define TIMER_CH_2                              ((uint16_t)0x0002U)                     /*!< TIMER channel 2(TIMERx(x=0..4)) */
#define TIMER_CH_3                              ((uint16_t)0x0004U)

#define TIMER_ALIGN_EDGE                        (0b00UL << TIMER_CTL0_CAM_POS)                             /*!< edge-aligned mode */
#define TIMER_ALIGN_CENTER_DOWN                 (0b01UL << TIMER_CTL0_CAM_POS)                             /*!< center-aligned and counting down assert mode */
#define TIMER_ALIGN_CENTER_UP                   (0b10UL << TIMER_CTL0_CAM_POS)                             /*!< center-aligned and counting up assert mode */
#define TIMER_ALIGN_CENTER_BOTH                 (0b11UL << TIMER_CTL0_CAM_POS)

#define TIMER_COUNTER_DIR_UP                    (0b0UL << TIMER_CTL0_DIR_POS)              /*!< counter up direction */
#define TIMER_COUNTER_DIR_DOWN                  (0b1UL << TIMER_CTL0_DIR_POS)              /*!< counter down direction */

#define TIMER_CKDIV_DIV1                        (0b00UL << TIMER_CTL0_CKDIV_POS)           /*!< clock division value is 1,fDTS=fTIMER_CK */
#define TIMER_CKDIV_DIV2                        (0b01UL << TIMER_CTL0_CKDIV_POS)           /*!< clock division value is 2,fDTS= fTIMER_CK/2 */
#define TIMER_CKDIV_DIV4                        (0b10UL << TIMER_CTL0_CKDIV_POS)

/* channel enable state */
#define TIMER_CCX_DISABLE                       (0b0UL << TIMER_CHCTL2_CH0EN_POS)                     /*!< channel disable */
#define TIMER_CCX_ENABLE                        (0b1UL << TIMER_CHCTL2_CH0EN_POS)                     /*!< channel enable */


/* channel complementary output enable state */
#define TIMER_CCXN_DISABLE                      (0b0UL << TIMER_CHCTL2_CH0NEN_POS)                     /*!< channel complementary disable */
#define TIMER_CCXN_ENABLE                       (0b1UL << TIMER_CHCTL2_CH0NEN_POS)                     /*!< channel complementary enable */

/* channel output polarity */
#define TIMER_OC_POLARITY_HIGH                  (0b0UL << TIMER_CHCTL2_CH0P_POS)                     /*!< channel output polarity is high */
#define TIMER_OC_POLARITY_LOW                   (0b1UL << TIMER_CHCTL2_CH0P_POS)                     /*!< channel output polarity is low */

/* channel complementary output polarity */
#define TIMER_OCN_POLARITY_HIGH                 (0b0UL << TIMER_CHCTL2_CH0NP_POS)                     /*!< channel output polarity is high */
#define TIMER_OCN_POLARITY_LOW                  (0b1UL << TIMER_CHCTL2_CH0NP_POS)                     /*!< channel output polarity is low */

/* idle state of channel output */
#define TIMER_OC_IDLE_STATE_LOW                 (0b0UL << TIMER_CTL1_ISO0_POS)                      /*!< idle state of channel output is low */
#define TIMER_OC_IDLE_STATE_HIGH                (0b1UL << TIMER_CTL1_ISO0_POS)                      /*!< idle state of channel output is high */

/* idle state of channel complementary output */
#define TIMER_OCN_IDLE_STATE_LOW                (0b0UL << TIMER_CTL1_ISO0N_POS)                     /*!< idle state of channel complementary output is low */
#define TIMER_OCN_IDLE_STATE_HIGH               (0b1UL << TIMER_CTL1_ISO0N_POS)                     /*!< idle state of channel complementary output is high */

/* channel output compare mode */
#define TIMER_OC_MODE_TIMING                    ((uint16_t)0x0000U)                     /*!< timing mode */
#define TIMER_OC_MODE_ACTIVE                    ((uint16_t)0x0010U)                     /*!< active mode */
#define TIMER_OC_MODE_INACTIVE                  ((uint16_t)0x0020U)                     /*!< inactive mode */
#define TIMER_OC_MODE_TOGGLE                    ((uint16_t)0x0030U)                     /*!< toggle mode */
#define TIMER_OC_MODE_LOW                       ((uint16_t)0x0040U)                     /*!< force low mode */
#define TIMER_OC_MODE_HIGH                      ((uint16_t)0x0050U)                     /*!< force high mode */
#define TIMER_OC_MODE_PWM0                      ((uint16_t)0x0060U)                     /*!< PWM0 mode */
#define TIMER_OC_MODE_PWM1                      ((uint16_t)0x0070U)                     /*!< PWM1 mode */

/* channel output compare shadow enable */
#define TIMER_OC_SHADOW_DISABLE                 ((uint16_t)0x0000U)                     /*!< channel output shadow state disable */
#define TIMER_OC_SHADOW_ENABLE                  ((uint16_t)0x0008U)                     /*!< channel output shadow state enable */

/* channel output compare fast enable */
#define TIMER_OC_FAST_DISABLE               ((uint16_t)0x0000)                      /*!< channel output fast function disable */
#define TIMER_OC_FAST_ENABLE                ((uint16_t)0x0004)                      /*!< channel output fast function enable */

/* channel output compare clear enable */
#define TIMER_OC_CLEAR_DISABLE              ((uint16_t)0x0000U)                     /*!< channel output clear function disable */
#define TIMER_OC_CLEAR_ENABLE               ((uint16_t)0x0080U)                     /*!< channel output clear function enable */

/* channel control shadow register update control */
#define TIMER_UPDATECTL_CCUTRI              TIMER_CTL1_CCUC                         /*!< the shadow registers update by when CMTG bit is set or an rising edge of TRGI occurs */
#define TIMER_UPDATECTL_CCU                 ((uint32_t)0x00000000U)                 /*!< the shadow registers update by when CMTG bit is set */


/* TIMER interrupt enable or disable */
#define TIMER_INT_UP                        TIMER_DMAINTEN_UPIE_MSK                     /*!< update interrupt */
#define TIMER_INT_CH0                       TIMER_DMAINTEN_CH0IE_MSK                    /*!< channel 0 interrupt */
#define TIMER_INT_CH1                       TIMER_DMAINTEN_CH1IE_MSK                    /*!< channel 1 interrupt */
#define TIMER_INT_CH2                       TIMER_DMAINTEN_CH2IE_MSK                    /*!< channel 2 interrupt */
#define TIMER_INT_CH3                       TIMER_DMAINTEN_CH3IE_MSK                    /*!< channel 3 interrupt */
#define TIMER_INT_CMT                       TIMER_DMAINTEN_CMTIE_MSK                    /*!< channel commutation interrupt flag */
#define TIMER_INT_TRG                       TIMER_DMAINTEN_TRGIE_MSK                    /*!< trigger interrupt */
#define TIMER_INT_BRK                       TIMER_DMAINTEN_BRKIE_MSK                    /*!< break interrupt */


/* TIMER flag */
#define TIMER_FLAG_UP                       TIMER_INTF_UPIF_MSK                         /*!< update flag */
#define TIMER_FLAG_CH0                      TIMER_INTF_CH0IF_MSK                        /*!< channel 0 flag */
#define TIMER_FLAG_CH1                      TIMER_INTF_CH1IF_MSK                        /*!< channel 1 flag */
#define TIMER_FLAG_CH2                      TIMER_INTF_CH2IF_MSK                        /*!< channel 2 flag */
#define TIMER_FLAG_CH3                      TIMER_INTF_CH3IF_MSK                        /*!< channel 3 flag */
#define TIMER_FLAG_CMT                      TIMER_INTF_CMTIF_MSK                        /*!< channel control update flag */
#define TIMER_FLAG_TRG                      TIMER_INTF_TRGIF_MSK                        /*!< trigger flag */
#define TIMER_FLAG_BRK                      TIMER_INTF_BRKIF_MSK                        /*!< break flag */
#define TIMER_FLAG_CH0O                     TIMER_INTF_CH0OF_MSK                        /*!< channel 0 overcapture flag */
#define TIMER_FLAG_CH1O                     TIMER_INTF_CH1OF_MSK                        /*!< channel 1 overcapture flag */
#define TIMER_FLAG_CH2O                     TIMER_INTF_CH2OF_MSK                        /*!< channel 2 overcapture flag */
#define TIMER_FLAG_CH3O                     TIMER_INTF_CH3OF_MSK 


#define LL_TIMER_disable(TIMERx)                  CLEAR_BITS(TIMERx->CTL0, TIMER_CTL0_CEN_MSK)
#define LL_TIMER_enable(TIMERx)                   SET_BITS(TIMERx->CTL0, TIMER_CTL0_CEN_MSK)

#define LL_TIMER_generateUpdateEvent(TIMERx)        SET_BITS(TIMERx->SWEVG, TIMER_SWEVG_UPG_MSK)
#define LL_TIMER_setAutoReloadValue(TIMERx, value)  (TIMERx->CAR = value);


#define LL_TIMER_getFlag(TIMERx, flag)                  (READ_BITS(TIMERx->INTF, flag & FLAG_MASK) ? SET : RESET)
#define LL_TIMER_clearFlag(TIMERx, flag)                CLEAR_BITS(TIMERx->INTF, flag & FLAG_MASK)


void HAL_TIMER_init(TIMER_TypeDef *TIMERx, TIMER_InitTypeDef* TIMER_init);

void HAL_TIMER_setup(TIMER_TypeDef *TIMERx, uint32_t prescaler, uint32_t align_mode, uint32_t counter_direction, uint32_t period, uint32_t clock_division, uint32_t repetition_counter);

void HAL_TIMER_configChannelOutput(TIMER_TypeDef *TIMERx, uint16_t channel, TIMER_OCInitTypeDef *ocpara);

void HAL_TIMER_configAutoreloadValue(TIMER_TypeDef *TIMERx, uint16_t value);

void HAL_TIMER_configChannelOutputMode(TIMER_TypeDef *TIMERx, uint16_t channel, uint16_t mode);

void HAL_TIMER_configChannelOutputPulseValue(TIMER_TypeDef *TIMERx, uint16_t channel, uint32_t pulse);

void HAL_TIMER_configPrimaryOutput(TIMER_TypeDef *TIMERx, FunctionalState state);

void HAL_TIMER_enable(TIMER_TypeDef *TIMERx);

void HAL_TIMER_disable(TIMER_TypeDef *TIMERx);

uint8_t HAL_TIMER_getFlag(TIMER_TypeDef *TIMERx, uint32_t flag);

void HAL_TIMER_clearFlag(TIMER_TypeDef *TIMERx, uint32_t flag);

void HAL_TIMER_enableInterrupt(TIMER_TypeDef *TIMERx, uint32_t interrupt);

#ifdef __cplusplus
}
#endif

#endif  // __GD32VF1XX_HAL_TIMER_H
