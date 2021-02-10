
#include "gd32vf1xx_hal_timer.h"


void HAL_TIMER_init(TIMER_TypeDef *TIMERx, TIMER_InitTypeDef* TIMER_init) {
  /* configure the counter prescaler value */
  TIMERx->PSC = (uint16_t)TIMER_init->prescaler;

  /* configure the counter direction and aligned mode */
  CLEAR_BITS(TIMERx->CTL0, TIMER_CTL0_DIR_MSK);
  SET_BITS(TIMERx->CTL0, (TIMER_init->counter_direction & TIMER_CTL0_DIR_MSK));
  if ((TIMERx == TIMER0) || (TIMERx == TIMER1) || (TIMERx == TIMER2)
    || (TIMERx == TIMER3) || (TIMERx == TIMER4) ) {
    CLEAR_BITS(TIMERx->CTL0, TIMER_CTL0_CAM_MSK);
    SET_BITS(TIMERx->CTL0, (TIMER_init->align_mode & TIMER_CTL0_CAM_MSK));
  }

  /* configure the autoreload value */
  TIMERx->CAR = (uint32_t)TIMER_init->period;
  if ((TIMERx != TIMER5) && (TIMERx != TIMER6)) {
    /* reset the CKDIV bit */
    CLEAR_BITS(TIMERx->CTL0, TIMER_CTL0_CKDIV_MSK);
    SET_BITS(TIMERx->CTL0, (TIMER_init->clock_division & TIMER_CTL0_CKDIV_MSK));
  }

  if (TIMERx == TIMER0) {
    /* configure the repetition counter value */
    TIMERx->CREP = TIMER_init->repetition_counter;
  }

  /* generate an update event */
  LL_TIMER_generateUpdateEvent(TIMERx);
}

void HAL_TIMER_setup(TIMER_TypeDef *TIMERx, uint32_t prescaler, uint32_t align_mode, uint32_t counter_direction, uint32_t period, uint32_t clock_division, uint32_t repetition_counter) {
  TIMER_InitTypeDef timer_init_struct;
  timer_init_struct.prescaler = prescaler;
  timer_init_struct.align_mode = align_mode;
  timer_init_struct.counter_direction = counter_direction;
  timer_init_struct.period = period;
  timer_init_struct.clock_division = clock_division;
  timer_init_struct.repetition_counter = repetition_counter;
  HAL_TIMER_init(TIMERx, &timer_init_struct);
}

void HAL_TIMER_configChannelOutput(TIMER_TypeDef *TIMERx, uint16_t channel, TIMER_OCInitTypeDef *ocpara) {
  switch(channel) {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
      CLEAR_BITS(TIMERx->CHCTL2, (TIMER_CHCTL2_CH0EN_MSK | TIMER_CHCTL2_CH0P_MSK));
      SET_BITS(TIMERx->CHCTL2, (ocpara->output_state | ocpara->oc_polarity) << TIMER_CHCTL2_CH0EN_POS);

      if (TIMERx == TIMER0) {
        CLEAR_BITS(TIMERx->CHCTL2, (TIMER_CHCTL2_CH0NEN_MSK | TIMER_CHCTL2_CH0NP_MSK));
        SET_BITS(TIMERx->CHCTL2, (ocpara->outputn_state | ocpara->ocn_polarity) << TIMER_CHCTL2_CH0EN_POS);
        CLEAR_BITS(TIMERx->CTL1, (TIMER_CTL1_ISO0_MSK | TIMER_CTL1_ISO0N_MSK));
        SET_BITS(TIMERx->CTL1, (ocpara->oc_idlestate | ocpara->ocn_idlestate) << 0UL);
      }
      CLEAR_BITS(TIMERx->CHCTL0, TIMER_CHCTL0_Output_CH0MS_MSK);
      break;

    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
      CLEAR_BITS(TIMERx->CHCTL2, (TIMER_CHCTL2_CH1EN_MSK | TIMER_CHCTL2_CH1P_MSK));
      SET_BITS(TIMERx->CHCTL2, (ocpara->output_state | ocpara->oc_polarity) << TIMER_CHCTL2_CH1EN_POS);

      if (TIMERx == TIMER0) {

        CLEAR_BITS(TIMERx->CHCTL2, (TIMER_CHCTL2_CH1NEN_MSK | TIMER_CHCTL2_CH1NP_MSK));
        SET_BITS(TIMERx->CHCTL2, (ocpara->outputn_state | ocpara->ocn_polarity) << TIMER_CHCTL2_CH1EN_POS);
        CLEAR_BITS(TIMERx->CTL1, (TIMER_CTL1_ISO1_MSK | TIMER_CTL1_ISO1N_MSK));
        SET_BITS(TIMERx->CTL1, (ocpara->oc_idlestate | ocpara->ocn_idlestate) << 2UL);
      }
      CLEAR_BITS(TIMERx->CHCTL0, TIMER_CHCTL0_Output_CH1MS_MSK);
      break;

    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
      CLEAR_BITS(TIMERx->CHCTL2, (TIMER_CHCTL2_CH2EN_MSK | TIMER_CHCTL2_CH2P_MSK));
      SET_BITS(TIMERx->CHCTL2, (ocpara->output_state | ocpara->oc_polarity) << TIMER_CHCTL2_CH2EN_POS);

      if (TIMERx == TIMER0) {
        CLEAR_BITS(TIMERx->CHCTL2, (TIMER_CHCTL2_CH2NEN_MSK | TIMER_CHCTL2_CH2NP_MSK));
        SET_BITS(TIMERx->CHCTL2, (ocpara->outputn_state | ocpara->ocn_polarity) << TIMER_CHCTL2_CH2EN_POS);
        CLEAR_BITS(TIMERx->CTL1, (TIMER_CTL1_ISO2_MSK | TIMER_CTL1_ISO2N_MSK));
        SET_BITS(TIMERx->CTL1, (ocpara->oc_idlestate | ocpara->ocn_idlestate) << 4UL);
      }
      CLEAR_BITS(TIMERx->CHCTL0, TIMER_CHCTL1_Output_CH2MS_MSK);
      break;

    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
      CLEAR_BITS(TIMERx->CHCTL2, (TIMER_CHCTL2_CH3EN_MSK | TIMER_CHCTL2_CH3P_MSK));
      SET_BITS(TIMERx->CHCTL2, (ocpara->output_state | ocpara->oc_polarity) << TIMER_CHCTL2_CH3EN_POS);

      if (TIMERx == TIMER0) {
        CLEAR_BITS(TIMERx->CTL1, (TIMER_CTL1_ISO3_MSK));
        SET_BITS(TIMERx->CTL1, (ocpara->oc_idlestate | ocpara->ocn_idlestate) << 6UL);
      }
      CLEAR_BITS(TIMERx->CHCTL0, TIMER_CHCTL1_Output_CH3MS_MSK);
      break;
  }
}

void HAL_TIMER_configAutoreloadValue(TIMER_TypeDef *TIMERx, uint16_t value) {
  LL_TIMER_setAutoReloadValue(TIMERx, (uint32_t)value);
}


void HAL_TIMER_configChannelOutputMode(TIMER_TypeDef *TIMERx, uint16_t channel, uint16_t mode) {
  uint32_t ocmode = (uint32_t)mode;
  switch (channel) {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
      CLEAR_BITS(TIMERx->CHCTL0, TIMER_CHCTL0_Output_CH0COMCTL_MSK);
      SET_BITS(TIMERx->CHCTL0, ocmode);
      break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
      CLEAR_BITS(TIMERx->CHCTL0, TIMER_CHCTL0_Output_CH1COMCTL_MSK);
      SET_BITS(TIMERx->CHCTL0, ocmode << TIMER_CHCTL0_Output_CH1MS_POS);
      break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
      CLEAR_BITS(TIMERx->CHCTL1, TIMER_CHCTL1_Output_CH2COMCTL_MSK);
      SET_BITS(TIMERx->CHCTL1, ocmode);
      break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
      CLEAR_BITS(TIMERx->CHCTL1, TIMER_CHCTL1_Output_CH3COMCTL_MSK);
      SET_BITS(TIMERx->CHCTL1, ocmode << TIMER_CHCTL1_Output_CH3MS_POS);
      break;
  }
}


void HAL_TIMER_configChannelOutputPulseValue(TIMER_TypeDef *TIMERx, uint16_t channel, uint32_t pulse) {
  switch (channel) {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
      TIMERx->CH0CV = pulse;
      break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
      TIMERx->CH1CV = pulse;
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
      TIMERx->CH2CV = pulse;
        break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
      TIMERx->CH3CV = pulse;
        break;
  }
}

void HAL_TIMER_configPrimaryOutput(TIMER_TypeDef *TIMERx, FunctionalState state) {
  if (state) {
    SET_BITS(TIMERx->CCHP, TIMER_CCHP_POEN_MSK);
  }
  else {
    CLEAR_BITS(TIMERx->CCHP, TIMER_CCHP_POEN_MSK);
  }
}

void HAL_TIMER_enable(TIMER_TypeDef *TIMERx) {
  LL_TIMER_enable(TIMERx);
}

void HAL_TIMER_disable(TIMER_TypeDef *TIMERx) {
  LL_TIMER_disable(TIMERx);
}


uint8_t HAL_TIMER_getFlag(TIMER_TypeDef *TIMERx, uint32_t flag) {
  return LL_TIMER_getFlag(TIMERx, flag);
}

void HAL_TIMER_clearFlag(TIMER_TypeDef *TIMERx, uint32_t flag) {
  LL_TIMER_clearFlag(TIMERx, flag);
}

void HAL_TIMER_enableInterrupt(TIMER_TypeDef *TIMERx, uint32_t interrupt) {
  SET_BITS(TIMERx->DMAINTEN, interrupt); 
}
