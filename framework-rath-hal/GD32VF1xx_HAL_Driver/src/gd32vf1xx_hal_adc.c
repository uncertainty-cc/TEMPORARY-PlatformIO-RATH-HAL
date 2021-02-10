/**
 * @file gd32vf1xx_hal_adc.c
 * @author -T.K.- (t_k_233@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2021-01-23
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "gd32vf1xx_hal_adc.h"


void HAL_ADC_init(ADC_TypeDef *ADCx, ADC_InitTypeDef* ADC_init) {
  HAL_ADC_configMode(ADCx, ADC_init->mode);
  HAL_ADC_configDataAlignment(ADCx, ADC_init->data_align);
  HAL_ADC_configConversionMode(ADCx, ADC_REGULAR_CHANNEL, ADC_init->conversion_mode);
  HAL_ADC_configChannelLength(ADCx, ADC_REGULAR_CHANNEL, 1);
}

void HAL_ADC_setup(ADC_TypeDef *ADCx, uint32_t mode, uint32_t data_align, uint32_t conversion_mode, uint32_t channel_length) {
  ADC_InitTypeDef adc_init_struct;
  adc_init_struct.mode = mode;
  adc_init_struct.conversion_mode = conversion_mode;
  adc_init_struct.data_align = data_align;
  HAL_ADC_init(ADCx, &adc_init_struct);
}

void HAL_ADC_enable(ADC_TypeDef *ADCx) {
  LL_ADC_enable(ADCx);
}

void HAL_ADC_disable(ADC_TypeDef *ADCx) {
  // TODO: disable functionality
}

uint8_t HAL_ADC_getFlag(ADC_TypeDef *ADCx, uint32_t flag) {
  return LL_ADC_getFlag(ADCx, flag);
}

void HAL_ADC_clearFlag(ADC_TypeDef *ADCx, uint32_t flag) {
  LL_ADC_clearFlag(ADCx, flag);
}

uint8_t HAL_ADC_waitForFlag(ADC_TypeDef *ADCx, uint32_t flag, uint8_t state, uint32_t timestart, uint32_t timeout) {
  while (LL_ADC_getFlag(ADC0, ADC_FLAG_EOC) != state) {
    if (timeout == 0UL) {
      continue;
    }
    if ((HAL_getTimeW() - timestart) > timeout) {
      return HAL_TIMEOUT;
    }
  }
  return HAL_OK;
}

void HAL_ADC_configMode(ADC_TypeDef *ADCx, uint32_t mode) {
  CLEAR_BITS(ADCx->CTL0, ADC_CTL0_SYNCM_MSK);
  SET_BITS(ADCx->CTL0, mode);
}

void HAL_ADC_configConversionMode(ADC_TypeDef *ADCx, uint8_t adc_channel_group, uint32_t mode) {
  if (adc_channel_group == ADC_REGULAR_CHANNEL) {
    switch (mode) {
      case ADC_SINGLE_CONVERSION_MODE:
        /* clear SM, DISRC, CTN */
        CLEAR_BITS(ADCx->CTL0, ADC_CTL0_SM_MSK | ADC_CTL0_DISRC_MSK);
        CLEAR_BITS(ADCx->CTL1, ADC_CTL1_CTN_MSK);
        break;
      case ADC_CONTINUOUS_CONVERSION_MODE:
        break;
    }
  }
  else {
    // TODO: inserted channel conversion config
  }
}

void HAL_ADC_configDataAlignment(ADC_TypeDef *ADCx, uint32_t value) {
  CLEAR_BITS(ADCx->CTL1, ADC_CTL1_DAL_POS);
  SET_BITS(ADCx->CTL1, value);
}

void HAL_ADC_configChannelLength(ADC_TypeDef *ADCx, uint8_t adc_channel_group, uint32_t length) {
  if (READ_BITS(adc_channel_group, ADC_REGULAR_CHANNEL)) {
    /* configure the length of regular channel group */
    CLEAR_BITS(ADCx->RSQ0, ((uint32_t)ADC_RSQ0_RL_MSK));
    SET_BITS(ADCx->RSQ0, (ADC_RSQ0_RL_MSK & (uint32_t)(length-1UL)) << ADC_RSQ0_RL_POS);
  }
  if (READ_BITS(adc_channel_group, ADC_INSERTED_CHANNEL)) {
    /* configure the length of inserted channel group */
    CLEAR_BITS(ADCx->ISQ, ((uint32_t)ADC_ISQ_IL_MSK));
    SET_BITS(ADCx->ISQ, (ADC_ISQ_IL_MSK & (uint32_t)(length-1UL)) << ADC_ISQ_IL_POS);
  }
}

void HAL_ADC_configRegularChannel(ADC_TypeDef *ADCx, uint8_t rank, uint8_t adc_channel, uint32_t sample_time) {
  uint32_t rsq, sampt;

  /* ADC regular sequence config */
  if (rank < ADC_REGULAR_CHANNEL_RANK_SIX) {
    /* the regular group sequence rank is smaller than six */
    rsq = ADCx->RSQ2;
    CLEAR_BITS(rsq, ((uint32_t)(ADC_RSQx_RSQx_MSK << (ADC_REGULAR_CHANNEL_RANK_LENGTH * rank))));
    /* the channel number is written to these bits to select a channel as the nth conversion in the regular channel group */
    SET_BITS(rsq, ((uint32_t)adc_channel << (ADC_REGULAR_CHANNEL_RANK_LENGTH * rank)));
    ADCx->RSQ2 = rsq;
  }
  else if (rank < ADC_REGULAR_CHANNEL_RANK_TWELVE) {
    /* the regular group sequence rank is smaller than twelve */
    rsq = ADCx->RSQ1;
    CLEAR_BITS(rsq, ((uint32_t)(ADC_RSQx_RSQx_MSK << (ADC_REGULAR_CHANNEL_RANK_LENGTH*(rank - ADC_REGULAR_CHANNEL_RANK_SIX)))));
    /* the channel number is written to these bits to select a channel as the nth conversion in the regular channel group */
    SET_BITS(rsq, ((uint32_t)adc_channel << (ADC_REGULAR_CHANNEL_RANK_LENGTH * (rank - ADC_REGULAR_CHANNEL_RANK_SIX))));
    ADCx->RSQ1 = rsq;
  }
  else if (rank < ADC_REGULAR_CHANNEL_RANK_SIXTEEN) {
    /* the regular group sequence rank is smaller than sixteen */
    rsq = ADCx->RSQ0;
    CLEAR_BITS(rsq, ((uint32_t)(ADC_RSQx_RSQx_MSK << (ADC_REGULAR_CHANNEL_RANK_LENGTH * (rank - ADC_REGULAR_CHANNEL_RANK_TWELVE)))));
    /* the channel number is written to these bits to select a channel as the nth conversion in the regular channel group */
    SET_BITS(rsq, ((uint32_t)adc_channel << (ADC_REGULAR_CHANNEL_RANK_LENGTH * (rank - ADC_REGULAR_CHANNEL_RANK_TWELVE))));
    ADCx->RSQ0 = rsq;
  }

  /* ADC sampling time config */
  if (adc_channel < ADC_CHANNEL_SAMPLE_TEN) {
    /* the regular group sequence rank is smaller than ten */
    sampt = ADCx->SAMPT1;
    CLEAR_BITS(sampt, ADC_SAMPTx_SPTx_MSK << (ADC_CHANNEL_SAMPLE_LENGTH * adc_channel));
    /* channel sample time set*/
    SET_BITS(sampt, sample_time << (ADC_CHANNEL_SAMPLE_LENGTH * adc_channel));
    ADCx->SAMPT1 = sampt;
  }
  else if (adc_channel < ADC_CHANNEL_SAMPLE_EIGHTEEN) {
    /* the regular group sequence rank is smaller than eighteen */
    sampt = ADCx->SAMPT0;
    CLEAR_BITS(sampt, ADC_SAMPTx_SPTx_MSK << (ADC_CHANNEL_SAMPLE_LENGTH * (adc_channel - ADC_CHANNEL_SAMPLE_TEN)));
    /* channel sample time set*/
    SET_BITS(sampt, sample_time << (ADC_CHANNEL_SAMPLE_LENGTH * (adc_channel - ADC_CHANNEL_SAMPLE_TEN)));
    ADCx->SAMPT0 = sampt;
  }
}

void HAL_ADC_configExternalTriggerSource(ADC_TypeDef *ADCx, uint8_t adc_channel_group, uint32_t external_trigger_source) {
  if (READ_BITS(adc_channel_group, ADC_REGULAR_CHANNEL)) {
    /* configure ADC regular group external trigger source */
    CLEAR_BITS(ADCx->CTL1, ADC_CTL1_ETSRC_MSK);
    SET_BITS(ADCx->CTL1, external_trigger_source);
  }
  if (READ_BITS(adc_channel_group, ADC_INSERTED_CHANNEL)) {
    /* configure ADC inserted group external trigger source */
    CLEAR_BITS(ADCx->CTL1, ADC_CTL1_ETSIC_MSK);
    SET_BITS(ADCx->CTL1, external_trigger_source);
  }
}

void HAL_ADC_configExternalTrigger(ADC_TypeDef *ADCx, uint8_t adc_channel_group, FunctionalState state) {
  if (READ_BITS(adc_channel_group, ADC_REGULAR_CHANNEL)) {
    /* disable ADC regular channel group external trigger */
    if (state) {
      SET_BITS(ADCx->CTL1, ADC_CTL1_ETERC_MSK);
    }
    else {
      CLEAR_BITS(ADCx->CTL1, ADC_CTL1_ETERC_MSK);
    }
  }
  if (READ_BITS(adc_channel_group, ADC_INSERTED_CHANNEL)) {
    /* disable ADC regular channel group external trigger */
    if (state) {
      SET_BITS(ADCx->CTL1, ADC_CTL1_ETEIC_MSK);
    }
    else {
      CLEAR_BITS(ADCx->CTL1, ADC_CTL1_ETEIC_MSK);
    }
  }
}

void HAL_ADC_enableSoftwareTrigger(ADC_TypeDef *ADCx, uint8_t adc_channel_group) {
  if (READ_BITS(adc_channel_group, ADC_REGULAR_CHANNEL)) {
    /* enable ADC regular channel group software trigger */
    SET_BITS(ADCx->CTL1, ADC_CTL1_SWRCST_MSK);
  }
  if (READ_BITS(adc_channel_group, ADC_INSERTED_CHANNEL)) {
    /* enable ADC inserted channel group software trigger */
    SET_BITS(ADCx->CTL1, ADC_CTL1_SWICST_MSK);
  }
}

uint16_t HAL_ADC_readRegularData(ADC_TypeDef *ADCx, uint32_t timeout) {
  uint32_t timestart = HAL_getTimeW();

  if (HAL_ADC_waitForFlag(ADCx, ADC_FLAG_EOC, SET, timestart, timeout)) {
    return 0;
  }
  uint16_t data = LL_ADC_readRegularData(ADCx);
  LL_ADC_clearFlag(ADC0, ADC_FLAG_EOC);
  return data;
}

