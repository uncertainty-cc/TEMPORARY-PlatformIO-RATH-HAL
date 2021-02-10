/**
 * @file gd32vf1xx_hal_adc.h
 * @author -T.K.- (t_k_233@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2021-02-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/* include guard */
#ifndef __GD32VF1XX_HAL_ADC_H
#define __GD32VF1XX_HAL_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gd32vf1xx_hal_def.h"
#include "gd32vf1xx_hal_time.h"

/**
 * @brief ADC Init structure definition
 * 
 */
typedef struct {
  uint32_t mode;
  uint32_t data_align;
  uint32_t conversion_mode;
} ADC_InitTypeDef;

/* generic ADC bit masks */
#define ADC_RSQx_RSQx_MSK                 ADC_RSQ2_RSQ0_MSK  /** RSQ mask */
#define ADC_SAMPTx_SPTx_MSK               ADC_SAMPT1_SPT0_MSK  /** SPT mask */


#define ADC_REGULAR_CHANNEL                     0x01U
#define ADC_INSERTED_CHANNEL                    0x02U

#define ADC_INSERTED_CHANNEL_0                  0x00U
#define ADC_INSERTED_CHANNEL_1                  0x01U
#define ADC_INSERTED_CHANNEL_2                  0x02U
#define ADC_INSERTED_CHANNEL_3                  0x03U

#define ADC_CHANNEL_0                           0x00U
#define ADC_CHANNEL_1                           0x01U
#define ADC_CHANNEL_2                           0x02U
#define ADC_CHANNEL_3                           0x03U
#define ADC_CHANNEL_4                           0x04U
#define ADC_CHANNEL_5                           0x05U
#define ADC_CHANNEL_6                           0x06U
#define ADC_CHANNEL_7                           0x07U
#define ADC_CHANNEL_8                           0x08U
#define ADC_CHANNEL_9                           0x09U
#define ADC_CHANNEL_10                          0x0AU
#define ADC_CHANNEL_11                          0x0BU
#define ADC_CHANNEL_12                          0x0CU
#define ADC_CHANNEL_13                          0x0DU
#define ADC_CHANNEL_14                          0x0EU
#define ADC_CHANNEL_15                          0x0FU
#define ADC_CHANNEL_16                          0x10U
#define ADC_CHANNEL_17                          0x11U


#define  ADC_REGULAR_CHANNEL_RANK_SIX               6U
#define  ADC_REGULAR_CHANNEL_RANK_TWELVE            12U
#define  ADC_REGULAR_CHANNEL_RANK_SIXTEEN           16U
#define  ADC_REGULAR_CHANNEL_RANK_LENGTH            5U


#define  ADC_CHANNEL_SAMPLE_TEN                     10U
#define  ADC_CHANNEL_SAMPLE_EIGHTEEN                18U
#define  ADC_CHANNEL_SAMPLE_LENGTH                  3U

/* sync mode options */
#define ADC_MODE_FREE                                     (0b0000 << ADC_CTL0_SYNCM_POS)  /** free mode */
#define ADC_MODE_REGULAR_PARALLEL_INSERTED_PARALLEL       (0b0001 << ADC_CTL0_SYNCM_POS)  /** Combined regular parallel + inserted parallel mode */
// TODO: finish up sync modes

/* conversion options */
#define ADC_SINGLE_CONVERSION_MODE            0x01UL
#define ADC_CONTINUOUS_CONVERSION_MODE        0x02UL
#define ADC_SCAN_CONVERSION_MODE              0x03UL
#define ADC_DISCONTINUOUS_CONVERSION_MODE     0x04UL

/* data alignment options */
#define ADC_DATAALIGN_RIGHT                 (0b0 << ADC_CTL1_DAL_POS)  /** LSB align */
#define ADC_DATAALIGN_LEFT                  (0b1 << ADC_CTL1_DAL_POS)  /** MSB align */

/* trigger source options */
#define ADC_EXTTRIG_REGULAR_T0_CH0          (0b000UL << ADC_CTL1_ETSRC_POS)  /** Timer 0 CH0 as trigger source */
#define ADC_EXTTRIG_REGULAR_T0_CH1          (0b001UL << ADC_CTL1_ETSRC_POS)  /** Timer 0 CH1 as trigger source */
#define ADC_EXTTRIG_REGULAR_T0_CH2          (0b010UL << ADC_CTL1_ETSRC_POS)  /** Timer 0 CH2 as trigger source */
#define ADC_EXTTRIG_REGULAR_T1_CH1          (0b011UL << ADC_CTL1_ETSRC_POS)  /** Timer 1 CH1 as trigger source */
#define ADC_EXTTRIG_REGULAR_T2_TRGO         (0b100UL << ADC_CTL1_ETSRC_POS)  /** Timer 2 TRGO as trigger source */
#define ADC_EXTTRIG_REGULAR_T3_CH3          (0b101UL << ADC_CTL1_ETSRC_POS)  /** Timer 3 CH3 as trigger source */
#define ADC_EXTTRIG_REGULAR_EXTI_11         (0b110UL << ADC_CTL1_ETSRC_POS)  /** EXTI line 11 as trigger source */
#define ADC_EXTTRIG_REGULAR_NONE            (0b111UL << ADC_CTL1_ETSRC_POS)  /** software trigger as trigger source */


#define ADC_SAMPLETIME_1POINT5              (0b000UL << ADC_CTL1_ETSIC_POS)
#define ADC_SAMPLETIME_7POINT5              (0b001UL << ADC_CTL1_ETSIC_POS)
#define ADC_SAMPLETIME_13POINT5             (0b010UL << ADC_CTL1_ETSIC_POS)
#define ADC_SAMPLETIME_28POINT5             (0b011UL << ADC_CTL1_ETSIC_POS)
#define ADC_SAMPLETIME_41POINT5             (0b100UL << ADC_CTL1_ETSIC_POS)
#define ADC_SAMPLETIME_55POINT5             (0b101UL << ADC_CTL1_ETSIC_POS)
#define ADC_SAMPLETIME_71POINT5             (0b110UL << ADC_CTL1_ETSIC_POS)
#define ADC_SAMPLETIME_239POINT5            (0b111UL << ADC_CTL1_ETSIC_POS)   

/* ADC flag definition */
#define ADC_FLAG_WDE                              ADC_STAT_WDE_MSK    /** Analog watchdog event flag */
#define ADC_FLAG_EOC                              ADC_STAT_EOC_MSK    /** End of group conversion flag */
#define ADC_FLAG_EOIC                             ADC_STAT_EOIC_MSK   /** End of inserted group conversion flag */
#define ADC_FLAG_STIC                             ADC_STAT_STIC_MSK   /** Start of inserted channel group conversion flag */
#define ADC_FLAG_STRC                             ADC_STAT_STRC_MSK   /** Start of regular channel group conversion flag */


#define LL_ADC_disable(ADCx)                      CLEAR_BITS(ADCx->CTL1, ADC_CTL1_ADCON_MSK)
#define LL_ADC_enable(ADCx)                       SET_BITS(ADCx->CTL1, ADC_CTL1_ADCON_MSK)

#define LL_ADC_clearFlag(ADCx, flag)              CLEAR_BITS(ADCx->STAT, flag & FLAG_MASK)
#define LL_ADC_getFlag(ADCx, flag)                (READ_BITS(ADCx->STAT, flag & FLAG_MASK) ? SET : RESET)

#define LL_ADC_readRegularData(ADCx)              ((uint16_t)(ADCx->RDATA))
#define LL_ADC_readInsertedData0(ADCx)            ((uint16_t)(ADCx->IDATA0))
#define LL_ADC_readInsertedData1(ADCx)            ((uint16_t)(ADCx->IDATA1))
#define LL_ADC_readInsertedData2(ADCx)            ((uint16_t)(ADCx->IDATA2))
#define LL_ADC_readInsertedData3(ADCx)            ((uint16_t)(ADCx->IDATA3))

/**
 * @brief Initializes the ADCx peripheral according to the specified parameters in the ADC_init.
 * 
 * @param ADCx the ADC peripheral (x = 0, 1)
 * @param ADC_init pointer to a ADC_InitTypeDef structure that contains the configuration 
 * information for the specified ADC peripheral
 */
void HAL_ADC_init(ADC_TypeDef *ADCx, ADC_InitTypeDef* ADC_init);

void HAL_ADC_setup(ADC_TypeDef *ADCx, uint32_t mode, uint32_t data_align, uint32_t conversion_mode, uint32_t channel_length);

/**
 * @brief Enable the specified ADC peripheral.
 * 
 * @param ADCx the ADC peripheral (x = 0, 1)
 */
void HAL_ADC_enable(ADC_TypeDef *ADCx);

/**
 * @brief Disable the specified ADC peripheral.
 * 
 * @param ADCx the ADC peripheral (x = 0, 1)
 */
void HAL_ADC_disable(ADC_TypeDef *ADCx);

uint8_t HAL_ADC_getFlag(ADC_TypeDef *ADCx, uint32_t flag);

void HAL_ADC_clearFlag(ADC_TypeDef *ADCx, uint32_t flag);

uint8_t HAL_ADC_waitForFlag(ADC_TypeDef *ADCx, uint32_t flag, uint8_t state, uint32_t timestart, uint32_t timeout);

void HAL_ADC_configMode(ADC_TypeDef *ADCx, uint32_t mode);

void HAL_ADC_configConversionMode(ADC_TypeDef *ADCx, uint8_t adc_channel_group, uint32_t mode);

void HAL_ADC_configDataAlignment(ADC_TypeDef *ADCx, uint32_t value);

void HAL_ADC_configChannelLength(ADC_TypeDef *ADCx, uint8_t adc_channel_group, uint32_t length);

void HAL_ADC_configRegularChannel(ADC_TypeDef *ADCx, uint8_t rank, uint8_t adc_channel, uint32_t sample_time);

void HAL_ADC_configExternalTriggerSource(ADC_TypeDef *ADCx, uint8_t adc_channel_group, uint32_t external_trigger_source);

void HAL_ADC_configExternalTrigger(ADC_TypeDef *ADCx, uint8_t adc_channel_group, FunctionalState state);

void HAL_ADC_enableSoftwareTrigger(ADC_TypeDef *ADCx, uint8_t adc_channel_group);

uint16_t HAL_ADC_readRegularData(ADC_TypeDef *ADCx, uint32_t timeout);


#ifdef __cplusplus
}
#endif

#endif  // __GD32VF1XX_HAL_ADC_H
