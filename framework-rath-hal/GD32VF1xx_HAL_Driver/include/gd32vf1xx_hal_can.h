/**
 * @file gd32vf1xx_hal_can.h
 * @author -T.K.- (t_k_233@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2021-02-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/* include guard */
#ifndef __GD32VF1XX_HAL_CAN_H
#define __GD32VF1XX_HAL_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gd32vf1xx_hal_def.h"

/**
 * @brief CAN Init structure definition
 * 
 */
typedef struct {
	uint32_t working_mode;
	uint32_t resync_jump_width;
	uint32_t time_segment_1;
	uint32_t time_segment_2;
	uint32_t time_triggered;
	uint32_t auto_bus_off_recovery;
	uint32_t auto_wake_up;
	uint32_t auto_retrans;
	uint32_t rec_fifo_overwrite;
	uint32_t trans_fifo_order;
	uint16_t prescaler;
} CAN_InitTypeDef;

/**
 * @brief CAN Filter Init structure definition
 * 
 */
typedef struct {
	uint16_t filter_list_high;
	uint16_t filter_list_low;
	uint16_t filter_mask_high;
	uint16_t filter_mask_low;
	uint16_t filter_fifo_number;
	uint16_t filter_number;
	uint16_t filter_mode;
	uint16_t filter_bits;
	uint32_t filter_enable;
} CAN_FilterInitTypeDef;

/**
 * @brief CAN Message Init structure definition
 * 
 */
typedef struct {
	uint32_t sfid; /*!< standard format frame identifier */
	uint32_t efid; /*!< extended format frame identifier */
	uint8_t frame_format; /*!< format of frame, standard or extended format */
	uint8_t frame_type; /*!< type of frame, data or remote */
	uint8_t data_length; /*!< data length */
	uint8_t data[8]; /*!< receive data */
	uint8_t filter_index; /*!< filtering index */
} CAN_MessageTypeDef;

#define CAN_FILTERBITS_16BIT               0x00U
#define CAN_FILTERBITS_32BIT               0x01U

#define CAN_FILTER_MASK_16BITS             0x0000FFFFUL

#define CAN_FILTER_MODE_MASK                       (0b0UL << CAN_FMCFG_FMOD0_POS)  /** filter with mask mode */
#define CAN_FILTER_MODE_LIST                       (0b1UL << CAN_FMCFG_FMOD0_POS)  /** filter with list mode */

#define CAN_FRAME_FORMAT_STANDARD                 (0b0UL << CAN_TMI0_FF_POS)  /** data frame */
#define CAN_FRAME_FORMAT_EXTENDED                 (0b1UL << CAN_TMI0_FF_POS)  /** remote frame */

#define CAN_FIFO0                          0x00UL
#define CAN_FIFO1                          0x01UL

#define CAN_FRAME_TYPE_DATA                       (0b0UL << CAN_TMI0_FT_POS)	 /** data frame */
#define CAN_FRAME_TYPE_REMOTE                     (0b1UL << CAN_TMI0_FT_POS)	 /** remote frame */

#define CAN_MAILBOX0                       0x00U             /*!< mailbox0 */
#define CAN_MAILBOX1                       0x01U             /*!< mailbox1 */
#define CAN_MAILBOX2                       0x02U             /*!< mailbox2 */
#define CAN_NOMAILBOX                      0x03U             /*!< no mailbox empty */

/* operation mode options */
#define CAN_NORMAL_MODE                    ((0b0UL << CAN_BT_SCMOD_POS) | (0b0UL << CAN_BT_LCMOD_POS))
#define CAN_LOOPBACK_MODE                  ((0b0UL << CAN_BT_SCMOD_POS) | (0b1UL << CAN_BT_LCMOD_POS))
#define CAN_SILENT_MODE                    ((0b1UL << CAN_BT_SCMOD_POS) | (0b0UL << CAN_BT_LCMOD_POS))
#define CAN_SILENT_LOOPBACK_MODE           ((0b1UL << CAN_BT_SCMOD_POS) | (0b1UL << CAN_BT_LCMOD_POS))

/* resynchronisation jump width options */
#define CAN_BT_SJW_1TQ                     (0b00UL << CAN_BT_SJW_POS)
#define CAN_BT_SJW_2TQ                     (0b01UL << CAN_BT_SJW_POS)
#define CAN_BT_SJW_3TQ                     (0b10UL << CAN_BT_SJW_POS)
#define CAN_BT_SJW_4TQ                     (0b11UL << CAN_BT_SJW_POS)

/* time segment 1 options */
#define CAN_BT_BS1_1TQ                     (0b0000UL << CAN_BT_BS1_POS)
#define CAN_BT_BS1_2TQ                     (0b0001UL << CAN_BT_BS1_POS)
#define CAN_BT_BS1_3TQ                     (0b0010UL << CAN_BT_BS1_POS)
#define CAN_BT_BS1_4TQ                     (0b0011UL << CAN_BT_BS1_POS)
#define CAN_BT_BS1_5TQ                     (0b0100UL << CAN_BT_BS1_POS)
#define CAN_BT_BS1_6TQ                     (0b0101UL << CAN_BT_BS1_POS)
#define CAN_BT_BS1_7TQ                     (0b0110UL << CAN_BT_BS1_POS)
#define CAN_BT_BS1_8TQ                     (0b0111UL << CAN_BT_BS1_POS)
#define CAN_BT_BS1_9TQ                     (0b1000UL << CAN_BT_BS1_POS)
#define CAN_BT_BS1_10TQ                    (0b1001UL << CAN_BT_BS1_POS)
#define CAN_BT_BS1_11TQ                    (0b1010UL << CAN_BT_BS1_POS)
#define CAN_BT_BS1_12TQ                    (0b1011UL << CAN_BT_BS1_POS)
#define CAN_BT_BS1_13TQ                    (0b1100UL << CAN_BT_BS1_POS)
#define CAN_BT_BS1_14TQ                    (0b1101UL << CAN_BT_BS1_POS)
#define CAN_BT_BS1_15TQ                    (0b1110UL << CAN_BT_BS1_POS)
#define CAN_BT_BS1_16TQ                    (0b1111UL << CAN_BT_BS1_POS)

/* time segment 2 options */
#define CAN_BT_BS2_1TQ                     (0b000UL << CAN_BT_BS2_POS)
#define CAN_BT_BS2_2TQ                     (0b001UL << CAN_BT_BS2_POS)
#define CAN_BT_BS2_3TQ                     (0b010UL << CAN_BT_BS2_POS)
#define CAN_BT_BS2_4TQ                     (0b011UL << CAN_BT_BS2_POS)
#define CAN_BT_BS2_5TQ                     (0b100UL << CAN_BT_BS2_POS)
#define CAN_BT_BS2_6TQ                     (0b101UL << CAN_BT_BS2_POS)
#define CAN_BT_BS2_7TQ                     (0b110UL << CAN_BT_BS2_POS)
#define CAN_BT_BS2_8TQ                     (0b111UL << CAN_BT_BS2_POS)

/* CAN timeout */
#define CAN_TIMEOUT                        0x0000FFFFUL

/**
 * @brief Initializes the CANx peripheral according to the specified parameters in the CAN_init.
 * 
 * @param CANx the CAN peripheral (x = 0, 1)
 * @param CAN_init pointer to a CAN_InitTypeDef structure that contains the configuration 
 * information for the specified CAN peripheral
 * @return uint8_t HAL status
 */
uint8_t HAL_CAN_init(CAN_TypeDef *CANx, CAN_InitTypeDef *CAN_init);

void HAL_CAN_initFilter(CAN_FilterInitTypeDef* CAN_filter_init);

/**
 * @brief Transmits a CAN message in blocking mode.
 * 
 * @param CANx the CAN peripheral (x = 0, 1)
 * @param message pointer to a CAN_MessageTypeDef structure that contains the configuration 
 * information and data for the CAN message
 * @param timeout 
 * @return uint8_t HAL status
 */
uint8_t HAL_CAN_transmit(CAN_TypeDef *CANx, CAN_MessageTypeDef* message, uint64_t timeout);

/**
 * @brief Receives a CAN message in blocking mode.
 * 
 * @param CANx the CAN peripheral (x = 0, 1)
 * @param fifo_number 
 * @param message pointer to a CAN_MessageTypeDef structure that will store the configuration 
 * information and data for the CAN message
 * @param timeout 
 * @return uint8_t HAL status
 */
uint8_t HAL_CAN_receive(CAN_TypeDef *CANx, uint8_t fifo_number, CAN_MessageTypeDef* message, uint64_t timeout);

uint8_t HAL_CAN_getReceiveFIFOLength(CAN_TypeDef *CANx, uint8_t fifo_number);


#ifdef __cplusplus
}
#endif

#endif  // __GD32VF1XX_HAL_CAN_H
