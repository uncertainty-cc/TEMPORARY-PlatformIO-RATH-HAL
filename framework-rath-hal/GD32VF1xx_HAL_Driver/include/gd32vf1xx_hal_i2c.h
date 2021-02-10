/**
 * @file gd32vf1xx_hal_i2c.h
 * @author -T.K.- (t_k_233@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2021-01-23
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/* include guard */
#ifndef __GD32VF1XX_HAL_I2C_H
#define __GD32VF1XX_HAL_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gd32vf1xx_hal_def.h"
#include "gd32vf1xx_hal_rcu.h"
#include "gd32vf1xx_hal_time.h"

#define I2C_BUSY_TIMEOUT          25UL         /*!< Timeout 25 ms             */

typedef struct {
  uint32_t mode;
  uint32_t clock_speed;
  uint32_t duty_cycle;
  uint32_t addressing_mode;
  uint32_t address;
} I2C_InitTypeDef;

/* I2C flags */
/* flags in STAT0 register */
#define I2C_FLAG_SBSEND                       I2C_STAT0_SBSEND_MSK
#define I2C_FLAG_ADDSEND                      I2C_STAT0_ADDSEND_MSK
#define I2C_FLAG_BTC                          I2C_STAT0_BTC_MSK
#define I2C_FLAG_ADD10SEND                    I2C_STAT0_ADD10SEND_MSK
#define I2C_FLAG_STPDET                       I2C_STAT0_STPDET_MSK
#define I2C_FLAG_RBNE                         I2C_STAT0_RBNE_MSK
#define I2C_FLAG_TBE                          I2C_STAT0_TBE_MSK
#define I2C_FLAG_BERR                         I2C_STAT0_BERR_MSK
#define I2C_FLAG_LOSTARB                      I2C_STAT0_LOSTARB_MSK
#define I2C_FLAG_AERR                         I2C_STAT0_AERR_MSK
#define I2C_FLAG_OUERR                        I2C_STAT0_OUERR_MSK
#define I2C_FLAG_PECERR                       I2C_STAT0_PECERR_MSK
#define I2C_FLAG_SMBTO                        I2C_STAT0_SMBTO_MSK
#define I2C_FLAG_SMBALT                       I2C_STAT0_SMBALT_MSK
/* flags in STAT1 register */
#define I2C_FLAG_MASTER                       ((0b1UL << FLAG_OFFSET) | I2C_STAT1_MASTER_MSK)
#define I2C_FLAG_I2CBSY                       ((0b1UL << FLAG_OFFSET) | I2C_STAT1_I2CBSY_MSK)
#define I2C_FLAG_TR                           ((0b1UL << FLAG_OFFSET) | I2C_STAT1_TR_MSK)
#define I2C_FLAG_RXGC                         ((0b1UL << FLAG_OFFSET) | I2C_STAT1_RXGC_MSK)
#define I2C_FLAG_DEFSMB                       ((0b1UL << FLAG_OFFSET) | I2C_STAT1_DEFSMB_MSK)
#define I2C_FLAG_HSTSMB                       ((0b1UL << FLAG_OFFSET) | I2C_STAT1_HSTSMB_MSK)
#define I2C_FLAG_DUMODF                       ((0b1UL << FLAG_OFFSET) | I2C_STAT1_DUMODF_MSK)

/* I2C register bit mask */
#define I2CCLK_MAX                    0x00000048UL             /*!< i2cclk maximum value */
#define I2CCLK_MIN                    0x00000002UL             /*!< i2cclk minimum value */
#define I2C_ADDRESS_MASK              (0x000003FFUL << 1UL)             /*!< i2c address mask */
#define I2C_ADDRESS2_MASK             0x000000FEUL             /*!< the second i2c address mask */

/* I2C duty cycle in fast mode */
#define I2C_DTCY_2                    (0b0UL << I2C_CKCFG_DTCY_POS)                  /*!< I2C fast mode Tlow/Thigh = 2 */
#define I2C_DTCY_16_9                 (0b1UL << I2C_CKCFG_DTCY_POS)                           /*!< I2C fast mode Tlow/Thigh = 16/9 */

/* address mode for the I2C slave */
#define I2C_ADDFORMAT_7BITS           (0b0UL << I2C_SADDR0_ADDFORMAT_POS)                  /*!< address:7 bits */
#define I2C_ADDFORMAT_10BITS          (0b1UL << I2C_SADDR0_ADDFORMAT_POS)                     /*!< address:10 bits */


#define I2C_MODE_I2CMODE              (0b0UL << I2C_CTL0_SMBEN_POS)                  /*!< I2C mode */
#define I2C_MODE_SMBUSMODE            (0b1UL << I2C_CTL0_SMBEN_POS)                           /*!< SMBus mode */

#define I2C_ACKPOS_CURRENT            (0b0UL << I2C_CTL0_POAP_POS)
#define I2C_ACKPOS_NEXT               (0b1UL << I2C_CTL0_POAP_POS)

#define LL_I2C_disable(I2Cx)                      CLEAR_BITS(I2Cx->CTL0, I2C_CTL0_I2CEN_MSK)
#define LL_I2C_enable(I2Cx)                       SET_BITS(I2Cx->CTL0, I2C_CTL0_I2CEN_MSK)

#define LL_I2C_disableAcknowledge(I2Cx)           CLEAR_BITS(I2Cx->CTL0, I2C_CTL0_ACKEN_MSK)
#define LL_I2C_enableAcknowledge(I2Cx)            SET_BITS(I2Cx->CTL0, I2C_CTL0_ACKEN_MSK)

#define LL_I2C_getFlag(I2Cx, flag)                ((flag >> FLAG_OFFSET) ? \
                                                    (READ_BITS(I2Cx->STAT1, flag & FLAG_MASK) ? SET : RESET) : \
                                                    (READ_BITS(I2Cx->STAT0, flag & FLAG_MASK) ? SET : RESET))
#define LL_I2C_clearFlag(I2Cx, flag)              ((flag >> FLAG_OFFSET) ? \
                                                    CLEAR_BITS(I2Cx->STAT1, flag & FLAG_MASK) : \
                                                    CLEAR_BITS(I2Cx->STAT0, flag & FLAG_MASK))

#define LL_I2C_configAckPosition(I2Cx, value)     (value ? SET_BITS(I2Cx->CTL0, I2C_CTL0_POAP_MSK) : CLEAR_BITS(I2Cx->CTL0, I2C_CTL0_POAP_MSK))

#define LL_I2C_transmitData(I2Cx, data)           (I2Cx->DATA = READ_BITS(data, I2C_DATA_TRB_MSK))
#define LL_I2C_receiveData(I2Cx)                  ((uint8_t)READ_BITS(I2Cx->DATA, I2C_DATA_TRB_MSK))


#define LL_I2C_generateStart(I2Cx)                SET_BITS(I2Cx->CTL0, I2C_CTL0_START_MSK)
#define LL_I2C_generateStop(I2Cx)                 SET_BITS(I2Cx->CTL0, I2C_CTL0_STOP_MSK)

/* the address need to be shifted */
#define LL_I2C_offsetAddress(address)             (address << 1UL)


void HAL_I2C_disable(I2C_TypeDef *I2Cx);

void HAL_I2C_enable(I2C_TypeDef *I2Cx);

uint32_t HAL_I2C_getFlag(I2C_TypeDef *I2Cx, uint32_t flag);

void HAL_I2C_clearADDSENDFlag(I2C_TypeDef *I2Cx);

void HAL_I2C_clearFlag(I2C_TypeDef *I2Cx, uint32_t flag);

void HAL_I2C_configAckPosition(I2C_TypeDef *I2Cx, uint32_t value);

void HAL_I2C_configClock(I2C_TypeDef *I2Cx, uint32_t clock_speed, uint32_t duty_cycle);

void HAL_I2C_configModeAddr(I2C_TypeDef *I2Cx, uint32_t mode, uint32_t addformat, uint32_t address);

void HAL_I2C_init(I2C_TypeDef *I2Cx, I2C_InitTypeDef *I2C_init);

void HAL_I2C_setup(I2C_TypeDef *I2Cx, uint32_t mode, uint32_t clock_speed, uint32_t duty_cycle, 
    uint32_t addressing_mode, uint32_t address);


/**
 * @brief 
 * 
 * @param I2Cx 
 * @param flag 
 * @param state 
 * @param timestart 
 * @param timeout 
 * @return HAL_StatusTypeDef 
 */
uint8_t HAL_I2C_waitForFlag(I2C_TypeDef *I2Cx, uint32_t flag, uint32_t state, uint64_t timestart, uint64_t timeout);

/**
 * @brief Receive data from slave device in blocking mode
 *
 *
 * @param I2Cx
 * @param device_addr
 * @param buffer
 * @param size
 * @param timeout
 * @return uint8_t
 */
uint8_t HAL_I2C_masterReceive(I2C_TypeDef *I2Cx, uint16_t device_addr, uint8_t *buffer, uint16_t size, uint64_t timeout);

/**
 * @brief 
 * 
 * @param I2Cx 
 * @param device_addr 
 * @param buffer 
 * @param size 
 * @param timeout 
 * @return uint8_t 
 */
uint8_t HAL_I2C_masterTransmit(I2C_TypeDef *I2Cx, uint16_t device_addr, uint8_t *buffer, uint16_t size, uint64_t timeout);

/**
 * @brief 
 * 
 * @param I2Cx 
 * @param buffer 
 * @param size 
 * @param timeout 
 * @return uint8_t 
 */
uint8_t HAL_I2C_slaveTransmit(I2C_TypeDef *I2Cx, uint8_t *buffer, uint16_t size, uint64_t timeout);

/**
 * @brief
 *
 * example:\n
  '''\n
  uint8_t data[30];\n
  uint32_t size = 2;\n
  uint32_t timeout = 1000;\n
  uint8_t status = HAL_I2C_slaveReceive(&data, size, timeout);\n
  printf("%d  data: %d\t%d\n", status, data[0], data[1]);\n
  '''
 *
 * @param buffer
 * @param size
 * @param timeout
 * @return uint8_t
 */
uint8_t HAL_I2C_slaveReceive(I2C_TypeDef *I2Cx, uint8_t *buffer, uint16_t size, uint64_t timeout);

/**
 * @brief 
 * 
 * @param I2Cx 
 * @param device_addr 
 * @param mem_addr 
 * @param data 
 * @param size 
 * @param timeout 
 * @return uint8_t 
 */
uint8_t HAL_I2C_readMemory(I2C_TypeDef *I2Cx, uint16_t device_addr, uint8_t mem_addr, uint8_t *data, uint16_t size, uint64_t timeout);

/**
 * @brief 
 * 
 * @param I2Cx 
 * @param device_addr 
 * @param mem_addr 
 * @param data 
 * @param size 
 * @param timeout 
 * @return uint8_t 
 */
uint8_t HAL_I2C_writeMemory(I2C_TypeDef *I2Cx, uint16_t device_addr, uint8_t mem_addr, uint8_t *data, uint16_t size, uint64_t timeout);


#ifdef __cplusplus
}
#endif

#endif  // __GD32VF1XX_HAL_I2C_H
