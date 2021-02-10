/* include guard */
#ifndef __GD32VF1XX_HAL_SPI_H
#define __GD32VF1XX_HAL_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gd32vf1xx_hal_def.h"


typedef struct {
    uint32_t mode;                                                       /*!< SPI master or slave */
    uint32_t frame_size;                                                        /*!< SPI frame size */
    uint32_t nss;                                                               /*!< SPI NSS control by handware or software */
    uint32_t endian;                                                            /*!< SPI big endian or little endian */
    uint32_t clock_polarity_phase;                                              /*!< SPI clock phase and polarity */
    uint32_t prescaler;                                                          /*!< SPI prescale factor */
} SPI_InitTypeDef;



/* SPI mode definitions */
#define SPI_MODE_MASTER_FULL_DUPLEX                       0x0UL
#define SPI_MODE_MASTER_TRANSMIT_UNIDIRECTIONAL           0x1UL
#define SPI_MODE_MASTER_RECEIVE_UNIDIRECTIONAL            0x2UL
#define SPI_MODE_MASTER_TRANSMIT_BIDIRECTIONAL            0x3UL
#define SPI_MODE_MASTER_RECEIVE_BIDIRECTIONAL             0x4UL
#define SPI_MODE_SLAVE_FULL_DUPLEX                        0x5UL
#define SPI_MODE_SLAVE_TRANSMIT_UNIDIRECTIONAL            0x6UL
#define SPI_MODE_SLAVE_RECEIVE_UNIDIRECTIONAL             0x7UL
#define SPI_MODE_SLAVE_TRANSMIT_BIDIRECTIONAL             0x8UL
#define SPI_MODE_SLAVE_RECEIVE_BIDIRECTIONAL              0x9UL

/* mode alias */
#define SPI_MODE_MFD                                      SPI_MODE_MASTER_FULL_DUPLEX
#define SPI_MODE_MTU                                      SPI_MODE_MASTER_TRANSMIT_UNIDIRECTIONAL
#define SPI_MODE_MRU                                      SPI_MODE_MASTER_RECEIVE_UNIDIRECTIONAL
#define SPI_MODE_MTB                                      SPI_MODE_MASTER_TRANSMIT_BIDIRECTIONAL
#define SPI_MODE_MRB                                      SPI_MODE_MASTER_RECEIVE_BIDIRECTIONAL
#define SPI_MODE_SFD                                      SPI_MODE_SLAVE_FULL_DUPLEX
#define SPI_MODE_STU                                      SPI_MODE_SLAVE_TRANSMIT_UNIDIRECTIONAL
#define SPI_MODE_SRU                                      SPI_MODE_SLAVE_RECEIVE_UNIDIRECTIONAL
#define SPI_MODE_STB                                      SPI_MODE_SLAVE_TRANSMIT_BIDIRECTIONAL
#define SPI_MODE_SRB                                      SPI_MODE_SLAVE_RECEIVE_BIDIRECTIONAL

/* SPI frame size */
#define SPI_FRAMESIZE_8BIT              (0b0UL << SPI_CTL0_FF16_POS)
#define SPI_FRAMESIZE_16BIT             (0b1UL << SPI_CTL0_FF16_POS)

/* SPI NSS control mode */
#define SPI_NSS_HARD                    (0b0UL << SPI_CTL0_SWNSSEN_POS)
#define SPI_NSS_SOFT                    (0b1UL << SPI_CTL0_SWNSSEN_POS)

/* SPI endianness */
#define SPI_ENDIAN_MSB                  (0b0UL << SPI_CTL0_LF_POS)
#define SPI_ENDIAN_LSB                  (0b1UL << SPI_CTL0_LF_POS)

/* SPI clock phase and polarity */
#define SPI_CLK_PL_LOW_PH_1EDGE          ((0b0UL << SPI_CTL0_CKPL_POS) | (0b0UL << SPI_CTL0_CKPH_POS))
#define SPI_CLK_PL_HIGH_PH_1EDGE         ((0b1UL << SPI_CTL0_CKPL_POS) | (0b0UL << SPI_CTL0_CKPH_POS))
#define SPI_CLK_PL_LOW_PH_2EDGE          ((0b0UL << SPI_CTL0_CKPL_POS) | (0b1UL << SPI_CTL0_CKPH_POS))
#define SPI_CLK_PL_HIGH_PH_2EDGE         ((0b1UL << SPI_CTL0_CKPL_POS) | (0b1UL << SPI_CTL0_CKPH_POS))

#define SPI_CLK_MODE0                    SPI_CLK_PL_LOW_PH_1EDGE
#define SPI_CLK_MODE1                    SPI_CLK_PL_LOW_PH_2EDGE
#define SPI_CLK_MODE2                    SPI_CLK_PL_HIGH_PH_1EDGE
#define SPI_CLK_MODE3                    SPI_CLK_PL_HIGH_PH_2EDGE


/* SPI clock prescale factor */
#define SPI_PSC_2                       (0b000UL << SPI_CTL0_PSC_POS)
#define SPI_PSC_4                       (0b001UL << SPI_CTL0_PSC_POS)
#define SPI_PSC_8                       (0b010UL << SPI_CTL0_PSC_POS)
#define SPI_PSC_16                      (0b011UL << SPI_CTL0_PSC_POS)
#define SPI_PSC_32                      (0b100UL << SPI_CTL0_PSC_POS)
#define SPI_PSC_64                      (0b101UL << SPI_CTL0_PSC_POS)
#define SPI_PSC_128                     (0b110UL << SPI_CTL0_PSC_POS)
#define SPI_PSC_256                     (0b111UL << SPI_CTL0_PSC_POS)

/* SPI/I2S flag definitions */                                                  
#define SPI_FLAG_RBNE                   SPI_STAT_RBNE_MSK
#define SPI_FLAG_TBE                    SPI_STAT_TBE_MSK
#define SPI_FLAG_CRCERR                 SPI_STAT_CRCERR_MSK
#define SPI_FLAG_CONFERR                SPI_STAT_CONFERR_MSK
#define SPI_FLAG_RXORERR                SPI_STAT_RXORERR_MSK
#define SPI_FLAG_TRANS                  SPI_STAT_TRANS_MSK
#define SPI_FLAG_FERR                   SPI_STAT_FERR_MSK
#define I2S_FLAG_RBNE                   SPI_STAT_RBNE_MSK
#define I2S_FLAG_TBE                    SPI_STAT_TBE_MSK
#define I2S_FLAG_CH                     SPI_STAT_I2SCH_MSK
#define I2S_FLAG_TXURERR                SPI_STAT_TXURERR_MSK
#define I2S_FLAG_RXORERR                SPI_STAT_RXORERR_MSK
#define I2S_FLAG_TRANS                  SPI_STAT_TRANS_MSK
#define I2S_FLAG_FERR                   SPI_STAT_FERR_MSK

#define LL_SPI_transmitData(SPIx, data)             (SPIx->DATA = READ_BITS(data, SPI_DATA_SPI_DATA_MSK))
#define LL_SPI_receiveData(SPIx)                    READ_BITS(SPIx->DATA, SPI_DATA_SPI_DATA_MSK)

#define LL_SPI_getFlag(SPIx, flag)                  (READ_BITS(SPIx->STAT, flag & FLAG_MASK) ? SET : RESET)
#define LL_SPI_clearFlag(SPIx, flag)                CLEAR_BITS(SPIx->STAT, flag & FLAG_MASK)


void HAL_SPI_init(SPI_TypeDef *SPIx, SPI_InitTypeDef *SPI_init);

void HAL_SPI_setup(SPI_TypeDef *SPIx, uint32_t mode, uint32_t frame_size, uint32_t nss_type, uint32_t endian, uint32_t clock_mode, uint32_t prescaler);

void HAL_SPI_enable(SPI_TypeDef *SPIx);

void HAL_SPI_disable(SPI_TypeDef *SPIx);

void HAL_SPI_enableCRC(SPI_TypeDef *SPIx);

void HAL_SPI_disableCRC(SPI_TypeDef *SPIx);

uint8_t HAL_SPI_waitForFlag(SPI_TypeDef *SPIx, uint32_t flag, uint32_t state, uint64_t timestart, uint64_t timeout);

uint8_t HAL_SPI_masterTransmitReceive(SPI_TypeDef *SPIx, uint8_t *tx_buffer, uint8_t *rx_buffer, uint16_t size, uint64_t timeout);

uint8_t HAL_SPI_readMemory(SPI_TypeDef *SPIx, uint8_t mem_addr, uint8_t *data, uint16_t size, uint64_t timeout);

uint8_t HAL_SPI_writeMemory(SPI_TypeDef *SPIx, uint8_t mem_addr, uint8_t *data, uint16_t size, uint64_t timeout);


#ifdef __cplusplus
}
#endif

#endif  // __GD32VF1XX_HAL_SPI_H
