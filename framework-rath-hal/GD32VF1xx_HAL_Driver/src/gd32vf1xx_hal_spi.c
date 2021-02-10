
#include "gd32vf1xx_hal_spi.h"


void HAL_SPI_init(SPI_TypeDef *SPIx, SPI_InitTypeDef *SPI_init) {
    uint32_t settings = READ_BITS(SPIx->CTL0, SPI_CTL0_CRCEN_MSK | SPI_CTL0_CRCNT_MSK | SPI_CTL0_SPIEN_MSK);

    switch (SPI_init->mode) {
        case SPI_MODE_MFD:
        case SPI_MODE_MTU:
            CLEAR_BITS(settings, SPI_CTL0_RO_MSK | SPI_CTL0_BDEN_MSK | SPI_CTL0_BDOEN_MSK);
            SET_BITS(settings, SPI_CTL0_MSTMOD_MSK | SPI_CTL0_SWNSS_MSK);
            break;
        case SPI_MODE_MRU:
            CLEAR_BITS(settings, SPI_CTL0_BDEN_MSK | SPI_CTL0_BDOEN_MSK);
            SET_BITS(settings, SPI_CTL0_MSTMOD_MSK | SPI_CTL0_RO_MSK | SPI_CTL0_SWNSS_MSK);
            break;
        case SPI_MODE_MTB:
            CLEAR_BITS(settings, SPI_CTL0_RO_MSK);
            SET_BITS(settings, SPI_CTL0_MSTMOD_MSK | SPI_CTL0_BDEN_MSK | SPI_CTL0_BDOEN_MSK | SPI_CTL0_SWNSS_MSK);
            break;
        case SPI_MODE_MRB:
            CLEAR_BITS(settings, SPI_CTL0_RO_MSK | SPI_CTL0_BDOEN_MSK);
            SET_BITS(settings, SPI_CTL0_MSTMOD_MSK | SPI_CTL0_BDEN_MSK | SPI_CTL0_SWNSS_MSK);
            break;
        case SPI_MODE_SFD:
        case SPI_MODE_STU:
            CLEAR_BITS(settings, SPI_CTL0_MSTMOD_MSK | SPI_CTL0_RO_MSK | SPI_CTL0_BDEN_MSK | SPI_CTL0_BDOEN_MSK);
            break;
        case SPI_MODE_SRU:
            CLEAR_BITS(settings, SPI_CTL0_MSTMOD_MSK | SPI_CTL0_BDEN_MSK | SPI_CTL0_BDOEN_MSK);
            SET_BITS(settings, SPI_CTL0_RO_MSK);
            break;
        case SPI_MODE_STB:
            CLEAR_BITS(settings, SPI_CTL0_MSTMOD_MSK | SPI_CTL0_RO_MSK);
            SET_BITS(settings, SPI_CTL0_BDEN_MSK | SPI_CTL0_BDOEN_MSK);
            break;
        case SPI_MODE_SRB:
            CLEAR_BITS(settings, SPI_CTL0_MSTMOD_MSK | SPI_CTL0_RO_MSK | SPI_CTL0_BDOEN_MSK);
            SET_BITS(settings, SPI_CTL0_BDEN_MSK);
            break;
    }

    /* select SPI frame size */
    SET_BITS(settings, SPI_init->frame_size);
    /* select SPI NSS use hardware or software */
    SET_BITS(settings, SPI_init->nss);
    /* select SPI LSB or MSB */
    SET_BITS(settings, SPI_init->endian);
    /* select SPI polarity and phase */
    SET_BITS(settings, SPI_init->clock_polarity_phase);
    /* select SPI prescale to adjust transmit speed */
    SET_BITS(settings, SPI_init->prescaler);

    SPIx->CTL0 = settings;

    /* disable I2S and select SPI mode */
    CLEAR_BITS(SPIx->I2SCTL, SPI_I2SCTL_I2SSEL_MSK);
}

void HAL_SPI_setup(SPI_TypeDef *SPIx, uint32_t mode, uint32_t frame_size, uint32_t nss_type, uint32_t endian, uint32_t clock_mode, uint32_t prescaler) {
    SPI_InitTypeDef SPI_init_struct;
    SPI_init_struct.mode = mode;
    SPI_init_struct.frame_size = frame_size;
    SPI_init_struct.nss = nss_type;
    SPI_init_struct.endian = endian;
    SPI_init_struct.clock_polarity_phase = clock_mode;
    SPI_init_struct.prescaler = prescaler;
    
    HAL_SPI_init(SPIx, &SPI_init_struct);
}

void HAL_SPI_enable(SPI_TypeDef *SPIx) {
    SET_BITS(SPIx->CTL0, SPI_CTL0_SPIEN_MSK);
}

void HAL_SPI_disable(SPI_TypeDef *SPIx) {
    CLEAR_BITS(SPIx->CTL0, SPI_CTL0_SPIEN_MSK);
}

void HAL_SPI_enableCRC(SPI_TypeDef *SPIx) {
    SET_BITS(SPIx->CTL0, SPI_CTL0_CRCEN_MSK);
}

void HAL_SPI_disableCRC(SPI_TypeDef *SPIx) {
    CLEAR_BITS(SPIx->CTL0, SPI_CTL0_CRCEN_MSK);
}

uint8_t HAL_SPI_waitForFlag(SPI_TypeDef *SPIx, uint32_t flag, uint32_t state, uint64_t timestart, uint64_t timeout) {
  while (LL_SPI_getFlag(SPIx, flag) != state) {
    if (timeout == 0UL) {
      continue;
    }
    if (HAL_getTime() - timestart > timeout) {
      return HAL_TIMEOUT;
    }
  }
  return HAL_OK;
}

uint8_t HAL_SPI_masterTransmitReceive(SPI_TypeDef *SPIx, uint8_t *tx_buffer, uint8_t *rx_buffer, uint16_t size, uint64_t timeout) {
    uint64_t timestart = HAL_getTime();

    while (size > 0) {
        if (HAL_SPI_waitForFlag(SPIx, SPI_FLAG_TBE, SET, timestart, timeout) != HAL_OK) {
            return HAL_TIMEOUT;
        }
        LL_SPI_transmitData(SPIx, *tx_buffer);
        
        tx_buffer += sizeof(uint8_t);
    
        if (HAL_SPI_waitForFlag(SPIx, SPI_FLAG_RBNE, SET, timestart, timeout) != HAL_OK) {
            return HAL_TIMEOUT;
        }
        *rx_buffer = LL_SPI_receiveData(SPIx);

        rx_buffer += sizeof(uint8_t);
        size -= 1;
    }
    return HAL_OK;
}

uint8_t HAL_SPI_readMemory(SPI_TypeDef *SPIx, uint8_t mem_addr, uint8_t *data, uint16_t size, uint64_t timeout) {

}

uint8_t HAL_SPI_writeMemory(SPI_TypeDef *SPIx, uint8_t mem_addr, uint8_t *data, uint16_t size, uint64_t timeout) {
    
}

