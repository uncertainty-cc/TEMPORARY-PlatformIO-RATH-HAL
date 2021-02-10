
#include "gd32vf1xx_hal_i2c.h"

void HAL_I2C_disable(I2C_TypeDef *I2Cx) {
  LL_I2C_disable(I2Cx);
}

void HAL_I2C_enable(I2C_TypeDef *I2Cx) {
  LL_I2C_enable(I2Cx);
}

uint32_t HAL_I2C_getFlag(I2C_TypeDef *I2Cx, uint32_t flag) {
  return LL_I2C_getFlag(I2Cx, flag);
}

void HAL_I2C_clearADDSENDFlag(I2C_TypeDef *I2Cx) {
  /* read I2C_STAT0 and then read I2C_STAT1 to clear ADDSEND */
  uint32_t temp;
  temp = READ_BITS(I2Cx->STAT0, 0);
  temp = READ_BITS(I2Cx->STAT1, 0);
}

void HAL_I2C_clearFlag(I2C_TypeDef *I2Cx, uint32_t flag) {
  if (I2C_FLAG_ADDSEND == flag) {
    HAL_I2C_clearADDSENDFlag(I2Cx);
  }
  LL_I2C_clearFlag(I2Cx, flag);
}

void HAL_I2C_configAckPosition(I2C_TypeDef *I2Cx, uint32_t value) {
  LL_I2C_configAckPosition(I2Cx, value);
}

void HAL_I2C_configClock(I2C_TypeDef *I2Cx, uint32_t clock_speed, uint32_t duty_cycle) {
  uint32_t pclk1, clkc, freq, risetime;

  pclk1 = HAL_RCU_getAPB1ClockFreq();
  /* I2C peripheral clock frequency */
  freq = (uint32_t) (pclk1 / 1000000U);
  if (freq >= I2CCLK_MAX) {
    freq = I2CCLK_MAX;
  }

  CLEAR_BITS(I2Cx->CTL1, I2C_CTL1_I2CCLK_MSK);
  SET_BITS(I2Cx->CTL1, freq);

  if (clock_speed <= 100000U) {
    /* the maximum SCL rise time is 1000ns in standard mode */
    risetime = (uint32_t) ((pclk1 / 1000000U) + 1U);
    if (risetime >= I2CCLK_MAX) {
        I2Cx->RT = I2CCLK_MAX;
    } else if (risetime <= I2CCLK_MIN) {
        I2Cx->RT = I2CCLK_MIN;
    } else {
        I2Cx->RT = risetime;
    }
    clkc = (uint32_t) (pclk1 / (clock_speed * 2UL));
    if (clkc < 0x04U) {
      /* the CLKC in standard mode minmum value is 4 */
      clkc = 0x04U;
    }
    SET_BITS(I2Cx->CKCFG, I2C_CKCFG_CLKC_MSK & clkc);

  } else if (clock_speed <= 400000U) {
    /* the maximum SCL rise time is 300ns in fast mode */
    I2Cx->RT = ((freq * 300UL) / 1000UL) + 1UL;
    if (I2C_DTCY_2 == duty_cycle) {
      /* I2C duty cycle is 2 */
      clkc = pclk1 / (clock_speed * 3UL);
      CLEAR_BITS(I2Cx->CKCFG, I2C_CKCFG_DTCY_MSK);
    } else {
      /* I2C duty cycle is 16/9 */
      clkc = pclk1 / (clock_speed * 25UL);
      SET_BITS(I2Cx->CKCFG, I2C_CKCFG_DTCY_MSK);
    }
    if (!READ_BITS(clkc, I2C_CKCFG_CLKC_MSK)) {
      /* the CLKC in fast mode minmum value is 1 */
      SET_BITS(clkc, 0x0001UL);
    }
    SET_BITS(I2Cx->CKCFG, clkc | I2C_CKCFG_FAST_MSK);
  }
}

void HAL_I2C_configModeAddr(I2C_TypeDef *I2Cx, uint32_t mode, uint32_t addformat, uint32_t address)  {
  CLEAR_BITS(I2Cx->CTL0, I2C_CTL0_SMBEN_MSK);
  SET_BITS(I2Cx->CTL0, mode);

  /* the address need to be shifted by 1 when writing into the register */
  I2Cx->SADDR0 = (addformat | (LL_I2C_offsetAddress(address) & I2C_ADDRESS_MASK));
}


void HAL_I2C_init(I2C_TypeDef *I2Cx, I2C_InitTypeDef *I2C_init) {
  LL_I2C_disable(I2Cx);

  HAL_I2C_configModeAddr(I2Cx, I2C_init->mode, I2C_init->addressing_mode, I2C_init->address);
  HAL_I2C_configClock(I2Cx, I2C_init->clock_speed, I2C_init->duty_cycle);

  LL_I2C_disableAcknowledge(I2Cx);
}

void HAL_I2C_setup(I2C_TypeDef *I2Cx, uint32_t mode, uint32_t clock_speed, uint32_t duty_cycle, 
    uint32_t addressing_mode, uint32_t address) {
  I2C_InitTypeDef I2C_init_struct;
  I2C_init_struct.mode = mode;
  I2C_init_struct.clock_speed = clock_speed;
  I2C_init_struct.duty_cycle = duty_cycle;
  I2C_init_struct.addressing_mode = addressing_mode;
  I2C_init_struct.address = address;
  HAL_I2C_init(I2Cx, &I2C_init_struct);
}

uint8_t HAL_I2C_waitForFlag(I2C_TypeDef *I2Cx, uint32_t flag, uint32_t state, uint64_t timestart, uint64_t timeout) {
  while (LL_I2C_getFlag(I2Cx, flag) != state) {
    if (timeout == 0UL) {
      continue;
    }
    if (HAL_getTime() - timestart > timeout) {
      return HAL_TIMEOUT;
    }
  }
  return HAL_OK;
}

uint8_t HAL_I2C_masterReceive(I2C_TypeDef *I2Cx, uint16_t device_addr, uint8_t *buffer, uint16_t size, uint64_t timeout) {
  /* this method implements the Solution B in the user manual for time independency. */
  uint64_t timestart = HAL_getTime();

  if (HAL_I2C_waitForFlag(I2Cx, I2C_FLAG_I2CBSY, RESET, timestart, I2C_BUSY_TIMEOUT) != HAL_OK) {
    return HAL_BUSY;
  }

  /* disable POAP */
  LL_I2C_configAckPosition(I2Cx, I2C_ACKPOS_CURRENT);

  /* enable acknowledge */
  LL_I2C_enableAcknowledge(I2Cx);

  /* if first transfer */
  /* step 2 for N=2: set POAP bit before START condition */
  if (size == 2) {
    LL_I2C_configAckPosition(I2Cx, I2C_ACKPOS_NEXT);
  }


  /* step 2 */
  /* generate START condition */
  LL_I2C_generateStart(I2Cx);

  /* step 3 */
  /* wait for hardware to set SBSEND bit */
  if (HAL_I2C_waitForFlag(I2Cx, I2C_FLAG_SBSEND, SET, timestart, timeout) != HAL_OK) {
    return HAL_TIMEOUT;
  }

  if (READ_BITS(I2Cx->SADDR0, I2C_SADDR0_ADDFORMAT_MSK) == I2C_ADDFORMAT_10BITS) {
    /* if in 10-bit mode */
    // TODO: implement 10-bit mode
  }
  else {
    /* if in 7-bit mode */
    LL_I2C_transmitData(I2Cx, LL_I2C_offsetAddress(device_addr) | 0b1UL);
    LL_I2C_clearFlag(I2Cx, I2C_FLAG_SBSEND);
  }

  /* step 4 */
  /* wait for hardware to set ADDSEND bit */
  if (HAL_I2C_waitForFlag(I2Cx, I2C_FLAG_ADDSEND, SET, timestart, timeout) != HAL_OK) {
    return HAL_TIMEOUT;
  }

  if (READ_BITS(I2Cx->SADDR0, I2C_SADDR0_ADDFORMAT_MSK) == I2C_ADDFORMAT_10BITS) {
    /* if in 10-bit mode */
    // TODO: implement 10-bit mode
  }



  /* continue step 4 */
  if (size == 0U) {
    /* clear ADDSEND flag */
    HAL_I2C_clearADDSENDFlag(I2Cx);

    /* generate STOP condition */
    LL_I2C_generateStop(I2Cx);
  }
  else if (size == 1U) {
    /* step 4 for N=1 */
    /* disable ACK and clear ADDR flag */
    LL_I2C_disableAcknowledge(I2Cx);
    HAL_I2C_clearADDSENDFlag(I2Cx);

    /* generate STOP condition */
    LL_I2C_generateStop(I2Cx);
  }
  else if (size == 2U) {
    /* disable ACK and clear ADDR flag */
    LL_I2C_disableAcknowledge(I2Cx);
    HAL_I2C_clearADDSENDFlag(I2Cx);
  }
  else {
    /* clear ADDR flag */
    HAL_I2C_clearADDSENDFlag(I2Cx);
  }

  while (size > 0U) {
    if (size <= 3U) {
      if (size == 1U) {
        /* step 5 for N=1 */
        /* wait for hardware to set RBNE bit */
        if (HAL_I2C_waitForFlag(I2Cx, I2C_FLAG_RBNE, SET, timestart, timeout) != HAL_OK) {
          return HAL_TIMEOUT;
        }

        /* reads DATA, RBNE is cleared automatically */
        *buffer = LL_I2C_receiveData(I2Cx);

        /* update counter */
        buffer += sizeof(uint8_t);
        size -= 1;
      }
      else if (size == 2U) {
        /* step 5 for N=2 */
        if (HAL_I2C_waitForFlag(I2Cx, I2C_FLAG_BTC, SET, timestart, timeout) != HAL_OK) {
          return HAL_TIMEOUT;
        }

        /* generate STOP condition */
        LL_I2C_generateStop(I2Cx);

        /* reads DATA twice */
        *buffer = LL_I2C_receiveData(I2Cx);

        buffer += sizeof(uint8_t);
        size -= 1;

        *buffer = LL_I2C_receiveData(I2Cx);

        buffer += sizeof(uint8_t);
        size -= 1;
      }
      else {
        /* N=3 (comes from normal receive which N>2, read the last N-2, N-1 & N byte here) */
        if (HAL_I2C_waitForFlag(I2Cx, I2C_FLAG_BTC, SET, timestart, timeout) != HAL_OK) {
          return HAL_TIMEOUT;
        }

        /* disable ACK */
        LL_I2C_disableAcknowledge(I2Cx);

        *buffer = LL_I2C_receiveData(I2Cx);
        buffer += sizeof(uint8_t);
        size -= 1;

        
        if (HAL_I2C_waitForFlag(I2Cx, I2C_FLAG_BTC, SET, timestart, timeout) != HAL_OK) {
          return HAL_TIMEOUT;
        }

        /* generate STOP condition */
        LL_I2C_generateStop(I2Cx);

        *buffer = LL_I2C_receiveData(I2Cx);
        buffer += sizeof(uint8_t);
        size -= 1;

        *buffer = LL_I2C_receiveData(I2Cx);
        buffer += sizeof(uint8_t);
        size -= 1;
      }
    }
    else {
      /* if size > 3 */
      if (HAL_I2C_waitForFlag(I2Cx, I2C_FLAG_RBNE, SET, timestart, timeout) != HAL_OK) {
        return HAL_TIMEOUT;
      }

      *buffer = LL_I2C_receiveData(I2Cx);
      buffer += 1;
      size -= 1;

      if (LL_I2C_getFlag(I2Cx, I2C_FLAG_BTC)) {
        *buffer = LL_I2C_receiveData(I2Cx);
        buffer += 1;
        size -= 1;
      }
    }
  }
  return 0;
}

uint8_t HAL_I2C_masterTransmit(I2C_TypeDef *I2Cx, uint16_t device_addr, uint8_t *buffer, uint16_t size, uint64_t timeout) {
  uint64_t timestart = HAL_getTime();

  if (HAL_I2C_waitForFlag(I2Cx, I2C_FLAG_I2CBSY, RESET, timestart, I2C_BUSY_TIMEOUT) != HAL_OK) {
    return HAL_BUSY;
  }

  LL_I2C_generateStart(I2Cx);

  if (HAL_I2C_waitForFlag(I2Cx, I2C_FLAG_SBSEND, SET, timestart, timeout) != HAL_OK) {
    LL_I2C_generateStop(I2Cx);
    return HAL_TIMEOUT;
  }

  LL_I2C_transmitData(I2Cx, LL_I2C_offsetAddress(device_addr));
  LL_I2C_clearFlag(I2Cx, I2C_FLAG_SBSEND);


  if (HAL_I2C_waitForFlag(I2Cx, I2C_FLAG_ADDSEND, SET, timestart, timeout) != HAL_OK) {
    LL_I2C_generateStop(I2Cx);
    return HAL_TIMEOUT;
  }
  HAL_I2C_clearADDSENDFlag(I2Cx);


  /* transmit all the data */
  while (size > 0) {
    /* wait for TBE (transmit buffer empty) flag to be cleared */    
    if (HAL_I2C_waitForFlag(I2Cx, I2C_FLAG_TBE, SET, timestart, timeout) != HAL_OK) {
      LL_I2C_generateStop(I2Cx);
      return HAL_TIMEOUT;
    }

    /* write data to transmit register */
    LL_I2C_transmitData(I2Cx, *buffer);
    buffer += 1;
    size -= 1;

    /* if this is the first byte to be transmitted, ignore TBE status and transmit another one */
    if ((LL_I2C_getFlag(I2Cx, I2C_FLAG_BTC)) && (size != 0U)) {
      /* write data to transmit register */
      LL_I2C_transmitData(I2Cx, *buffer);

      buffer += 1;
      size -= 1;
    }
  }

  if (HAL_I2C_waitForFlag(I2Cx, I2C_FLAG_BTC, SET, timestart, timeout) != HAL_OK) {
    LL_I2C_generateStop(I2Cx);
    return HAL_TIMEOUT;
  }

  LL_I2C_generateStop(I2Cx);

  return 0;
}


uint8_t HAL_I2C_slaveTransmit(I2C_TypeDef *I2Cx, uint8_t *buffer, uint16_t size, uint64_t timeout) {
  uint64_t timestart = HAL_getTime();

  LL_I2C_enableAcknowledge(I2Cx);

  /* TODO: If 10 bit addressing format
is selected, the I2C master should then send a repeated START(Sr) condition followed
by a header to the I2C bus. The slave sets ADDSEND bit again after it detects the
repeated START(Sr) condition and the following h eader. Software needs to clear the
ADDSEND bit again by reading I2C_STAT0 and then I2C_STAT1. */
  /* in 7-bit addr mode, only one ADDSEND need to be read and cleared */
  if (HAL_I2C_waitForFlag(I2Cx, I2C_FLAG_ADDSEND, SET, timestart, timeout) != HAL_OK) {
    return HAL_TIMEOUT;
  }
  HAL_I2C_clearADDSENDFlag(I2Cx);

  /* transmit all the data */
  while (size > 0) {    
    if (HAL_I2C_waitForFlag(I2Cx, I2C_FLAG_TBE, SET, timestart, timeout) != HAL_OK) {
      return HAL_TIMEOUT;
    }

    /* write data to transmit register */
    LL_I2C_transmitData(I2Cx, *buffer);
    buffer += 1;
    size -= 1;

    /* if this is the first byte to be transmitted, ignore TBE status and transmit another one */
    if ((LL_I2C_getFlag(I2Cx, I2C_FLAG_BTC)) && (size != 0U)) {
      /* write data to transmit register */
      LL_I2C_transmitData(I2Cx, *buffer);

      buffer += 1;
      size -= 1;
    }
  }

  if (HAL_I2C_waitForFlag(I2Cx, I2C_FLAG_AERR, SET, timestart, timeout) != HAL_OK) {
    return HAL_TIMEOUT;
  }
  LL_I2C_clearFlag(I2Cx, I2C_FLAG_AERR);

  return HAL_OK;
}

uint8_t HAL_I2C_slaveReceive(I2C_TypeDef *I2Cx, uint8_t *buffer, uint16_t size, uint64_t timeout) {
  uint64_t timestart = HAL_getTime();

  /* enable acknowledge */
  LL_I2C_enableAcknowledge(I2Cx);

  if (HAL_I2C_waitForFlag(I2Cx, I2C_FLAG_ADDSEND, SET, timestart, timeout) != HAL_OK) {
    return HAL_TIMEOUT;
  }
  HAL_I2C_clearADDSENDFlag(I2Cx);

  while (size > 0) {
    if (HAL_I2C_waitForFlag(I2Cx, I2C_FLAG_RBNE, SET, timestart, timeout) != HAL_OK) {
      return HAL_TIMEOUT;
    }
    /* read from income data register and clears flag for more income data */
    uint8_t c = LL_I2C_receiveData(I2Cx);
    *buffer = c;
    buffer += 1;
    size -= 1;
  }

  if (HAL_I2C_waitForFlag(I2Cx, I2C_FLAG_STPDET, SET, timestart, timeout) != HAL_OK) {
    return HAL_TIMEOUT;
  }

  /* resets all flags */
  LL_I2C_clearFlag(I2Cx, I2C_FLAG_STPDET);

  return HAL_OK;
}


uint8_t HAL_I2C_readMemory(I2C_TypeDef *I2Cx, uint16_t device_addr, uint8_t mem_addr, uint8_t *data, uint16_t size, uint64_t timeout) {
  uint8_t status;
  status = HAL_I2C_masterTransmit(I2Cx, device_addr, &mem_addr, sizeof(uint8_t), timeout);
  if (status != HAL_OK) {
    return status;
  }
  status = HAL_I2C_masterReceive(I2Cx, device_addr, data, size, timeout);
  return status;
}

uint8_t HAL_I2C_writeMemory(I2C_TypeDef *I2Cx, uint16_t device_addr, uint8_t mem_addr, uint8_t *data, uint16_t size, uint64_t timeout) {
  uint8_t status;
  uint8_t *buffer = malloc(size + sizeof(uint8_t));
  *buffer = mem_addr;
  memcpy(buffer + sizeof(uint8_t), data, size);
  status = HAL_I2C_masterTransmit(I2Cx, device_addr, buffer, size + sizeof(uint8_t), timeout);
  free(buffer);
  return status;
}
