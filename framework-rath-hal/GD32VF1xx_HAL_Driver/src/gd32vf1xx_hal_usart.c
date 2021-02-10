
#include "gd32vf1xx_hal_usart.h"


void HAL_USART_enablePrintFloat(void) {
  asm(".global _printf_float");
}

void HAL_USART_configBaudrate(USART_TypeDef *USARTx, uint32_t value) {
  uint32_t uclk = 0UL, intdiv = 0UL, fradiv = 0UL, udiv = 0UL;
  if (USARTx == USART0) {
    /* USART0 is using APB2 clock */
    uclk = HAL_RCU_getAPB2ClockFreq();
  }
  else {
    uclk = HAL_RCU_getAPB1ClockFreq();
  }
  
  /* oversampling by 16, configure the value of USART_BAUD */
  udiv = (uclk+value / 2UL) / value;
  intdiv = udiv & (0x0000fff0UL);
  fradiv = udiv & (0x0000000fUL);
  USARTx->BAUD = ((USART_BAUD_FRADIV_MSK | USART_BAUD_INTDIV_MSK) & (intdiv | fradiv));
}


void HAL_USART_configWordLength(USART_TypeDef *USARTx, uint32_t value) {
  /* clear USART_CTL0 WL bit */
  CLEAR_BITS(USARTx->CTL0, USART_CTL0_WL_MSK);
  /* configure USART word length */
  SET_BITS(USARTx->CTL0, value);
}

void HAL_USART_configStopBit(USART_TypeDef *USARTx, uint32_t value) {
  /* clear USART_CTL1 STB bits */
  CLEAR_BITS(USARTx->CTL1, USART_CTL1_STB_MSK); 
  /* configure USART stop bits */
  SET_BITS(USARTx->CTL1, value);
}

void HAL_USART_configParity(USART_TypeDef *USARTx, uint32_t value) {
  /* clear USART_CTL0 PM,PCEN bits */
  CLEAR_BITS(USARTx->CTL0, (USART_CTL0_PM_MSK | USART_CTL0_PCEN_MSK));
  /* configure USART parity mode */
  SET_BITS(USARTx->CTL0, value);
}

void HAL_USART_configHardwareFlow(USART_TypeDef *USARTx, uint32_t value) {
  CLEAR_BITS(USARTx->CTL2, USART_CTL2_RTSEN_MSK | USART_CTL2_CTSEN_MSK);
  SET_BITS(USARTx->CTL2, value);
}

void HAL_USART_configMode(USART_TypeDef *USARTx, uint32_t value) {
  CLEAR_BITS(USARTx->CTL0, USART_CTL0_REN_MSK | USART_CTL0_TEN_MSK);
  SET_BITS(USARTx->CTL0, value);
}

void HAL_USART_enable(USART_TypeDef *USARTx) {
  SET_BITS(USARTx->CTL0, USART_CTL0_UEN_MSK);
}

void HAL_USART_init(USART_TypeDef *USARTx, USART_InitTypeDef *USART_init) {
  HAL_USART_configBaudrate(USARTx, USART_init->baudrate);
  HAL_USART_configWordLength(USARTx, USART_init->word_length);
  HAL_USART_configStopBit(USARTx, USART_init->stop_bits);
  HAL_USART_configParity(USARTx, USART_init->parity);
  HAL_USART_configHardwareFlow(USARTx, USART_init->hardware_flow);
  HAL_USART_configMode(USARTx, USART_init->mode);
}

void HAL_USART_setup(USART_TypeDef *USARTx, uint32_t baudrate, uint32_t word_length, uint32_t stop_bits, uint32_t parity, uint32_t hardware_flow, uint32_t mode) {
  USART_InitTypeDef usart_init_struct;
  usart_init_struct.baudrate = baudrate;
  usart_init_struct.word_length = word_length;
  usart_init_struct.stop_bits = stop_bits;
  usart_init_struct.parity = parity;
  usart_init_struct.hardware_flow = hardware_flow;
  usart_init_struct.mode = mode;
  HAL_USART_init(USARTx, &usart_init_struct);
}



HAL_StatusTypeDef HAL_USART_waitForFlag(USART_TypeDef *USARTx, uint32_t flag, uint8_t state, uint32_t timestart, uint32_t timeout) {

  while (LL_USART_getFlag(USARTx, flag) != state) {
    if (timeout == 0UL) {
      continue;
    }
    if ((HAL_getTimeW() - timestart) > timeout) {
      
//       /* Disable TXE, RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts for the interrupt process */
//       CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR1_TXEIE));
//       CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

//       huart->gState  = HAL_UART_STATE_READY;
//       huart->RxState = HAL_UART_STATE_READY;
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}

HAL_StatusTypeDef HAL_USART_transmit(USART_TypeDef *USARTx, uint8_t *buffer, uint16_t size, uint32_t timeout) {
  uint32_t timestart = HAL_getTimeW();

  while (size > 0) {
    USARTx->DATA = (USART_DATA_DATA_MSK & *buffer);
    if (HAL_USART_waitForFlag(USARTx, USART_FLAG_TBE, SET, timestart, timeout) != HAL_OK) {
      return HAL_TIMEOUT;
    }
    buffer += sizeof(uint8_t);
    size -= 1;
  }
  return HAL_OK;
}

HAL_StatusTypeDef HAL_USART_receive(USART_TypeDef *USARTx, uint8_t *buffer, uint16_t size, uint64_t timeout) {
  uint64_t timestart = HAL_getTime();
  return HAL_OK;
}

// retarget the C library printf function to USART0
int _put_char(int ch) {
  HAL_USART_transmit(USART0, (uint8_t *)(&ch), sizeof(uint8_t), 0);
  return ch;
}


