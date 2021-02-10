/* include guard */
#ifndef __GD32VF1XX_HAL_USART_H
#define __GD32VF1XX_HAL_USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gd32vf1xx_hal_def.h"
#include "gd32vf1xx_hal_time.h"

typedef struct {
  uint32_t baudrate;
  uint32_t word_length;
  uint32_t stop_bits;
  uint32_t parity;
  uint32_t mode;
  uint32_t hardware_flow;
  uint32_t over_sampling;
} USART_InitTypeDef;



#define USART_RECEIVE_DISABLE         (0b0UL << USART_CTL0_REN_POS)                       /*!< disable receiver */
#define USART_RECEIVE_ENABLE          (0b1UL << USART_CTL0_REN_POS)                       /*!< enable receiver */

#define USART_TRANSMIT_DISABLE        (0b0UL << USART_CTL0_TEN_POS)                       /*!< disable transmitter */
#define USART_TRANSMIT_ENABLE         (0b1UL << USART_CTL0_TEN_POS)                       /*!< enable transmitter */

/* USART parity bits definitions */
#define USART_PM_NONE                 (0b00UL << USART_CTL0_PM_POS)                        /*!< no parity */
#define USART_PM_EVEN                 (0b10UL << USART_CTL0_PM_POS)                        /*!< even parity */
#define USART_PM_ODD                  (0b11UL << USART_CTL0_PM_POS)                        /*!< odd parity */

/* USART wakeup method in mute mode */
#define USART_WM_IDLE                 (0b0UL << USART_CTL0_WM_POS)                        /*!< idle line */
#define USART_WM_ADDR                 (0b1UL << USART_CTL0_WM_POS)                        /*!< address match */

/* USART word length definitions */
#define USART_WL_8BIT                 (0b0UL << USART_CTL0_WL_POS)                        /*!< 8 bits */
#define USART_WL_9BIT                 (0b1UL << USART_CTL0_WL_POS)                        /*!< 9 bits */

/* USART stop bits definitions */
#define USART_STB_1BIT                (0b00UL << USART_CTL1_STB_POS)                       /*!< 1 bit */
#define USART_STB_0_5BIT              (0b01UL << USART_CTL1_STB_POS)                       /*!< 0.5 bit */
#define USART_STB_2BIT                (0b10UL << USART_CTL1_STB_POS)                       /*!< 2 bits */
#define USART_STB_1_5BIT              (0b11UL << USART_CTL1_STB_POS)                       /*!< 1.5 bits */

/* USART LIN break frame length */
#define USART_LBLEN_10B               (0b0UL << USART_CTL1_LBLEN_POS)                     /*!< 10 bits */
#define USART_LBLEN_11B               (0b1UL << USART_CTL1_LBLEN_POS)                     /*!< 11 bits */

/* USART CK length */
#define USART_CLEN_NONE               (0b0UL << USART_CTL1_CLEN_POS)                      /*!< there are 7 CK pulses for an 8 bit frame and 8 CK pulses for a 9 bit frame */
#define USART_CLEN_EN                 (0b0UL << USART_CTL1_CLEN_POS)                      /*!< there are 8 CK pulses for an 8 bit frame and 9 CK pulses for a 9 bit frame */

/* USART clock phase */
#define USART_CPH_1CK                 (0b0UL << USART_CTL1_CPH_POS)                       /*!< first clock transition is the first data capture edge */
#define USART_CPH_2CK                 (0b0UL << USART_CTL1_CPH_POS)                       /*!< second clock transition is the first data capture edge */

/* USART clock polarity */
#define USART_CPL_LOW                 (0b0UL << USART_CTL1_CPL_POS)                       /*!< steady low value on CK pin */
#define USART_CPL_HIGH                (0b1UL << USART_CTL1_CPL_POS)                       /*!< steady high value on CK pin */

/* USART DMA request for receive configure */
#define USART_DENR_DISABLE            (0b0UL << USART_CTL2_DENR_POS)                      /*!< DMA request disable for reception */
#define USART_DENR_ENABLE             (0b1UL << USART_CTL2_DENR_POS)                      /*!< DMA request enable for reception */

/* USART DMA request for transmission configure */
#define USART_DENT_DISABLE            (0b0UL << USART_CTL2_DENT_POS)                      /*!< DMA request disable for transmission */
#define USART_DENT_ENABLE             (0b1UL << USART_CTL2_DENT_POS)                      /*!< DMA request enable for transmission */

/* USART RTS configure */
#define USART_RTS_DISABLE             (0b0UL << USART_CTL2_RTSEN_POS)                     /*!< RTS disable */
#define USART_RTS_ENABLE              (0b1UL << USART_CTL2_RTSEN_POS)

/* USART CTS configure */
#define USART_CTS_DISABLE             (0b0UL << USART_CTL2_CTSEN_POS)
#define USART_CTS_ENABLE              (0b1UL << USART_CTL2_CTSEN_POS)

/* USART IrDA low-power enable */
#define USART_IRLP_NORMAL             (0b0UL << USART_CTL2_IRLP_POS)                      /*!< normal */
#define USART_IRLP_LOW                (0b1UL << USART_CTL2_IRLP_POS)                      /*!< low-power */

#define USART_FLAG_PERR                       USART_STAT_PERR_MSK
#define USART_FLAG_FERR                       USART_STAT_FERR_MSK
#define USART_FLAG_NERR                       USART_STAT_NERR_MSK
#define USART_FLAG_ORERR                      USART_STAT_ORERR_MSK
#define USART_FLAG_IDLEF                      USART_STAT_IDLEF_MSK
#define USART_FLAG_RBNE                       USART_STAT_RBNE_MSK
#define USART_FLAG_TC                         USART_STAT_TC_MSK
#define USART_FLAG_TBE                        USART_STAT_TBE_MSK
#define USART_FLAG_LBDF                       USART_STAT_LBDF_MSK
#define USART_FLAG_CTSF                       USART_STAT_CTSF_MSK


#define LL_USART_getFlag(USARTx, flag)            (READ_BITS(USARTx->STAT, flag & FLAG_MASK) ? SET : RESET)
#define LL_USART_clearFlag(USARTx, flag)          CLEAR_BITS(USARTx->STAT, flag & FLAG_MASK)


void HAL_USART_enablePrintFloat(void);

void HAL_USART_configBaudrate(USART_TypeDef *USARTx, uint32_t value);

void HAL_USART_configWordLength(USART_TypeDef *USARTx, uint32_t value);

void HAL_USART_configStopBit(USART_TypeDef *USARTx, uint32_t value);

void HAL_USART_configParity(USART_TypeDef *USARTx, uint32_t value);

void HAL_USART_configHardwareFlow(USART_TypeDef *USARTx, uint32_t value);

void HAL_USART_configMode(USART_TypeDef *USARTx, uint32_t value);

void HAL_USART_enable(USART_TypeDef *USARTx);

void HAL_USART_setup(USART_TypeDef *USARTx, uint32_t baudrate, uint32_t word_length, uint32_t stop_bits, uint32_t parity, uint32_t hardware_flow, uint32_t mode);

void HAL_USART_init(USART_TypeDef *USARTx, USART_InitTypeDef *USART_init);

HAL_StatusTypeDef HAL_USART_waitForFlag(USART_TypeDef *USARTx, uint32_t flag, uint8_t status, uint32_t timestart, uint32_t timeout);

HAL_StatusTypeDef HAL_USART_transmit(USART_TypeDef *USARTx, uint8_t *buffer, uint16_t size, uint32_t timeout);


extern int _put_char(int ch) __attribute__((weak));


#ifdef __cplusplus
}
#endif

#endif  // __GD32VF1XX_HAL_USART_H
