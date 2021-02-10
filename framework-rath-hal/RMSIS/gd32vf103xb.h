/**
 * @file gd32vf103xb.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2021-01-11
 *
 * @copyright Copyright (c) 2021
 *
 */

/* include guard */
#ifndef __GD32VF103XB_H
#define __GD32VF103XB_H

#ifdef __cplusplus
extern "C" {
#endif

#define PLATFORM_GD32VF103

#include <stdint.h>

#include "core_rv5.h"
#include "system_gd32vf1xx.h"


/* define startup timeout value of high speed crystal oscillator (HXTAL) */
#ifndef HXTAL_STARTUP_TIMEOUT
#define HXTAL_STARTUP_TIMEOUT             0xFFFFUL
#endif /* high speed crystal oscillator startup timeout */

/* define value of internal 8MHz RC oscillator (IRC8M) in Hz */
#ifndef IRC8M_VALUE
#define IRC8M_VALUE                       8000000UL
#endif /* internal 8MHz RC oscillator value */

/* define startup timeout value of internal 8MHz RC oscillator (IRC8M) */
#ifndef IRC8M_STARTUP_TIMEOUT
#define IRC8M_STARTUP_TIMEOUT             0x0500UL
#endif /* internal 8MHz RC oscillator startup timeout */

/* define value of internal 40KHz RC oscillator(IRC40K) in Hz */
#ifndef IRC40K_VALUE
#define IRC40K_VALUE                      40000UL
#endif /* internal 40KHz RC oscillator value */

/* define value of low speed crystal oscillator (LXTAL)in Hz */
#ifndef LXTAL_VALUE
#define LXTAL_VALUE                       32768UL
#endif /* low speed crystal oscillator value */


/* define interrupt number */
typedef enum {
  CLIC_INT_RESERVED = 0, /*!< RISC-V reserved		*/
  CLIC_INT_SFT = 3,      /*!< Software interrupt		*/
  CLIC_INT_TMR = 7,      /*!< CPU Timer interrupt	*/
  CLIC_INT_BWEI = 17,    /*!< Bus Error interrupt	*/
  CLIC_INT_PMOVI = 18,   /*!< Performance Monitor	*/

  /* interruput numbers */
  WWDGT_IRQn = 19,          /*!< window watchDog timer interrupt                          */
  LVD_IRQn = 20,            /*!< LVD through EXTI line detect interrupt                   */
  TAMPER_IRQn = 21,         /*!< tamper through EXTI line detect                          */
  RTC_IRQn = 22,            /*!< RTC alarm interrupt                                      */
  FMC_IRQn = 23,            /*!< FMC interrupt                                            */
  RCU_CTC_IRQn = 24,        /*!< RCU and CTC interrupt                                    */
  EXTI0_IRQn = 25,          /*!< EXTI line 0 interrupts                                   */
  EXTI1_IRQn = 26,          /*!< EXTI line 1 interrupts                                   */
  EXTI2_IRQn = 27,          /*!< EXTI line 2 interrupts                                   */
  EXTI3_IRQn = 28,          /*!< EXTI line 3 interrupts                                   */
  EXTI4_IRQn = 29,          /*!< EXTI line 4 interrupts                                   */
  DMA0_Channel0_IRQn = 30,  /*!< DMA0 channel0 interrupt                                  */
  DMA0_Channel1_IRQn = 31,  /*!< DMA0 channel1 interrupt                                  */
  DMA0_Channel2_IRQn = 32,  /*!< DMA0 channel2 interrupt                                  */
  DMA0_Channel3_IRQn = 33,  /*!< DMA0 channel3 interrupt                                  */
  DMA0_Channel4_IRQn = 34,  /*!< DMA0 channel4 interrupt                                  */
  DMA0_Channel5_IRQn = 35,  /*!< DMA0 channel5 interrupt                                  */
  DMA0_Channel6_IRQn = 36,  /*!< DMA0 channel6 interrupt                                  */
  ADC0_1_IRQn = 37,         /*!< ADC0 and ADC1 interrupt                                  */
  CAN0_TX_IRQn = 38,        /*!< CAN0 TX interrupts                                       */
  CAN0_RX0_IRQn = 39,       /*!< CAN0 RX0 interrupts                                      */
  CAN0_RX1_IRQn = 40,       /*!< CAN0 RX1 interrupts                                      */
  CAN0_EWMC_IRQn = 41,      /*!< CAN0 EWMC interrupts                                     */
  EXTI5_9_IRQn = 42,        /*!< EXTI[9:5] interrupts                                     */
  TIMER0_BRK_IRQn = 43,     /*!< TIMER0 break interrupts                                  */
  TIMER0_UP_IRQn = 44,      /*!< TIMER0 update interrupts                                 */
  TIMER0_TRG_CMT_IRQn = 45, /*!< TIMER0 trigger and commutation interrupts                */
  TIMER0_Channel_IRQn = 46, /*!< TIMER0 channel capture compare interrupts                */
  TIMER1_IRQn = 47,         /*!< TIMER1 interrupt                                         */
  TIMER2_IRQn = 48,         /*!< TIMER2 interrupt                                         */
  TIMER3_IRQn = 49,         /*!< TIMER3 interrupts                                        */
  I2C0_EV_IRQn = 50,        /*!< I2C0 event interrupt                                     */
  I2C0_ER_IRQn = 51,        /*!< I2C0 error interrupt                                     */
  I2C1_EV_IRQn = 52,        /*!< I2C1 event interrupt                                     */
  I2C1_ER_IRQn = 53,        /*!< I2C1 error interrupt                                     */
  SPI0_IRQn = 54,           /*!< SPI0 interrupt                                           */
  SPI1_IRQn = 55,           /*!< SPI1 interrupt                                           */
  USART0_IRQn = 56,         /*!< USART0 interrupt                                         */
  USART1_IRQn = 57,         /*!< USART1 interrupt                                         */
  USART2_IRQn = 58,         /*!< USART2 interrupt                                         */
  EXTI10_15_IRQn = 59,      /*!< EXTI[15:10] interrupts                                   */
  RTC_ALARM_IRQn = 60,      /*!< RTC alarm interrupt EXTI                                 */
  USBFS_WKUP_IRQn = 61,     /*!< USBFS wakeup interrupt                                   */

  EXMC_IRQn = 67, /*!< EXMC global interrupt                                    */

  TIMER4_IRQn = 69,        /*!< TIMER4 global interrupt                                  */
  SPI2_IRQn = 70,          /*!< SPI2 global interrupt                                    */
  UART3_IRQn = 71,         /*!< UART3 global interrupt                                   */
  UART4_IRQn = 72,         /*!< UART4 global interrupt                                   */
  TIMER5_IRQn = 73,        /*!< TIMER5 global interrupt                                  */
  TIMER6_IRQn = 74,        /*!< TIMER6 global interrupt                                  */
  DMA1_Channel0_IRQn = 75, /*!< DMA1 channel0 global interrupt                           */
  DMA1_Channel1_IRQn = 76, /*!< DMA1 channel1 global interrupt                           */
  DMA1_Channel2_IRQn = 77, /*!< DMA1 channel2 global interrupt                           */
  DMA1_Channel3_IRQn = 78, /*!< DMA1 channel3 global interrupt                           */
  DMA1_Channel4_IRQn = 79, /*!< DMA1 channel3 global interrupt                           */

  CAN1_TX_IRQn = 82,   /*!< CAN1 TX interrupt                                        */
  CAN1_RX0_IRQn = 83,  /*!< CAN1 RX0 interrupt                                       */
  CAN1_RX1_IRQn = 84,  /*!< CAN1 RX1 interrupt                                       */
  CAN1_EWMC_IRQn = 85, /*!< CAN1 EWMC interrupt                                      */
  USBFS_IRQn = 86,     /*!< USBFS global interrupt                                   */

  ECLIC_NUM_INTERRUPTS
} IRQn_Type;


typedef enum {
  DISABLE = 0U,
  ENABLE = !DISABLE
} FunctionalState;

typedef enum {
  SUCCESS = 0U, 
  ERROR = !SUCCESS
} ErrorStatus;

// #define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))



typedef struct {
  __IO uint32_t STAT;   // 0x0
  __IO uint32_t CTL0;   // 0x4
  __IO uint32_t CTL1;   // 0x8
  __IO uint32_t SAMPT0; // 0xC
  __IO uint32_t SAMPT1; // 0x10
  __IO uint32_t IOFF0;  // 0x14
  __IO uint32_t IOFF1;  // 0x18
  __IO uint32_t IOFF2;  // 0x1C
  __IO uint32_t IOFF3;  // 0x20
  __IO uint32_t WDHT;   // 0x24
  __IO uint32_t WDLT;   // 0x28
  __IO uint32_t RSQ0;   // 0x2C
  __IO uint32_t RSQ1;   // 0x30
  __IO uint32_t RSQ2;   // 0x34
  __IO uint32_t ISQ;    // 0x38
  __IO uint32_t IDATA0; // 0x3C
  __IO uint32_t IDATA1; // 0x40
  __IO uint32_t IDATA2; // 0x44
  __IO uint32_t IDATA3; // 0x48
  __IO uint32_t RDATA;  // 0x4C
  uint32_t RESERVED0[12];
  __IO uint32_t OVSAMPCTL; // 0x80
} ADC_TypeDef;


typedef struct {
  __IO uint32_t TMI;
  __IO uint32_t TMP;
  __IO uint32_t TMDATA0;
  __IO uint32_t TMDATA1;
} CAN_TxMailBox_TypeDef;

typedef struct {
  __IO uint32_t RFIFOMI;
  __IO uint32_t RFIFOMP;
  __IO uint32_t RFIFOMDATA0;
  __IO uint32_t RFIFOMDATA1;
} CAN_FIFOMailBox_TypeDef;


typedef struct {
  __IO uint32_t FDATA0;
  __IO uint32_t FDATA1;
} CAN_FilterRegister_TypeDef;

typedef struct {
  __IO uint32_t CTL;              // 0x0
  __IO uint32_t STAT;             // 0x4
  __IO uint32_t TSTAT;            // 0x8
  __IO uint32_t RFIFO0;           // 0xC
  __IO uint32_t RFIFO1;           // 0x10
  __IO uint32_t INTEN;            // 0x14
  __IO uint32_t ERR;              // 0x18
  __IO uint32_t BT;               // 0x1C
  uint32_t RESERVED0[88];
  CAN_TxMailBox_TypeDef TxMailBox[3]; //0x180 .. 0x1AC
  CAN_FIFOMailBox_TypeDef FIFOMailBox[2]; //0x1B0 .. 0x1CC
  uint32_t RESERVED1[12];
  __IO uint32_t FCTL;             // 0x200
  __IO uint32_t FMCFG;            // 0x204
  uint32_t RESERVED2[1];
  __IO uint32_t FSCFG;            // 0x20C
  uint32_t RESERVED3[1];
  __IO uint32_t FAFIFO;           // 0x214
  uint32_t RESERVED4[1];
  __IO uint32_t FW;               // 0x21C
  uint32_t RESERVED5[8];
  CAN_FilterRegister_TypeDef FilterRegister[27]; // 0x240 .. 0x31C
} CAN_TypeDef;

typedef struct {
  __IO uint32_t INTEN;      // 0x0
  __IO uint32_t EVEN;       // 0x4
  __IO uint32_t RTEN;       // 0x8
  __IO uint32_t FTEN;       // 0xC
  __IO uint32_t SWIEV;      // 0x10
  __IO uint32_t PD;         // 0x14
} EXTI_TypeDef;

typedef struct {
  __IO uint32_t CTL0;     // 0x0
  __IO uint32_t CTL1;     // 0x4
  __IO uint32_t ISTAT;    // 0x8
  __IO uint32_t OCTL;     // 0xC
  __IO uint32_t BOP;      // 0x10
  __IO uint32_t BC;       // 0x14
  __IO uint32_t LOCK;     // 0x18
} GPIO_TypeDef;

typedef struct {
  __IO uint32_t EC;       // 0x0
  __IO uint32_t PCF0;     // 0x4
  __IO uint32_t EXTISS0;  // 0x8
  __IO uint32_t EXTISS1;  // 0xC
  __IO uint32_t EXTISS2;  // 0x10
  __IO uint32_t EXTISS3;  // 0x14
  __IO uint32_t PCF1;     // 0x18
} AFIO_TypeDef;

typedef struct {
  __IO uint32_t CTL0;     // 0x00
  __IO uint32_t CTL1;     // 0x04
  __IO uint32_t SADDR0;   // 0x08
  __IO uint32_t SADDR1;   // 0x0C
  __IO uint32_t DATA;     // 0x10
  __IO uint32_t STAT0;    // 0x14
  __IO uint32_t STAT1;    // 0x18
  __IO uint32_t CKCFG;    // 0x1C
  __IO uint32_t RT;       // 0x20
  uint32_t RESERVED0[27];
  __IO uint32_t FMPCFG;   // 0x90
} I2C_TypeDef;

typedef struct {
  __IO uint32_t CTL;      // 0x00
  __IO uint32_t CFG0;     // 0x04
  __IO uint32_t INT;      // 0x08
  __IO uint32_t APB2RST;  // 0x0C
  __IO uint32_t APB1RST;  // 0x10
  __IO uint32_t AHBEN;    // 0x14
  __IO uint32_t APB2EN;   // 0x18
  __IO uint32_t APB1EN;   // 0x1C
  __IO uint32_t BDCTL;    // 0x20
  __IO uint32_t RSTSCK;   // 0x24
  __IO uint32_t AHBRST;   // 0x28
  __IO uint32_t CFG1;     // 0x2C
  uint32_t RESERVED0[1];
  __IO uint32_t DSV; // 0x34
} RCU_TypeDef;

typedef struct {
  __IO uint32_t CTL0;      // 0x00
  __IO uint32_t CTL1;     // 0x04
  __IO uint32_t STAT;      // 0x08
  __IO uint32_t DATA;  // 0x0C
  __IO uint32_t CRCPOLY;  // 0x10
  __IO uint32_t RCRC;    // 0x14
  __IO uint32_t TCRC;   // 0x18
  __IO uint32_t I2SCTL;   // 0x1C
  __IO uint32_t I2SPSC;    // 0x20
} SPI_TypeDef;

typedef struct {
  __IO uint32_t CTL0;     // 0x00
  __IO uint32_t CTL1;     // 0x04
  __IO uint32_t SMCFG;    // 0x08
  __IO uint32_t DMAINTEN; // 0x0C
  __IO uint32_t INTF;     // 0x10
  __IO uint32_t SWEVG;    // 0x14
  __IO uint32_t CHCTL0;   // 0x18
  __IO uint32_t CHCTL1;   // 0x1C
  __IO uint32_t CHCTL2;   // 0x20
  __IO uint32_t CNT;      // 0x24
  __IO uint32_t PSC;      // 0x28
  __IO uint32_t CAR;      // 0x2C
  __IO uint32_t CREP;     // 0x30
  __IO uint32_t CH0CV;    // 0x34
  __IO uint32_t CH1CV;    // 0x38
  __IO uint32_t CH2CV;    // 0x3C
  __IO uint32_t CH3CV;    // 0x40
  __IO uint32_t CCHP;     // 0x44
  __IO uint32_t DMACFG;   // 0x48
  __IO uint32_t DMATB;    // 0x4C
} TIMER_TypeDef;

typedef struct {
  __IO uint32_t STAT; // 0x00
  __IO uint32_t DATA; // 0x04
  __IO uint32_t BAUD; // 0x08
  __IO uint32_t CTL0; // 0x0C
  __IO uint32_t CTL1; // 0x10
  __IO uint32_t CTL2; // 0x14
  __IO uint32_t GP;   // 0x18
} USART_TypeDef;

/* main flash and SRAM memory map */
#define FLASH_BASE                    0x08000000UL /*!< main FLASH base address          */
#define SRAM_BASE                     0x20000000UL  /*!< SRAM0 base address               */
#define OB_BASE                       0x1FFFF800UL    /*!< OB base address                  */
#define DBG_BASE                      0xE0042000UL   /*!< DBG base address                 */
#define EXMC_BASE                     0xA0000000UL  /*!< EXMC register base address       */

#define PERIPH_BASE                   0x40000000UL

/* peripheral memory map */
#define APB1PERIPH_BASE               PERIPH_BASE                   /*!< apb1 base address                */
#define APB2PERIPH_BASE               (PERIPH_BASE + 0x00010000UL)  /*!< apb2 base address                */
#define AHBPERIPH_BASE                (PERIPH_BASE + 0x00018000UL)   /*!< ahb1 base address                */
#define AHBEXTERNAL_BASE              (PERIPH_BASE + 0x20000000UL) /*!< ahb3 base address                */

/* advanced peripheral bus 1 memory map */
#define TIMER1_BASE (APB1PERIPH_BASE + 0x00000000UL) /*!< TIMER base address               */
#define TIMER2_BASE (APB1PERIPH_BASE + 0x00000400UL) /*!< TIMER base address               */
#define TIMER3_BASE (APB1PERIPH_BASE + 0x00000800UL) /*!< TIMER base address               */
#define TIMER4_BASE (APB1PERIPH_BASE + 0x00000C00UL) /*!< TIMER base address               */
#define TIMER5_BASE (APB1PERIPH_BASE + 0x00001000UL) /*!< TIMER base address               */
#define TIMER6_BASE (APB1PERIPH_BASE + 0x00001400UL) /*!< TIMER base address               */
#define RTC_BASE (APB1PERIPH_BASE + 0x00002800UL)    /*!< TIMER base address               */
#define WWDGT_BASE (APB1PERIPH_BASE + 0x00002C00UL)  /*!< TIMER base address               */
#define FWDGT_BASE (APB1PERIPH_BASE + 0x00003000UL)  /*!< TIMER base address               */
#define SPI1_BASE (APB1PERIPH_BASE + 0x00003800UL)   /*!< TIMER base address               */
#define SPI2_BASE (APB1PERIPH_BASE + 0x00003C00UL)   /*!< TIMER base address               */
#define USART1_BASE (APB1PERIPH_BASE + 0x00004400UL) /*!< TIMER base address               */
#define USART2_BASE (APB1PERIPH_BASE + 0x00004800UL) /*!< TIMER base address               */
#define UART3_BASE (APB1PERIPH_BASE + 0x00004C00UL)  /*!< TIMER base address               */
#define UART4_BASE (APB1PERIPH_BASE + 0x00005000UL)  /*!< TIMER base address               */
#define I2C0_BASE (APB1PERIPH_BASE + 0x00005400UL)   /*!< I2C base address                 */
#define I2C1_BASE (APB1PERIPH_BASE + 0x00005800UL)   /*!< I2C base address                 */

#define USBFS_REG_BASE (APB1PERIPH_BASE + 0x00005C00UL) /*!< I2C base address                 */
#define CANSRAM_BASE (APB1PERIPH_BASE + 0x00006000UL)   /*!< I2C base address                 */
#define CAN0_BASE (APB1PERIPH_BASE + 0x00006400UL)      /*!< I2C base address                 */
#define CAN1_BASE (APB1PERIPH_BASE + 0x00006800UL)      /*!< I2C base address                 */

#define BKP_BASE (APB1PERIPH_BASE + 0x00006C00UL) /*!< I2C base address                 */
#define PMU_BASE (APB1PERIPH_BASE + 0x00007000UL) /*!< I2C base address                 */
#define DAC_BASE (APB1PERIPH_BASE + 0x00007400UL) /*!< I2C base address                 */

/* advanced peripheral bus 2 memory map */
#define AFIO_BASE                     (APB2PERIPH_BASE + 0x00000000UL)   /*!< I2C base address                 */
#define EXTI_BASE                     (APB2PERIPH_BASE + 0x00000400UL)   /*!< I2C base address                 */
#define GPIOA_BASE                    (APB2PERIPH_BASE + 0x00000800UL)  /*!< I2C base address                 */
#define GPIOB_BASE                    (APB2PERIPH_BASE + 0x00000C00UL)  /*!< I2C base address                 */
#define GPIOC_BASE                    (APB2PERIPH_BASE + 0x00001000UL)  /*!< I2C base address                 */
#define GPIOD_BASE                    (APB2PERIPH_BASE + 0x00001400UL)  /*!< I2C base address                 */
#define GPIOE_BASE                    (APB2PERIPH_BASE + 0x00001800UL)  /*!< I2C base address                 */
#define ADC0_BASE                     (APB2PERIPH_BASE + 0x00002400UL)   /*!< I2C base address                 */
#define ADC1_BASE                     (APB2PERIPH_BASE + 0x00002800UL)   /*!< I2C base address                 */
#define TIMER0_BASE                   (APB2PERIPH_BASE + 0x00002C00UL) /*!< I2C base address                 */
#define SPI0_BASE                     (APB2PERIPH_BASE + 0x00003000UL)   /*!< I2C base address                 */
#define USART0_BASE                   (APB2PERIPH_BASE + 0x00003800UL) /*!< I2C base address                 */

/* advanced high performance bus 1 memory map */
#define DMA0_BASE                     (AHBPERIPH_BASE + 0x00008000UL)  /*!< DMA base address                 */
#define DMA1_BASE                     (AHBPERIPH_BASE + 0x00008400UL)  /*!< DMA base address                 */
#define RCU_BASE                      (AHBPERIPH_BASE + 0x00009000UL)   /*!< RCU base address                 */
#define FMC_BASE                      (AHBPERIPH_BASE + 0x0000A000UL)   /*!< FMC base address                 */
#define CRC_BASE                      (AHBPERIPH_BASE + 0x0000B000UL)   /*!< CRC base address                 */
#define USBFS_BASE                    (AHBPERIPH_BASE + 0x0FFE8000UL) /*!< USBFS base address               */



#define ADC0                          ((ADC_TypeDef *)ADC0_BASE)
#define ADC1                          ((ADC_TypeDef *)ADC1_BASE)

#define CAN0                          ((CAN_TypeDef *)CAN0_BASE)
#define CAN1                          ((CAN_TypeDef *)CAN1_BASE)

#define EXTI                          ((EXTI_TypeDef *)EXTI_BASE)

#define GPIOA                         ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB                         ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC                         ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD                         ((GPIO_TypeDef *)GPIOD_BASE)
#define GPIOE                         ((GPIO_TypeDef *)GPIOE_BASE)
#define AFIO                          ((AFIO_TypeDef *)AFIO_BASE)

#define I2C0                          ((I2C_TypeDef *)I2C0_BASE)
#define I2C1                          ((I2C_TypeDef *)I2C1_BASE)

#define RCU                           ((RCU_TypeDef *)RCU_BASE)

#define SPI0                          ((SPI_TypeDef *)SPI0_BASE)
#define SPI1                          ((SPI_TypeDef *)SPI1_BASE)
#define SPI2                          ((SPI_TypeDef *)SPI2_BASE)

#define TIMER0                        ((TIMER_TypeDef *)TIMER0_BASE)
#define TIMER1                        ((TIMER_TypeDef *)TIMER1_BASE)
#define TIMER2                        ((TIMER_TypeDef *)TIMER2_BASE)
#define TIMER3                        ((TIMER_TypeDef *)TIMER3_BASE)
#define TIMER4                        ((TIMER_TypeDef *)TIMER4_BASE)
#define TIMER5                        ((TIMER_TypeDef *)TIMER5_BASE)
#define TIMER6                        ((TIMER_TypeDef *)TIMER6_BASE)

#define USART0                        ((USART_TypeDef *)USART0_BASE) /*!< USART0 base address */
#define USART1                        ((USART_TypeDef *)USART1_BASE) /*!< USART1 base address */
#define USART2                        ((USART_TypeDef *)USART2_BASE) /*!< USART2 base address */
#define UART3                         ((USART_TypeDef *)UART3_BASE)   /*!< UART3 base address */
#define UART4                         ((USART_TypeDef *)UART4_BASE)   /*!< UART4 base address */

























#define ADC_STAT_STRC_POS								0x04UL		/** Start flag of regular channel group */
#define ADC_STAT_STRC_MSK								(0x01UL << ADC_STAT_STRC_POS)		/** Start flag of regular channel group */
#define ADC_STAT_STIC_POS								0x03UL		/** Start flag of inserted channel group */
#define ADC_STAT_STIC_MSK								(0x01UL << ADC_STAT_STIC_POS)		/** Start flag of inserted channel group */
#define ADC_STAT_EOIC_POS								0x02UL		/** End of inserted group conversion flag */
#define ADC_STAT_EOIC_MSK								(0x01UL << ADC_STAT_EOIC_POS)		/** End of inserted group conversion flag */
#define ADC_STAT_EOC_POS								0x01UL		/** End of group conversion flag */
#define ADC_STAT_EOC_MSK								(0x01UL << ADC_STAT_EOC_POS)		/** End of group conversion flag */
#define ADC_STAT_WDE_POS								0x00UL		/** Analog watchdog event flag */
#define ADC_STAT_WDE_MSK								(0x01UL << ADC_STAT_WDE_POS)		/** Analog watchdog event flag */
#define ADC_CTL0_RWDEN_POS								0x17UL		/** Regular channel analog watchdog enable */
#define ADC_CTL0_RWDEN_MSK								(0x01UL << ADC_CTL0_RWDEN_POS)		/** Regular channel analog watchdog enable */
#define ADC_CTL0_IWDEN_POS								0x16UL		/** Inserted channel analog watchdog enable */
#define ADC_CTL0_IWDEN_MSK								(0x01UL << ADC_CTL0_IWDEN_POS)		/** Inserted channel analog watchdog enable */
#define ADC_CTL0_SYNCM_POS								0x10UL		/** sync mode selection */
#define ADC_CTL0_SYNCM_MSK								(0x0FUL << ADC_CTL0_SYNCM_POS)		/** sync mode selection */
#define ADC_CTL0_DISNUM_POS								0x0DUL		/** Number of conversions in discontinuous mode */
#define ADC_CTL0_DISNUM_MSK								(0x07UL << ADC_CTL0_DISNUM_POS)		/** Number of conversions in discontinuous mode */
#define ADC_CTL0_DISIC_POS								0x0CUL		/** Discontinuous mode on inserted channels */
#define ADC_CTL0_DISIC_MSK								(0x01UL << ADC_CTL0_DISIC_POS)		/** Discontinuous mode on inserted channels */
#define ADC_CTL0_DISRC_POS								0x0BUL		/** Discontinuous mode on regular channels */
#define ADC_CTL0_DISRC_MSK								(0x01UL << ADC_CTL0_DISRC_POS)		/** Discontinuous mode on regular channels */
#define ADC_CTL0_ICA_POS								0x0AUL		/** Inserted channel group convert automatically */
#define ADC_CTL0_ICA_MSK								(0x01UL << ADC_CTL0_ICA_POS)		/** Inserted channel group convert automatically */
#define ADC_CTL0_WDSC_POS								0x09UL		/** When in scan mode, analog watchdog is effective on a single channel */
#define ADC_CTL0_WDSC_MSK								(0x01UL << ADC_CTL0_WDSC_POS)		/** When in scan mode, analog watchdog is effective on a single channel */
#define ADC_CTL0_SM_POS								0x08UL		/** Scan mode */
#define ADC_CTL0_SM_MSK								(0x01UL << ADC_CTL0_SM_POS)		/** Scan mode */
#define ADC_CTL0_EOICIE_POS								0x07UL		/** Interrupt enable for EOIC */
#define ADC_CTL0_EOICIE_MSK								(0x01UL << ADC_CTL0_EOICIE_POS)		/** Interrupt enable for EOIC */
#define ADC_CTL0_WDEIE_POS								0x06UL		/** Interrupt enable for WDE */
#define ADC_CTL0_WDEIE_MSK								(0x01UL << ADC_CTL0_WDEIE_POS)		/** Interrupt enable for WDE */
#define ADC_CTL0_EOCIE_POS								0x05UL		/** Interrupt enable for EOC */
#define ADC_CTL0_EOCIE_MSK								(0x01UL << ADC_CTL0_EOCIE_POS)		/** Interrupt enable for EOC */
#define ADC_CTL0_WDCHSEL_POS								0x00UL		/** Analog watchdog channel select */
#define ADC_CTL0_WDCHSEL_MSK								(0x1FUL << ADC_CTL0_WDCHSEL_POS)		/** Analog watchdog channel select */
#define ADC_CTL1_TSVREN_POS								0x17UL		/** Channel 16 and 17 enable of ADC0 */
#define ADC_CTL1_TSVREN_MSK								(0x01UL << ADC_CTL1_TSVREN_POS)		/** Channel 16 and 17 enable of ADC0 */
#define ADC_CTL1_SWRCST_POS								0x16UL		/** Start on regular channel */
#define ADC_CTL1_SWRCST_MSK								(0x01UL << ADC_CTL1_SWRCST_POS)		/** Start on regular channel */
#define ADC_CTL1_SWICST_POS								0x15UL		/** Start on inserted channel */
#define ADC_CTL1_SWICST_MSK								(0x01UL << ADC_CTL1_SWICST_POS)		/** Start on inserted channel */
#define ADC_CTL1_ETERC_POS								0x14UL		/** External trigger enable for regular channel */
#define ADC_CTL1_ETERC_MSK								(0x01UL << ADC_CTL1_ETERC_POS)		/** External trigger enable for regular channel */
#define ADC_CTL1_ETSRC_POS								0x11UL		/** External trigger select for regular channel */
#define ADC_CTL1_ETSRC_MSK								(0x07UL << ADC_CTL1_ETSRC_POS)		/** External trigger select for regular channel */
#define ADC_CTL1_ETEIC_POS								0x0FUL		/** External trigger select for inserted channel */
#define ADC_CTL1_ETEIC_MSK								(0x01UL << ADC_CTL1_ETEIC_POS)		/** External trigger select for inserted channel */
#define ADC_CTL1_ETSIC_POS								0x0CUL		/** External trigger select for inserted channel */
#define ADC_CTL1_ETSIC_MSK								(0x07UL << ADC_CTL1_ETSIC_POS)		/** External trigger select for inserted channel */
#define ADC_CTL1_DAL_POS								0x0BUL		/** Data alignment */
#define ADC_CTL1_DAL_MSK								(0x01UL << ADC_CTL1_DAL_POS)		/** Data alignment */
#define ADC_CTL1_DMA_POS								0x08UL		/** DMA request enable */
#define ADC_CTL1_DMA_MSK								(0x01UL << ADC_CTL1_DMA_POS)		/** DMA request enable */
#define ADC_CTL1_RSTCLB_POS								0x03UL		/** Reset calibration */
#define ADC_CTL1_RSTCLB_MSK								(0x01UL << ADC_CTL1_RSTCLB_POS)		/** Reset calibration */
#define ADC_CTL1_CLB_POS								0x02UL		/** ADC calibration */
#define ADC_CTL1_CLB_MSK								(0x01UL << ADC_CTL1_CLB_POS)		/** ADC calibration */
#define ADC_CTL1_CTN_POS								0x01UL		/** Continuous mode */
#define ADC_CTL1_CTN_MSK								(0x01UL << ADC_CTL1_CTN_POS)		/** Continuous mode */
#define ADC_CTL1_ADCON_POS								0x00UL		/** ADC on */
#define ADC_CTL1_ADCON_MSK								(0x01UL << ADC_CTL1_ADCON_POS)		/** ADC on */
#define ADC_SAMPT0_SPT10_POS								0x00UL		/** Channel 10 sample time selection */
#define ADC_SAMPT0_SPT10_MSK								(0x07UL << ADC_SAMPT0_SPT10_POS)		/** Channel 10 sample time selection */
#define ADC_SAMPT0_SPT11_POS								0x03UL		/** Channel 11 sample time selection */
#define ADC_SAMPT0_SPT11_MSK								(0x07UL << ADC_SAMPT0_SPT11_POS)		/** Channel 11 sample time selection */
#define ADC_SAMPT0_SPT12_POS								0x06UL		/** Channel 12 sample time selection */
#define ADC_SAMPT0_SPT12_MSK								(0x07UL << ADC_SAMPT0_SPT12_POS)		/** Channel 12 sample time selection */
#define ADC_SAMPT0_SPT13_POS								0x09UL		/** Channel 13 sample time selection */
#define ADC_SAMPT0_SPT13_MSK								(0x07UL << ADC_SAMPT0_SPT13_POS)		/** Channel 13 sample time selection */
#define ADC_SAMPT0_SPT14_POS								0x0CUL		/** Channel 14 sample time selection */
#define ADC_SAMPT0_SPT14_MSK								(0x07UL << ADC_SAMPT0_SPT14_POS)		/** Channel 14 sample time selection */
#define ADC_SAMPT0_SPT15_POS								0x0FUL		/** Channel 15 sample time selection */
#define ADC_SAMPT0_SPT15_MSK								(0x07UL << ADC_SAMPT0_SPT15_POS)		/** Channel 15 sample time selection */
#define ADC_SAMPT0_SPT16_POS								0x12UL		/** Channel 16 sample time selection */
#define ADC_SAMPT0_SPT16_MSK								(0x07UL << ADC_SAMPT0_SPT16_POS)		/** Channel 16 sample time selection */
#define ADC_SAMPT0_SPT17_POS								0x15UL		/** Channel 17 sample time selection */
#define ADC_SAMPT0_SPT17_MSK								(0x07UL << ADC_SAMPT0_SPT17_POS)		/** Channel 17 sample time selection */
#define ADC_SAMPT1_SPT0_POS								0x00UL		/** Channel 0 sample time selection */
#define ADC_SAMPT1_SPT0_MSK								(0x07UL << ADC_SAMPT1_SPT0_POS)		/** Channel 0 sample time selection */
#define ADC_SAMPT1_SPT1_POS								0x03UL		/** Channel 1 sample time selection */
#define ADC_SAMPT1_SPT1_MSK								(0x07UL << ADC_SAMPT1_SPT1_POS)		/** Channel 1 sample time selection */
#define ADC_SAMPT1_SPT2_POS								0x06UL		/** Channel 2 sample time selection */
#define ADC_SAMPT1_SPT2_MSK								(0x07UL << ADC_SAMPT1_SPT2_POS)		/** Channel 2 sample time selection */
#define ADC_SAMPT1_SPT3_POS								0x09UL		/** Channel 3 sample time selection */
#define ADC_SAMPT1_SPT3_MSK								(0x07UL << ADC_SAMPT1_SPT3_POS)		/** Channel 3 sample time selection */
#define ADC_SAMPT1_SPT4_POS								0x0CUL		/** Channel 4 sample time selection */
#define ADC_SAMPT1_SPT4_MSK								(0x07UL << ADC_SAMPT1_SPT4_POS)		/** Channel 4 sample time selection */
#define ADC_SAMPT1_SPT5_POS								0x0FUL		/** Channel 5 sample time selection */
#define ADC_SAMPT1_SPT5_MSK								(0x07UL << ADC_SAMPT1_SPT5_POS)		/** Channel 5 sample time selection */
#define ADC_SAMPT1_SPT6_POS								0x12UL		/** Channel 6 sample time selection */
#define ADC_SAMPT1_SPT6_MSK								(0x07UL << ADC_SAMPT1_SPT6_POS)		/** Channel 6 sample time selection */
#define ADC_SAMPT1_SPT7_POS								0x15UL		/** Channel 7 sample time selection */
#define ADC_SAMPT1_SPT7_MSK								(0x07UL << ADC_SAMPT1_SPT7_POS)		/** Channel 7 sample time selection */
#define ADC_SAMPT1_SPT8_POS								0x18UL		/** Channel 8 sample time selection */
#define ADC_SAMPT1_SPT8_MSK								(0x07UL << ADC_SAMPT1_SPT8_POS)		/** Channel 8 sample time selection */
#define ADC_SAMPT1_SPT9_POS								0x1BUL		/** Channel 9 sample time selection */
#define ADC_SAMPT1_SPT9_MSK								(0x07UL << ADC_SAMPT1_SPT9_POS)		/** Channel 9 sample time selection */
#define ADC_IOFF0_IOFF_POS								0x00UL		/** Data offset for inserted channel 0 */
#define ADC_IOFF0_IOFF_MSK								(0xFFFUL << ADC_IOFF0_IOFF_POS)		/** Data offset for inserted channel 0 */
#define ADC_IOFF1_IOFF_POS								0x00UL		/** Data offset for inserted channel 1 */
#define ADC_IOFF1_IOFF_MSK								(0xFFFUL << ADC_IOFF1_IOFF_POS)		/** Data offset for inserted channel 1 */
#define ADC_IOFF2_IOFF_POS								0x00UL		/** Data offset for inserted channel 2 */
#define ADC_IOFF2_IOFF_MSK								(0xFFFUL << ADC_IOFF2_IOFF_POS)		/** Data offset for inserted channel 2 */
#define ADC_IOFF3_IOFF_POS								0x00UL		/** Data offset for inserted channel 3 */
#define ADC_IOFF3_IOFF_MSK								(0xFFFUL << ADC_IOFF3_IOFF_POS)		/** Data offset for inserted channel 3 */
#define ADC_WDHT_WDHT_POS								0x00UL		/** Analog watchdog higher threshold */
#define ADC_WDHT_WDHT_MSK								(0xFFFUL << ADC_WDHT_WDHT_POS)		/** Analog watchdog higher threshold */
#define ADC_WDLT_WDLT_POS								0x00UL		/** Analog watchdog lower threshold */
#define ADC_WDLT_WDLT_MSK								(0xFFFUL << ADC_WDLT_WDLT_POS)		/** Analog watchdog lower threshold */
#define ADC_RSQ0_RL_POS								0x14UL		/** Regular channel group length */
#define ADC_RSQ0_RL_MSK								(0x0FUL << ADC_RSQ0_RL_POS)		/** Regular channel group length */
#define ADC_RSQ0_RSQ15_POS								0x0FUL		/** 16th conversion in regular sequence */
#define ADC_RSQ0_RSQ15_MSK								(0x1FUL << ADC_RSQ0_RSQ15_POS)		/** 16th conversion in regular sequence */
#define ADC_RSQ0_RSQ14_POS								0x0AUL		/** 15th conversion in regular sequence */
#define ADC_RSQ0_RSQ14_MSK								(0x1FUL << ADC_RSQ0_RSQ14_POS)		/** 15th conversion in regular sequence */
#define ADC_RSQ0_RSQ13_POS								0x05UL		/** 14th conversion in regular sequence */
#define ADC_RSQ0_RSQ13_MSK								(0x1FUL << ADC_RSQ0_RSQ13_POS)		/** 14th conversion in regular sequence */
#define ADC_RSQ0_RSQ12_POS								0x00UL		/** 13th conversion in regular sequence */
#define ADC_RSQ0_RSQ12_MSK								(0x1FUL << ADC_RSQ0_RSQ12_POS)		/** 13th conversion in regular sequence */
#define ADC_RSQ1_RSQ11_POS								0x19UL		/** 12th conversion in regular sequence */
#define ADC_RSQ1_RSQ11_MSK								(0x1FUL << ADC_RSQ1_RSQ11_POS)		/** 12th conversion in regular sequence */
#define ADC_RSQ1_RSQ10_POS								0x14UL		/** 11th conversion in regular sequence */
#define ADC_RSQ1_RSQ10_MSK								(0x1FUL << ADC_RSQ1_RSQ10_POS)		/** 11th conversion in regular sequence */
#define ADC_RSQ1_RSQ9_POS								0x0FUL		/** 10th conversion in regular sequence */
#define ADC_RSQ1_RSQ9_MSK								(0x1FUL << ADC_RSQ1_RSQ9_POS)		/** 10th conversion in regular sequence */
#define ADC_RSQ1_RSQ8_POS								0x0AUL		/** 9th conversion in regular sequence */
#define ADC_RSQ1_RSQ8_MSK								(0x1FUL << ADC_RSQ1_RSQ8_POS)		/** 9th conversion in regular sequence */
#define ADC_RSQ1_RSQ7_POS								0x05UL		/** 8th conversion in regular sequence */
#define ADC_RSQ1_RSQ7_MSK								(0x1FUL << ADC_RSQ1_RSQ7_POS)		/** 8th conversion in regular sequence */
#define ADC_RSQ1_RSQ6_POS								0x00UL		/** 7th conversion in regular sequence */
#define ADC_RSQ1_RSQ6_MSK								(0x1FUL << ADC_RSQ1_RSQ6_POS)		/** 7th conversion in regular sequence */
#define ADC_RSQ2_RSQ5_POS								0x19UL		/** 6th conversion in regular sequence */
#define ADC_RSQ2_RSQ5_MSK								(0x1FUL << ADC_RSQ2_RSQ5_POS)		/** 6th conversion in regular sequence */
#define ADC_RSQ2_RSQ4_POS								0x14UL		/** 5th conversion in regular sequence */
#define ADC_RSQ2_RSQ4_MSK								(0x1FUL << ADC_RSQ2_RSQ4_POS)		/** 5th conversion in regular sequence */
#define ADC_RSQ2_RSQ3_POS								0x0FUL		/** 4th conversion in regular sequence */
#define ADC_RSQ2_RSQ3_MSK								(0x1FUL << ADC_RSQ2_RSQ3_POS)		/** 4th conversion in regular sequence */
#define ADC_RSQ2_RSQ2_POS								0x0AUL		/** 3rd conversion in regular sequence */
#define ADC_RSQ2_RSQ2_MSK								(0x1FUL << ADC_RSQ2_RSQ2_POS)		/** 3rd conversion in regular sequence */
#define ADC_RSQ2_RSQ1_POS								0x05UL		/** 2nd conversion in regular sequence */
#define ADC_RSQ2_RSQ1_MSK								(0x1FUL << ADC_RSQ2_RSQ1_POS)		/** 2nd conversion in regular sequence */
#define ADC_RSQ2_RSQ0_POS								0x00UL		/** 1st conversion in regular sequence */
#define ADC_RSQ2_RSQ0_MSK								(0x1FUL << ADC_RSQ2_RSQ0_POS)		/** 1st conversion in regular sequence */
#define ADC_ISQ_IL_POS								0x14UL		/** Inserted channel group length */
#define ADC_ISQ_IL_MSK								(0x03UL << ADC_ISQ_IL_POS)		/** Inserted channel group length */
#define ADC_ISQ_ISQ3_POS								0x0FUL		/** 4th conversion in inserted sequence */
#define ADC_ISQ_ISQ3_MSK								(0x1FUL << ADC_ISQ_ISQ3_POS)		/** 4th conversion in inserted sequence */
#define ADC_ISQ_ISQ2_POS								0x0AUL		/** 3rd conversion in inserted sequence */
#define ADC_ISQ_ISQ2_MSK								(0x1FUL << ADC_ISQ_ISQ2_POS)		/** 3rd conversion in inserted sequence */
#define ADC_ISQ_ISQ1_POS								0x05UL		/** 2nd conversion in inserted sequence */
#define ADC_ISQ_ISQ1_MSK								(0x1FUL << ADC_ISQ_ISQ1_POS)		/** 2nd conversion in inserted sequence */
#define ADC_ISQ_ISQ0_POS								0x00UL		/** 1st conversion in inserted sequence */
#define ADC_ISQ_ISQ0_MSK								(0x1FUL << ADC_ISQ_ISQ0_POS)		/** 1st conversion in inserted sequence */
#define ADC_IDATA0_IDATAn_POS								0x00UL		/** Inserted number n conversion data */
#define ADC_IDATA0_IDATAn_MSK								(0xFFFFUL << ADC_IDATA0_IDATAn_POS)		/** Inserted number n conversion data */
#define ADC_IDATA1_IDATAn_POS								0x00UL		/** Inserted number n conversion data */
#define ADC_IDATA1_IDATAn_MSK								(0xFFFFUL << ADC_IDATA1_IDATAn_POS)		/** Inserted number n conversion data */
#define ADC_IDATA2_IDATAn_POS								0x00UL		/** Inserted number n conversion data */
#define ADC_IDATA2_IDATAn_MSK								(0xFFFFUL << ADC_IDATA2_IDATAn_POS)		/** Inserted number n conversion data */
#define ADC_IDATA3_IDATAn_POS								0x00UL		/** Inserted number n conversion data */
#define ADC_IDATA3_IDATAn_MSK								(0xFFFFUL << ADC_IDATA3_IDATAn_POS)		/** Inserted number n conversion data */
#define ADC_RDATA_ADC1RDTR_POS								0x10UL		/** ADC regular channel data */
#define ADC_RDATA_ADC1RDTR_MSK								(0xFFFFUL << ADC_RDATA_ADC1RDTR_POS)		/** ADC regular channel data */
#define ADC_RDATA_RDATA_POS								0x00UL		/** Regular channel data */
#define ADC_RDATA_RDATA_MSK								(0xFFFFUL << ADC_RDATA_RDATA_POS)		/** Regular channel data */
#define ADC_OVSAMPCTL_DRES_POS								0x0CUL		/** ADC resolution */
#define ADC_OVSAMPCTL_DRES_MSK								(0x03UL << ADC_OVSAMPCTL_DRES_POS)		/** ADC resolution */
#define ADC_OVSAMPCTL_TOVS_POS								0x09UL		/** Triggered Oversampling */
#define ADC_OVSAMPCTL_TOVS_MSK								(0x01UL << ADC_OVSAMPCTL_TOVS_POS)		/** Triggered Oversampling */
#define ADC_OVSAMPCTL_OVSS_POS								0x05UL		/** Oversampling shift */
#define ADC_OVSAMPCTL_OVSS_MSK								(0x0FUL << ADC_OVSAMPCTL_OVSS_POS)		/** Oversampling shift */
#define ADC_OVSAMPCTL_OVSR_POS								0x02UL		/** Oversampling ratio */
#define ADC_OVSAMPCTL_OVSR_MSK								(0x07UL << ADC_OVSAMPCTL_OVSR_POS)		/** Oversampling ratio */
#define ADC_OVSAMPCTL_OVSEN_POS								0x00UL		/** Oversampler Enable */
#define ADC_OVSAMPCTL_OVSEN_MSK								(0x01UL << ADC_OVSAMPCTL_OVSEN_POS)		/** Oversampler Enable */
#define ADC_CTL1_ETEIC_POS								0x0FUL		/** External trigger enable for inserted channel */
#define ADC_CTL1_ETEIC_MSK								(0x01UL << ADC_CTL1_ETEIC_POS)		/** External trigger enable for inserted channel */
#define AFIO_EC_EOE_POS								0x07UL		/** Event output enable */
#define AFIO_EC_EOE_MSK								(0x01UL << AFIO_EC_EOE_POS)		/** Event output enable */
#define AFIO_EC_PORT_POS								0x04UL		/** Event output port selection */
#define AFIO_EC_PORT_MSK								(0x07UL << AFIO_EC_PORT_POS)		/** Event output port selection */
#define AFIO_EC_PIN_POS								0x00UL		/** Event output pin selection */
#define AFIO_EC_PIN_MSK								(0x0FUL << AFIO_EC_PIN_POS)		/** Event output pin selection */
#define AFIO_PCF0_TIMER1ITI1_REMAP_POS								0x1DUL		/** TIMER1 internal trigger 1 remapping */
#define AFIO_PCF0_TIMER1ITI1_REMAP_MSK								(0x01UL << AFIO_PCF0_TIMER1ITI1_REMAP_POS)		/** TIMER1 internal trigger 1 remapping */
#define AFIO_PCF0_SPI2_REMAP_POS								0x1CUL		/**  SPI2/I2S2 remapping */
#define AFIO_PCF0_SPI2_REMAP_MSK								(0x01UL << AFIO_PCF0_SPI2_REMAP_POS)		/**  SPI2/I2S2 remapping */
#define AFIO_PCF0_SWJ_CFG_POS								0x18UL		/** Serial wire JTAG configuration */
#define AFIO_PCF0_SWJ_CFG_MSK								(0x07UL << AFIO_PCF0_SWJ_CFG_POS)		/** Serial wire JTAG configuration */
#define AFIO_PCF0_CAN1_REMAP_POS								0x16UL		/** CAN1 I/O remapping */
#define AFIO_PCF0_CAN1_REMAP_MSK								(0x01UL << AFIO_PCF0_CAN1_REMAP_POS)		/** CAN1 I/O remapping */
#define AFIO_PCF0_TIMER4CH3_IREMAP_POS								0x10UL		/** TIMER4 channel3 internal remapping */
#define AFIO_PCF0_TIMER4CH3_IREMAP_MSK								(0x01UL << AFIO_PCF0_TIMER4CH3_IREMAP_POS)		/** TIMER4 channel3 internal remapping */
#define AFIO_PCF0_PD01_REMAP_POS								0x0FUL		/** Port D0/Port D1 mapping on OSC_IN/OSC_OUT */
#define AFIO_PCF0_PD01_REMAP_MSK								(0x01UL << AFIO_PCF0_PD01_REMAP_POS)		/** Port D0/Port D1 mapping on OSC_IN/OSC_OUT */
#define AFIO_PCF0_CAN0_REMAP_POS								0x0DUL		/** CAN0 alternate interface remapping */
#define AFIO_PCF0_CAN0_REMAP_MSK								(0x03UL << AFIO_PCF0_CAN0_REMAP_POS)		/** CAN0 alternate interface remapping */
#define AFIO_PCF0_TIMER3_REMAP_POS								0x0CUL		/** TIMER3 remapping */
#define AFIO_PCF0_TIMER3_REMAP_MSK								(0x01UL << AFIO_PCF0_TIMER3_REMAP_POS)		/** TIMER3 remapping */
#define AFIO_PCF0_TIMER2_REMAP_POS								0x0AUL		/** TIMER2 remapping */
#define AFIO_PCF0_TIMER2_REMAP_MSK								(0x03UL << AFIO_PCF0_TIMER2_REMAP_POS)		/** TIMER2 remapping */
#define AFIO_PCF0_TIMER1_REMAP_POS								0x08UL		/** TIMER1 remapping */
#define AFIO_PCF0_TIMER1_REMAP_MSK								(0x03UL << AFIO_PCF0_TIMER1_REMAP_POS)		/** TIMER1 remapping */
#define AFIO_PCF0_TIMER0_REMAP_POS								0x06UL		/** TIMER0 remapping */
#define AFIO_PCF0_TIMER0_REMAP_MSK								(0x03UL << AFIO_PCF0_TIMER0_REMAP_POS)		/** TIMER0 remapping */
#define AFIO_PCF0_USART2_REMAP_POS								0x04UL		/** USART2 remapping */
#define AFIO_PCF0_USART2_REMAP_MSK								(0x03UL << AFIO_PCF0_USART2_REMAP_POS)		/** USART2 remapping */
#define AFIO_PCF0_USART1_REMAP_POS								0x03UL		/** USART1 remapping */
#define AFIO_PCF0_USART1_REMAP_MSK								(0x01UL << AFIO_PCF0_USART1_REMAP_POS)		/** USART1 remapping */
#define AFIO_PCF0_USART0_REMAP_POS								0x02UL		/** USART0 remapping */
#define AFIO_PCF0_USART0_REMAP_MSK								(0x01UL << AFIO_PCF0_USART0_REMAP_POS)		/** USART0 remapping */
#define AFIO_PCF0_I2C0_REMAP_POS								0x01UL		/** I2C0 remapping */
#define AFIO_PCF0_I2C0_REMAP_MSK								(0x01UL << AFIO_PCF0_I2C0_REMAP_POS)		/** I2C0 remapping */
#define AFIO_PCF0_SPI0_REMAP_POS								0x00UL		/** SPI0 remapping */
#define AFIO_PCF0_SPI0_REMAP_MSK								(0x01UL << AFIO_PCF0_SPI0_REMAP_POS)		/** SPI0 remapping */
#define AFIO_EXTISS0_EXTI3_SS_POS								0x0CUL		/** EXTI 3 sources selection */
#define AFIO_EXTISS0_EXTI3_SS_MSK								(0x0FUL << AFIO_EXTISS0_EXTI3_SS_POS)		/** EXTI 3 sources selection */
#define AFIO_EXTISS0_EXTI2_SS_POS								0x08UL		/** EXTI 2 sources selection */
#define AFIO_EXTISS0_EXTI2_SS_MSK								(0x0FUL << AFIO_EXTISS0_EXTI2_SS_POS)		/** EXTI 2 sources selection */
#define AFIO_EXTISS0_EXTI1_SS_POS								0x04UL		/** EXTI 1 sources selection */
#define AFIO_EXTISS0_EXTI1_SS_MSK								(0x0FUL << AFIO_EXTISS0_EXTI1_SS_POS)		/** EXTI 1 sources selection */
#define AFIO_EXTISS0_EXTI0_SS_POS								0x00UL		/** EXTI 0 sources selection */
#define AFIO_EXTISS0_EXTI0_SS_MSK								(0x0FUL << AFIO_EXTISS0_EXTI0_SS_POS)		/** EXTI 0 sources selection */
#define AFIO_EXTISS1_EXTI7_SS_POS								0x0CUL		/** EXTI 7 sources selection */
#define AFIO_EXTISS1_EXTI7_SS_MSK								(0x0FUL << AFIO_EXTISS1_EXTI7_SS_POS)		/** EXTI 7 sources selection */
#define AFIO_EXTISS1_EXTI6_SS_POS								0x08UL		/** EXTI 6 sources selection */
#define AFIO_EXTISS1_EXTI6_SS_MSK								(0x0FUL << AFIO_EXTISS1_EXTI6_SS_POS)		/** EXTI 6 sources selection */
#define AFIO_EXTISS1_EXTI5_SS_POS								0x04UL		/** EXTI 5 sources selection */
#define AFIO_EXTISS1_EXTI5_SS_MSK								(0x0FUL << AFIO_EXTISS1_EXTI5_SS_POS)		/** EXTI 5 sources selection */
#define AFIO_EXTISS1_EXTI4_SS_POS								0x00UL		/** EXTI 4 sources selection */
#define AFIO_EXTISS1_EXTI4_SS_MSK								(0x0FUL << AFIO_EXTISS1_EXTI4_SS_POS)		/** EXTI 4 sources selection */
#define AFIO_EXTISS2_EXTI11_SS_POS								0x0CUL		/** EXTI 11 sources selection */
#define AFIO_EXTISS2_EXTI11_SS_MSK								(0x0FUL << AFIO_EXTISS2_EXTI11_SS_POS)		/** EXTI 11 sources selection */
#define AFIO_EXTISS2_EXTI10_SS_POS								0x08UL		/** EXTI 10 sources selection */
#define AFIO_EXTISS2_EXTI10_SS_MSK								(0x0FUL << AFIO_EXTISS2_EXTI10_SS_POS)		/** EXTI 10 sources selection */
#define AFIO_EXTISS2_EXTI9_SS_POS								0x04UL		/** EXTI 9 sources selection */
#define AFIO_EXTISS2_EXTI9_SS_MSK								(0x0FUL << AFIO_EXTISS2_EXTI9_SS_POS)		/** EXTI 9 sources selection */
#define AFIO_EXTISS2_EXTI8_SS_POS								0x00UL		/** EXTI 8 sources selection */
#define AFIO_EXTISS2_EXTI8_SS_MSK								(0x0FUL << AFIO_EXTISS2_EXTI8_SS_POS)		/** EXTI 8 sources selection */
#define AFIO_EXTISS3_EXTI15_SS_POS								0x0CUL		/** EXTI 15 sources selection */
#define AFIO_EXTISS3_EXTI15_SS_MSK								(0x0FUL << AFIO_EXTISS3_EXTI15_SS_POS)		/** EXTI 15 sources selection */
#define AFIO_EXTISS3_EXTI14_SS_POS								0x08UL		/** EXTI 14 sources selection */
#define AFIO_EXTISS3_EXTI14_SS_MSK								(0x0FUL << AFIO_EXTISS3_EXTI14_SS_POS)		/** EXTI 14 sources selection */
#define AFIO_EXTISS3_EXTI13_SS_POS								0x04UL		/** EXTI 13 sources selection */
#define AFIO_EXTISS3_EXTI13_SS_MSK								(0x0FUL << AFIO_EXTISS3_EXTI13_SS_POS)		/** EXTI 13 sources selection */
#define AFIO_EXTISS3_EXTI12_SS_POS								0x00UL		/** EXTI 12 sources selection */
#define AFIO_EXTISS3_EXTI12_SS_MSK								(0x0FUL << AFIO_EXTISS3_EXTI12_SS_POS)		/** EXTI 12 sources selection */
#define AFIO_PCF1_EXMC_NADV_POS								0x0AUL		/** EXMC_NADV connect/disconnect */
#define AFIO_PCF1_EXMC_NADV_MSK								(0x01UL << AFIO_PCF1_EXMC_NADV_POS)		/** EXMC_NADV connect/disconnect */
#define BKP_DATA0_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA0_DATA_MSK								(0xFFFFUL << BKP_DATA0_DATA_POS)		/** Backup data */
#define BKP_DATA1_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA1_DATA_MSK								(0xFFFFUL << BKP_DATA1_DATA_POS)		/** Backup data */
#define BKP_DATA2_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA2_DATA_MSK								(0xFFFFUL << BKP_DATA2_DATA_POS)		/** Backup data */
#define BKP_DATA3_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA3_DATA_MSK								(0xFFFFUL << BKP_DATA3_DATA_POS)		/** Backup data */
#define BKP_DATA4_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA4_DATA_MSK								(0xFFFFUL << BKP_DATA4_DATA_POS)		/** Backup data */
#define BKP_DATA5_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA5_DATA_MSK								(0xFFFFUL << BKP_DATA5_DATA_POS)		/** Backup data */
#define BKP_DATA6_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA6_DATA_MSK								(0xFFFFUL << BKP_DATA6_DATA_POS)		/** Backup data */
#define BKP_DATA7_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA7_DATA_MSK								(0xFFFFUL << BKP_DATA7_DATA_POS)		/** Backup data */
#define BKP_DATA8_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA8_DATA_MSK								(0xFFFFUL << BKP_DATA8_DATA_POS)		/** Backup data */
#define BKP_DATA9_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA9_DATA_MSK								(0xFFFFUL << BKP_DATA9_DATA_POS)		/** Backup data */
#define BKP_DATA10_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA10_DATA_MSK								(0xFFFFUL << BKP_DATA10_DATA_POS)		/** Backup data */
#define BKP_DATA11_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA11_DATA_MSK								(0xFFFFUL << BKP_DATA11_DATA_POS)		/** Backup data */
#define BKP_DATA12_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA12_DATA_MSK								(0xFFFFUL << BKP_DATA12_DATA_POS)		/** Backup data */
#define BKP_DATA13_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA13_DATA_MSK								(0xFFFFUL << BKP_DATA13_DATA_POS)		/** Backup data */
#define BKP_DATA14_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA14_DATA_MSK								(0xFFFFUL << BKP_DATA14_DATA_POS)		/** Backup data */
#define BKP_DATA15_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA15_DATA_MSK								(0xFFFFUL << BKP_DATA15_DATA_POS)		/** Backup data */
#define BKP_DATA16_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA16_DATA_MSK								(0xFFFFUL << BKP_DATA16_DATA_POS)		/** Backup data */
#define BKP_DATA17_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA17_DATA_MSK								(0xFFFFUL << BKP_DATA17_DATA_POS)		/** Backup data */
#define BKP_DATA18_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA18_DATA_MSK								(0xFFFFUL << BKP_DATA18_DATA_POS)		/** Backup data */
#define BKP_DATA19_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA19_DATA_MSK								(0xFFFFUL << BKP_DATA19_DATA_POS)		/** Backup data */
#define BKP_DATA20_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA20_DATA_MSK								(0xFFFFUL << BKP_DATA20_DATA_POS)		/** Backup data */
#define BKP_DATA21_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA21_DATA_MSK								(0xFFFFUL << BKP_DATA21_DATA_POS)		/** Backup data */
#define BKP_DATA22_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA22_DATA_MSK								(0xFFFFUL << BKP_DATA22_DATA_POS)		/** Backup data */
#define BKP_DATA23_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA23_DATA_MSK								(0xFFFFUL << BKP_DATA23_DATA_POS)		/** Backup data */
#define BKP_DATA24_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA24_DATA_MSK								(0xFFFFUL << BKP_DATA24_DATA_POS)		/** Backup data */
#define BKP_DATA25_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA25_DATA_MSK								(0xFFFFUL << BKP_DATA25_DATA_POS)		/** Backup data */
#define BKP_DATA26_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA26_DATA_MSK								(0xFFFFUL << BKP_DATA26_DATA_POS)		/** Backup data */
#define BKP_DATA27_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA27_DATA_MSK								(0xFFFFUL << BKP_DATA27_DATA_POS)		/** Backup data */
#define BKP_DATA28_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA28_DATA_MSK								(0xFFFFUL << BKP_DATA28_DATA_POS)		/** Backup data */
#define BKP_DATA29_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA29_DATA_MSK								(0xFFFFUL << BKP_DATA29_DATA_POS)		/** Backup data */
#define BKP_DATA30_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA30_DATA_MSK								(0xFFFFUL << BKP_DATA30_DATA_POS)		/** Backup data */
#define BKP_DATA31_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA31_DATA_MSK								(0xFFFFUL << BKP_DATA31_DATA_POS)		/** Backup data */
#define BKP_DATA32_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA32_DATA_MSK								(0xFFFFUL << BKP_DATA32_DATA_POS)		/** Backup data */
#define BKP_DATA33_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA33_DATA_MSK								(0xFFFFUL << BKP_DATA33_DATA_POS)		/** Backup data */
#define BKP_DATA34_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA34_DATA_MSK								(0xFFFFUL << BKP_DATA34_DATA_POS)		/** Backup data */
#define BKP_DATA35_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA35_DATA_MSK								(0xFFFFUL << BKP_DATA35_DATA_POS)		/** Backup data */
#define BKP_DATA36_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA36_DATA_MSK								(0xFFFFUL << BKP_DATA36_DATA_POS)		/** Backup data */
#define BKP_DATA37_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA37_DATA_MSK								(0xFFFFUL << BKP_DATA37_DATA_POS)		/** Backup data */
#define BKP_DATA38_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA38_DATA_MSK								(0xFFFFUL << BKP_DATA38_DATA_POS)		/** Backup data */
#define BKP_DATA39_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA39_DATA_MSK								(0xFFFFUL << BKP_DATA39_DATA_POS)		/** Backup data */
#define BKP_DATA40_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA40_DATA_MSK								(0xFFFFUL << BKP_DATA40_DATA_POS)		/** Backup data */
#define BKP_DATA41_DATA_POS								0x00UL		/** Backup data */
#define BKP_DATA41_DATA_MSK								(0xFFFFUL << BKP_DATA41_DATA_POS)		/** Backup data */
#define BKP_OCTL_ROSEL_POS								0x09UL		/** RTC output selection */
#define BKP_OCTL_ROSEL_MSK								(0x01UL << BKP_OCTL_ROSEL_POS)		/** RTC output selection */
#define BKP_OCTL_ASOEN_POS								0x08UL		/** RTC alarm or second signal output enable */
#define BKP_OCTL_ASOEN_MSK								(0x01UL << BKP_OCTL_ASOEN_POS)		/** RTC alarm or second signal output enable */
#define BKP_OCTL_COEN_POS								0x07UL		/** RTC clock calibration output enable */
#define BKP_OCTL_COEN_MSK								(0x01UL << BKP_OCTL_COEN_POS)		/** RTC clock calibration output enable */
#define BKP_OCTL_RCCV_POS								0x00UL		/** RTC clock calibration value */
#define BKP_OCTL_RCCV_MSK								(0x7FUL << BKP_OCTL_RCCV_POS)		/** RTC clock calibration value */
#define BKP_TPCTL_TPAL_POS								0x01UL		/** TAMPER pin active level */
#define BKP_TPCTL_TPAL_MSK								(0x01UL << BKP_TPCTL_TPAL_POS)		/** TAMPER pin active level */
#define BKP_TPCTL_TPEN_POS								0x00UL		/** TAMPER detection enable */
#define BKP_TPCTL_TPEN_MSK								(0x01UL << BKP_TPCTL_TPEN_POS)		/** TAMPER detection enable */
#define BKP_TPCS_TIF_POS								0x09UL		/** Tamper interrupt flag */
#define BKP_TPCS_TIF_MSK								(0x01UL << BKP_TPCS_TIF_POS)		/** Tamper interrupt flag */
#define BKP_TPCS_TEF_POS								0x08UL		/** Tamper event flag */
#define BKP_TPCS_TEF_MSK								(0x01UL << BKP_TPCS_TEF_POS)		/** Tamper event flag */
#define BKP_TPCS_TPIE_POS								0x02UL		/** Tamper interrupt enable */
#define BKP_TPCS_TPIE_MSK								(0x01UL << BKP_TPCS_TPIE_POS)		/** Tamper interrupt enable */
#define BKP_TPCS_TIR_POS								0x01UL		/** Tamper interrupt reset */
#define BKP_TPCS_TIR_MSK								(0x01UL << BKP_TPCS_TIR_POS)		/** Tamper interrupt reset */
#define BKP_TPCS_TER_POS								0x00UL		/** Tamper event reset */
#define BKP_TPCS_TER_MSK								(0x01UL << BKP_TPCS_TER_POS)		/** Tamper event reset */
#define CAN_CTL_DFZ_POS								0x10UL		/** Debug freeze */
#define CAN_CTL_DFZ_MSK								(0x01UL << CAN_CTL_DFZ_POS)		/** Debug freeze */
#define CAN_CTL_SWRST_POS								0x0FUL		/** Software reset */
#define CAN_CTL_SWRST_MSK								(0x01UL << CAN_CTL_SWRST_POS)		/** Software reset */
#define CAN_CTL_TTC_POS								0x07UL		/** Time-triggered communication */
#define CAN_CTL_TTC_MSK								(0x01UL << CAN_CTL_TTC_POS)		/** Time-triggered communication */
#define CAN_CTL_ABOR_POS								0x06UL		/** Automatic bus-off recovery */
#define CAN_CTL_ABOR_MSK								(0x01UL << CAN_CTL_ABOR_POS)		/** Automatic bus-off recovery */
#define CAN_CTL_AWU_POS								0x05UL		/** Automatic wakeup */
#define CAN_CTL_AWU_MSK								(0x01UL << CAN_CTL_AWU_POS)		/** Automatic wakeup */
#define CAN_CTL_ARD_POS								0x04UL		/** Automatic retransmission disable */
#define CAN_CTL_ARD_MSK								(0x01UL << CAN_CTL_ARD_POS)		/** Automatic retransmission disable */
#define CAN_CTL_RFOD_POS								0x03UL		/** Receive FIFO overwrite disable */
#define CAN_CTL_RFOD_MSK								(0x01UL << CAN_CTL_RFOD_POS)		/** Receive FIFO overwrite disable */
#define CAN_CTL_TFO_POS								0x02UL		/** Transmit FIFO order */
#define CAN_CTL_TFO_MSK								(0x01UL << CAN_CTL_TFO_POS)		/** Transmit FIFO order */
#define CAN_CTL_SLPWMOD_POS								0x01UL		/** Sleep working mode */
#define CAN_CTL_SLPWMOD_MSK								(0x01UL << CAN_CTL_SLPWMOD_POS)		/** Sleep working mode */
#define CAN_CTL_IWMOD_POS								0x00UL		/** Initial working mode */
#define CAN_CTL_IWMOD_MSK								(0x01UL << CAN_CTL_IWMOD_POS)		/** Initial working mode */
#define CAN_STAT_RXL_POS								0x0BUL		/** RX level */
#define CAN_STAT_RXL_MSK								(0x01UL << CAN_STAT_RXL_POS)		/** RX level */
#define CAN_STAT_LASTRX_POS								0x0AUL		/** Last sample value of RX pin */
#define CAN_STAT_LASTRX_MSK								(0x01UL << CAN_STAT_LASTRX_POS)		/** Last sample value of RX pin */
#define CAN_STAT_RS_POS								0x09UL		/** Receiving state */
#define CAN_STAT_RS_MSK								(0x01UL << CAN_STAT_RS_POS)		/** Receiving state */
#define CAN_STAT_TS_POS								0x08UL		/** Transmitting state */
#define CAN_STAT_TS_MSK								(0x01UL << CAN_STAT_TS_POS)		/** Transmitting state */
#define CAN_STAT_SLPIF_POS								0x04UL		/** Status change interrupt flag of sleep working mode entering */
#define CAN_STAT_SLPIF_MSK								(0x01UL << CAN_STAT_SLPIF_POS)		/** Status change interrupt flag of sleep working mode entering */
#define CAN_STAT_WUIF_POS								0x03UL		/** Status change interrupt flag of wakeup from sleep working mode */
#define CAN_STAT_WUIF_MSK								(0x01UL << CAN_STAT_WUIF_POS)		/** Status change interrupt flag of wakeup from sleep working mode */
#define CAN_STAT_ERRIF_POS								0x02UL		/** Error interrupt flag */
#define CAN_STAT_ERRIF_MSK								(0x01UL << CAN_STAT_ERRIF_POS)		/** Error interrupt flag */
#define CAN_STAT_SLPWS_POS								0x01UL		/** Sleep working state */
#define CAN_STAT_SLPWS_MSK								(0x01UL << CAN_STAT_SLPWS_POS)		/** Sleep working state */
#define CAN_STAT_IWS_POS								0x00UL		/** Initial working state */
#define CAN_STAT_IWS_MSK								(0x01UL << CAN_STAT_IWS_POS)		/** Initial working state */
#define CAN_TSTAT_TMLS2_POS								0x1FUL		/** Transmit mailbox 2 last sending in transmit FIFO */
#define CAN_TSTAT_TMLS2_MSK								(0x01UL << CAN_TSTAT_TMLS2_POS)		/** Transmit mailbox 2 last sending in transmit FIFO */
#define CAN_TSTAT_TMLS1_POS								0x1EUL		/** Transmit mailbox 1 last sending in transmit FIFO */
#define CAN_TSTAT_TMLS1_MSK								(0x01UL << CAN_TSTAT_TMLS1_POS)		/** Transmit mailbox 1 last sending in transmit FIFO */
#define CAN_TSTAT_TMLS0_POS								0x1DUL		/** Transmit mailbox 0 last sending in transmit FIFO */
#define CAN_TSTAT_TMLS0_MSK								(0x01UL << CAN_TSTAT_TMLS0_POS)		/** Transmit mailbox 0 last sending in transmit FIFO */
#define CAN_TSTAT_TME2_POS								0x1CUL		/** Transmit mailbox 2 empty */
#define CAN_TSTAT_TME2_MSK								(0x01UL << CAN_TSTAT_TME2_POS)		/** Transmit mailbox 2 empty */
#define CAN_TSTAT_TME1_POS								0x1BUL		/** Transmit mailbox 1 empty */
#define CAN_TSTAT_TME1_MSK								(0x01UL << CAN_TSTAT_TME1_POS)		/** Transmit mailbox 1 empty */
#define CAN_TSTAT_TME0_POS								0x1AUL		/** Transmit mailbox 0 empty */
#define CAN_TSTAT_TME0_MSK								(0x01UL << CAN_TSTAT_TME0_POS)		/** Transmit mailbox 0 empty */
#define CAN_TSTAT_NUM_POS								0x18UL		/** number of the transmit FIFO mailbox in which the frame will be transmitted if at least one mailbox is empty */
#define CAN_TSTAT_NUM_MSK								(0x03UL << CAN_TSTAT_NUM_POS)		/** number of the transmit FIFO mailbox in which the frame will be transmitted if at least one mailbox is empty */
#define CAN_TSTAT_MST2_POS								0x17UL		/** Mailbox 2 stop transmitting */
#define CAN_TSTAT_MST2_MSK								(0x01UL << CAN_TSTAT_MST2_POS)		/** Mailbox 2 stop transmitting */
#define CAN_TSTAT_MTE2_POS								0x13UL		/** Mailbox 2 transmit error */
#define CAN_TSTAT_MTE2_MSK								(0x01UL << CAN_TSTAT_MTE2_POS)		/** Mailbox 2 transmit error */
#define CAN_TSTAT_MAL2_POS								0x12UL		/** Mailbox 2 arbitration lost */
#define CAN_TSTAT_MAL2_MSK								(0x01UL << CAN_TSTAT_MAL2_POS)		/** Mailbox 2 arbitration lost */
#define CAN_TSTAT_MTFNERR2_POS								0x11UL		/** Mailbox 2 transmit finished and no error */
#define CAN_TSTAT_MTFNERR2_MSK								(0x01UL << CAN_TSTAT_MTFNERR2_POS)		/** Mailbox 2 transmit finished and no error */
#define CAN_TSTAT_MTF2_POS								0x10UL		/** Mailbox 2 transmit finished */
#define CAN_TSTAT_MTF2_MSK								(0x01UL << CAN_TSTAT_MTF2_POS)		/** Mailbox 2 transmit finished */
#define CAN_TSTAT_MST1_POS								0x0FUL		/** Mailbox 1 stop transmitting */
#define CAN_TSTAT_MST1_MSK								(0x01UL << CAN_TSTAT_MST1_POS)		/** Mailbox 1 stop transmitting */
#define CAN_TSTAT_MTE1_POS								0x0BUL		/** Mailbox 1 transmit error */
#define CAN_TSTAT_MTE1_MSK								(0x01UL << CAN_TSTAT_MTE1_POS)		/** Mailbox 1 transmit error */
#define CAN_TSTAT_MAL1_POS								0x0AUL		/** Mailbox 1 arbitration lost */
#define CAN_TSTAT_MAL1_MSK								(0x01UL << CAN_TSTAT_MAL1_POS)		/** Mailbox 1 arbitration lost */
#define CAN_TSTAT_MTFNERR1_POS								0x09UL		/** Mailbox 1 transmit finished and no error */
#define CAN_TSTAT_MTFNERR1_MSK								(0x01UL << CAN_TSTAT_MTFNERR1_POS)		/** Mailbox 1 transmit finished and no error */
#define CAN_TSTAT_MTF1_POS								0x08UL		/** Mailbox 1 transmit finished */
#define CAN_TSTAT_MTF1_MSK								(0x01UL << CAN_TSTAT_MTF1_POS)		/** Mailbox 1 transmit finished */
#define CAN_TSTAT_MST0_POS								0x07UL		/** Mailbox 0 stop transmitting */
#define CAN_TSTAT_MST0_MSK								(0x01UL << CAN_TSTAT_MST0_POS)		/** Mailbox 0 stop transmitting */
#define CAN_TSTAT_MTE0_POS								0x03UL		/** Mailbox 0 transmit error */
#define CAN_TSTAT_MTE0_MSK								(0x01UL << CAN_TSTAT_MTE0_POS)		/** Mailbox 0 transmit error */
#define CAN_TSTAT_MAL0_POS								0x02UL		/** Mailbox 0 arbitration lost */
#define CAN_TSTAT_MAL0_MSK								(0x01UL << CAN_TSTAT_MAL0_POS)		/** Mailbox 0 arbitration lost */
#define CAN_TSTAT_MTFNERR0_POS								0x01UL		/** Mailbox 0 transmit finished and no error */
#define CAN_TSTAT_MTFNERR0_MSK								(0x01UL << CAN_TSTAT_MTFNERR0_POS)		/** Mailbox 0 transmit finished and no error */
#define CAN_TSTAT_MTF0_POS								0x00UL		/** Mailbox 0 transmit finished */
#define CAN_TSTAT_MTF0_MSK								(0x01UL << CAN_TSTAT_MTF0_POS)		/** Mailbox 0 transmit finished */
#define CAN_RFIFO0_RFD0_POS								0x05UL		/** Receive FIFO0 dequeue */
#define CAN_RFIFO0_RFD0_MSK								(0x01UL << CAN_RFIFO0_RFD0_POS)		/** Receive FIFO0 dequeue */
#define CAN_RFIFO0_RFO0_POS								0x04UL		/** Receive FIFO0 overfull */
#define CAN_RFIFO0_RFO0_MSK								(0x01UL << CAN_RFIFO0_RFO0_POS)		/** Receive FIFO0 overfull */
#define CAN_RFIFO0_RFF0_POS								0x03UL		/** Receive FIFO0 full */
#define CAN_RFIFO0_RFF0_MSK								(0x01UL << CAN_RFIFO0_RFF0_POS)		/** Receive FIFO0 full */
#define CAN_RFIFO0_RFL0_POS								0x00UL		/** Receive FIFO0 length */
#define CAN_RFIFO0_RFL0_MSK								(0x03UL << CAN_RFIFO0_RFL0_POS)		/** Receive FIFO0 length */
#define CAN_RFIFO1_RFD1_POS								0x05UL		/** Receive FIFO1 dequeue */
#define CAN_RFIFO1_RFD1_MSK								(0x01UL << CAN_RFIFO1_RFD1_POS)		/** Receive FIFO1 dequeue */
#define CAN_RFIFO1_RFO1_POS								0x04UL		/** Receive FIFO1 overfull */
#define CAN_RFIFO1_RFO1_MSK								(0x01UL << CAN_RFIFO1_RFO1_POS)		/** Receive FIFO1 overfull */
#define CAN_RFIFO1_RFF1_POS								0x03UL		/** Receive FIFO1 full */
#define CAN_RFIFO1_RFF1_MSK								(0x01UL << CAN_RFIFO1_RFF1_POS)		/** Receive FIFO1 full */
#define CAN_RFIFO1_RFL1_POS								0x00UL		/** Receive FIFO1 length */
#define CAN_RFIFO1_RFL1_MSK								(0x03UL << CAN_RFIFO1_RFL1_POS)		/** Receive FIFO1 length */
#define CAN_INTEN_SLPWIE_POS								0x11UL		/** Sleep working interrupt enable */
#define CAN_INTEN_SLPWIE_MSK								(0x01UL << CAN_INTEN_SLPWIE_POS)		/** Sleep working interrupt enable */
#define CAN_INTEN_WIE_POS								0x10UL		/** Wakeup interrupt enable */
#define CAN_INTEN_WIE_MSK								(0x01UL << CAN_INTEN_WIE_POS)		/** Wakeup interrupt enable */
#define CAN_INTEN_ERRIE_POS								0x0FUL		/** Error interrupt enable */
#define CAN_INTEN_ERRIE_MSK								(0x01UL << CAN_INTEN_ERRIE_POS)		/** Error interrupt enable */
#define CAN_INTEN_ERRNIE_POS								0x0BUL		/** Error number interrupt enable */
#define CAN_INTEN_ERRNIE_MSK								(0x01UL << CAN_INTEN_ERRNIE_POS)		/** Error number interrupt enable */
#define CAN_INTEN_BOIE_POS								0x0AUL		/** Bus-off interrupt enable */
#define CAN_INTEN_BOIE_MSK								(0x01UL << CAN_INTEN_BOIE_POS)		/** Bus-off interrupt enable */
#define CAN_INTEN_PERRIE_POS								0x09UL		/** Passive error interrupt enable */
#define CAN_INTEN_PERRIE_MSK								(0x01UL << CAN_INTEN_PERRIE_POS)		/** Passive error interrupt enable */
#define CAN_INTEN_WERRIE_POS								0x08UL		/** Warning error interrupt enable */
#define CAN_INTEN_WERRIE_MSK								(0x01UL << CAN_INTEN_WERRIE_POS)		/** Warning error interrupt enable */
#define CAN_INTEN_RFOIE1_POS								0x06UL		/** Receive FIFO1 overfull interrupt enable */
#define CAN_INTEN_RFOIE1_MSK								(0x01UL << CAN_INTEN_RFOIE1_POS)		/** Receive FIFO1 overfull interrupt enable */
#define CAN_INTEN_RFFIE1_POS								0x05UL		/** Receive FIFO1 full interrupt enable */
#define CAN_INTEN_RFFIE1_MSK								(0x01UL << CAN_INTEN_RFFIE1_POS)		/** Receive FIFO1 full interrupt enable */
#define CAN_INTEN_RFNEIE1_POS								0x04UL		/** Receive FIFO1 not empty interrupt enable */
#define CAN_INTEN_RFNEIE1_MSK								(0x01UL << CAN_INTEN_RFNEIE1_POS)		/** Receive FIFO1 not empty interrupt enable */
#define CAN_INTEN_RFOIE0_POS								0x03UL		/** Receive FIFO0 overfull interrupt enable */
#define CAN_INTEN_RFOIE0_MSK								(0x01UL << CAN_INTEN_RFOIE0_POS)		/** Receive FIFO0 overfull interrupt enable */
#define CAN_INTEN_RFFIE0_POS								0x02UL		/** Receive FIFO0 full interrupt enable */
#define CAN_INTEN_RFFIE0_MSK								(0x01UL << CAN_INTEN_RFFIE0_POS)		/** Receive FIFO0 full interrupt enable */
#define CAN_INTEN_RFNEIE0_POS								0x01UL		/** Receive FIFO0 not empty interrupt enable */
#define CAN_INTEN_RFNEIE0_MSK								(0x01UL << CAN_INTEN_RFNEIE0_POS)		/** Receive FIFO0 not empty interrupt enable */
#define CAN_INTEN_TMEIE_POS								0x00UL		/** Transmit mailbox empty interrupt enable */
#define CAN_INTEN_TMEIE_MSK								(0x01UL << CAN_INTEN_TMEIE_POS)		/** Transmit mailbox empty interrupt enable */
#define CAN_ERR_RECNT_POS								0x18UL		/** Receive Error Count defined by the CAN standard */
#define CAN_ERR_RECNT_MSK								(0xFFUL << CAN_ERR_RECNT_POS)		/** Receive Error Count defined by the CAN standard */
#define CAN_ERR_TECNT_POS								0x10UL		/** Transmit Error Count defined by the CAN standard */
#define CAN_ERR_TECNT_MSK								(0xFFUL << CAN_ERR_TECNT_POS)		/** Transmit Error Count defined by the CAN standard */
#define CAN_ERR_ERRN_POS								0x04UL		/** Error number */
#define CAN_ERR_ERRN_MSK								(0x07UL << CAN_ERR_ERRN_POS)		/** Error number */
#define CAN_ERR_BOERR_POS								0x02UL		/** Bus-off error */
#define CAN_ERR_BOERR_MSK								(0x01UL << CAN_ERR_BOERR_POS)		/** Bus-off error */
#define CAN_ERR_PERR_POS								0x01UL		/** Passive error */
#define CAN_ERR_PERR_MSK								(0x01UL << CAN_ERR_PERR_POS)		/** Passive error */
#define CAN_ERR_WERR_POS								0x00UL		/** Warning error */
#define CAN_ERR_WERR_MSK								(0x01UL << CAN_ERR_WERR_POS)		/** Warning error */
#define CAN_BT_SCMOD_POS								0x1FUL		/** Silent communication mode */
#define CAN_BT_SCMOD_MSK								(0x01UL << CAN_BT_SCMOD_POS)		/** Silent communication mode */
#define CAN_BT_LCMOD_POS								0x1EUL		/** Loopback communication mode */
#define CAN_BT_LCMOD_MSK								(0x01UL << CAN_BT_LCMOD_POS)		/** Loopback communication mode */
#define CAN_BT_SJW_POS								0x18UL		/** Resynchronization jump width */
#define CAN_BT_SJW_MSK								(0x03UL << CAN_BT_SJW_POS)		/** Resynchronization jump width */
#define CAN_BT_BS2_POS								0x14UL		/** Bit segment 2 */
#define CAN_BT_BS2_MSK								(0x07UL << CAN_BT_BS2_POS)		/** Bit segment 2 */
#define CAN_BT_BS1_POS								0x10UL		/** Bit segment 1 */
#define CAN_BT_BS1_MSK								(0x0FUL << CAN_BT_BS1_POS)		/** Bit segment 1 */
#define CAN_BT_BAUDPSC_POS								0x00UL		/** Baud rate prescaler */
#define CAN_BT_BAUDPSC_MSK								(0x3FFUL << CAN_BT_BAUDPSC_POS)		/** Baud rate prescaler */
#define CAN_TMI0_SFID_EFID_POS								0x15UL		/** The frame identifier */
#define CAN_TMI0_SFID_EFID_MSK								(0x7FFUL << CAN_TMI0_SFID_EFID_POS)		/** The frame identifier */
#define CAN_TMI0_EFID_POS								0x03UL		/** The frame identifier */
#define CAN_TMI0_EFID_MSK								(0x3FFFFUL << CAN_TMI0_EFID_POS)		/** The frame identifier */
#define CAN_TMI0_FF_POS								0x02UL		/** Frame format */
#define CAN_TMI0_FF_MSK								(0x01UL << CAN_TMI0_FF_POS)		/** Frame format */
#define CAN_TMI0_FT_POS								0x01UL		/** Frame type */
#define CAN_TMI0_FT_MSK								(0x01UL << CAN_TMI0_FT_POS)		/** Frame type */
#define CAN_TMI0_TEN_POS								0x00UL		/** Transmit enable */
#define CAN_TMI0_TEN_MSK								(0x01UL << CAN_TMI0_TEN_POS)		/** Transmit enable */
#define CAN_TMP0_TS_POS								0x10UL		/** Time stamp */
#define CAN_TMP0_TS_MSK								(0xFFFFUL << CAN_TMP0_TS_POS)		/** Time stamp */
#define CAN_TMP0_TSEN_POS								0x08UL		/** Time stamp enable */
#define CAN_TMP0_TSEN_MSK								(0x01UL << CAN_TMP0_TSEN_POS)		/** Time stamp enable */
#define CAN_TMP0_DLENC_POS								0x00UL		/** Data length code */
#define CAN_TMP0_DLENC_MSK								(0x0FUL << CAN_TMP0_DLENC_POS)		/** Data length code */
#define CAN_TMDATA00_DB3_POS								0x18UL		/** Data byte 3 */
#define CAN_TMDATA00_DB3_MSK								(0xFFUL << CAN_TMDATA00_DB3_POS)		/** Data byte 3 */
#define CAN_TMDATA00_DB2_POS								0x10UL		/** Data byte 2 */
#define CAN_TMDATA00_DB2_MSK								(0xFFUL << CAN_TMDATA00_DB2_POS)		/** Data byte 2 */
#define CAN_TMDATA00_DB1_POS								0x08UL		/** Data byte 1 */
#define CAN_TMDATA00_DB1_MSK								(0xFFUL << CAN_TMDATA00_DB1_POS)		/** Data byte 1 */
#define CAN_TMDATA00_DB0_POS								0x00UL		/** Data byte 0 */
#define CAN_TMDATA00_DB0_MSK								(0xFFUL << CAN_TMDATA00_DB0_POS)		/** Data byte 0 */
#define CAN_TMDATA10_DB7_POS								0x18UL		/** Data byte 7 */
#define CAN_TMDATA10_DB7_MSK								(0xFFUL << CAN_TMDATA10_DB7_POS)		/** Data byte 7 */
#define CAN_TMDATA10_DB6_POS								0x10UL		/** Data byte 6 */
#define CAN_TMDATA10_DB6_MSK								(0xFFUL << CAN_TMDATA10_DB6_POS)		/** Data byte 6 */
#define CAN_TMDATA10_DB5_POS								0x08UL		/** Data byte 5 */
#define CAN_TMDATA10_DB5_MSK								(0xFFUL << CAN_TMDATA10_DB5_POS)		/** Data byte 5 */
#define CAN_TMDATA10_DB4_POS								0x00UL		/** Data byte 4 */
#define CAN_TMDATA10_DB4_MSK								(0xFFUL << CAN_TMDATA10_DB4_POS)		/** Data byte 4 */
#define CAN_TMI1_SFID_EFID_POS								0x15UL		/** The frame identifier */
#define CAN_TMI1_SFID_EFID_MSK								(0x7FFUL << CAN_TMI1_SFID_EFID_POS)		/** The frame identifier */
#define CAN_TMI1_EFID_POS								0x03UL		/** The frame identifier */
#define CAN_TMI1_EFID_MSK								(0x3FFFFUL << CAN_TMI1_EFID_POS)		/** The frame identifier */
#define CAN_TMI1_FF_POS								0x02UL		/** Frame format */
#define CAN_TMI1_FF_MSK								(0x01UL << CAN_TMI1_FF_POS)		/** Frame format */
#define CAN_TMI1_FT_POS								0x01UL		/** Frame type */
#define CAN_TMI1_FT_MSK								(0x01UL << CAN_TMI1_FT_POS)		/** Frame type */
#define CAN_TMI1_TEN_POS								0x00UL		/** Transmit enable */
#define CAN_TMI1_TEN_MSK								(0x01UL << CAN_TMI1_TEN_POS)		/** Transmit enable */
#define CAN_TMP1_TS_POS								0x10UL		/** Time stamp */
#define CAN_TMP1_TS_MSK								(0xFFFFUL << CAN_TMP1_TS_POS)		/** Time stamp */
#define CAN_TMP1_TSEN_POS								0x08UL		/** Time stamp enable */
#define CAN_TMP1_TSEN_MSK								(0x01UL << CAN_TMP1_TSEN_POS)		/** Time stamp enable */
#define CAN_TMP1_DLENC_POS								0x00UL		/** Data length code */
#define CAN_TMP1_DLENC_MSK								(0x0FUL << CAN_TMP1_DLENC_POS)		/** Data length code */
#define CAN_TMDATA01_DB3_POS								0x18UL		/** Data byte 3 */
#define CAN_TMDATA01_DB3_MSK								(0xFFUL << CAN_TMDATA01_DB3_POS)		/** Data byte 3 */
#define CAN_TMDATA01_DB2_POS								0x10UL		/** Data byte 2 */
#define CAN_TMDATA01_DB2_MSK								(0xFFUL << CAN_TMDATA01_DB2_POS)		/** Data byte 2 */
#define CAN_TMDATA01_DB1_POS								0x08UL		/** Data byte 1 */
#define CAN_TMDATA01_DB1_MSK								(0xFFUL << CAN_TMDATA01_DB1_POS)		/** Data byte 1 */
#define CAN_TMDATA01_DB0_POS								0x00UL		/** Data byte 0 */
#define CAN_TMDATA01_DB0_MSK								(0xFFUL << CAN_TMDATA01_DB0_POS)		/** Data byte 0 */
#define CAN_TMDATA11_DB7_POS								0x18UL		/** Data byte 7 */
#define CAN_TMDATA11_DB7_MSK								(0xFFUL << CAN_TMDATA11_DB7_POS)		/** Data byte 7 */
#define CAN_TMDATA11_DB6_POS								0x10UL		/** Data byte 6 */
#define CAN_TMDATA11_DB6_MSK								(0xFFUL << CAN_TMDATA11_DB6_POS)		/** Data byte 6 */
#define CAN_TMDATA11_DB5_POS								0x08UL		/** Data byte 5 */
#define CAN_TMDATA11_DB5_MSK								(0xFFUL << CAN_TMDATA11_DB5_POS)		/** Data byte 5 */
#define CAN_TMDATA11_DB4_POS								0x00UL		/** Data byte 4 */
#define CAN_TMDATA11_DB4_MSK								(0xFFUL << CAN_TMDATA11_DB4_POS)		/** Data byte 4 */
#define CAN_TMI2_SFID_EFID_POS								0x15UL		/** The frame identifier */
#define CAN_TMI2_SFID_EFID_MSK								(0x7FFUL << CAN_TMI2_SFID_EFID_POS)		/** The frame identifier */
#define CAN_TMI2_EFID_POS								0x03UL		/** The frame identifier */
#define CAN_TMI2_EFID_MSK								(0x3FFFFUL << CAN_TMI2_EFID_POS)		/** The frame identifier */
#define CAN_TMI2_FF_POS								0x02UL		/** Frame format */
#define CAN_TMI2_FF_MSK								(0x01UL << CAN_TMI2_FF_POS)		/** Frame format */
#define CAN_TMI2_FT_POS								0x01UL		/** Frame type */
#define CAN_TMI2_FT_MSK								(0x01UL << CAN_TMI2_FT_POS)		/** Frame type */
#define CAN_TMI2_TEN_POS								0x00UL		/** Transmit enable */
#define CAN_TMI2_TEN_MSK								(0x01UL << CAN_TMI2_TEN_POS)		/** Transmit enable */
#define CAN_TMP2_TS_POS								0x10UL		/** Time stamp */
#define CAN_TMP2_TS_MSK								(0xFFFFUL << CAN_TMP2_TS_POS)		/** Time stamp */
#define CAN_TMP2_TSEN_POS								0x08UL		/** Time stamp enable */
#define CAN_TMP2_TSEN_MSK								(0x01UL << CAN_TMP2_TSEN_POS)		/** Time stamp enable */
#define CAN_TMP2_DLENC_POS								0x00UL		/** Data length code */
#define CAN_TMP2_DLENC_MSK								(0x0FUL << CAN_TMP2_DLENC_POS)		/** Data length code */
#define CAN_TMDATA02_DB3_POS								0x18UL		/** Data byte 3 */
#define CAN_TMDATA02_DB3_MSK								(0xFFUL << CAN_TMDATA02_DB3_POS)		/** Data byte 3 */
#define CAN_TMDATA02_DB2_POS								0x10UL		/** Data byte 2 */
#define CAN_TMDATA02_DB2_MSK								(0xFFUL << CAN_TMDATA02_DB2_POS)		/** Data byte 2 */
#define CAN_TMDATA02_DB1_POS								0x08UL		/** Data byte 1 */
#define CAN_TMDATA02_DB1_MSK								(0xFFUL << CAN_TMDATA02_DB1_POS)		/** Data byte 1 */
#define CAN_TMDATA02_DB0_POS								0x00UL		/** Data byte 0 */
#define CAN_TMDATA02_DB0_MSK								(0xFFUL << CAN_TMDATA02_DB0_POS)		/** Data byte 0 */
#define CAN_TMDATA12_DB7_POS								0x18UL		/** Data byte 7 */
#define CAN_TMDATA12_DB7_MSK								(0xFFUL << CAN_TMDATA12_DB7_POS)		/** Data byte 7 */
#define CAN_TMDATA12_DB6_POS								0x10UL		/** Data byte 6 */
#define CAN_TMDATA12_DB6_MSK								(0xFFUL << CAN_TMDATA12_DB6_POS)		/** Data byte 6 */
#define CAN_TMDATA12_DB5_POS								0x08UL		/** Data byte 5 */
#define CAN_TMDATA12_DB5_MSK								(0xFFUL << CAN_TMDATA12_DB5_POS)		/** Data byte 5 */
#define CAN_TMDATA12_DB4_POS								0x00UL		/** Data byte 4 */
#define CAN_TMDATA12_DB4_MSK								(0xFFUL << CAN_TMDATA12_DB4_POS)		/** Data byte 4 */
#define CAN_RFIFOMI0_SFID_EFID_POS								0x15UL		/** The frame identifier */
#define CAN_RFIFOMI0_SFID_EFID_MSK								(0x7FFUL << CAN_RFIFOMI0_SFID_EFID_POS)		/** The frame identifier */
#define CAN_RFIFOMI0_EFID_POS								0x03UL		/** The frame identifier */
#define CAN_RFIFOMI0_EFID_MSK								(0x3FFFFUL << CAN_RFIFOMI0_EFID_POS)		/** The frame identifier */
#define CAN_RFIFOMI0_FF_POS								0x02UL		/** Frame format */
#define CAN_RFIFOMI0_FF_MSK								(0x01UL << CAN_RFIFOMI0_FF_POS)		/** Frame format */
#define CAN_RFIFOMI0_FT_POS								0x01UL		/** Frame type */
#define CAN_RFIFOMI0_FT_MSK								(0x01UL << CAN_RFIFOMI0_FT_POS)		/** Frame type */
#define CAN_RFIFOMP0_TS_POS								0x10UL		/** Time stamp */
#define CAN_RFIFOMP0_TS_MSK								(0xFFFFUL << CAN_RFIFOMP0_TS_POS)		/** Time stamp */
#define CAN_RFIFOMP0_FI_POS								0x08UL		/** Filtering index */
#define CAN_RFIFOMP0_FI_MSK								(0xFFUL << CAN_RFIFOMP0_FI_POS)		/** Filtering index */
#define CAN_RFIFOMP0_DLENC_POS								0x00UL		/** Data length code */
#define CAN_RFIFOMP0_DLENC_MSK								(0x0FUL << CAN_RFIFOMP0_DLENC_POS)		/** Data length code */
#define CAN_RFIFOMDATA00_DB3_POS								0x18UL		/** Data byte 3 */
#define CAN_RFIFOMDATA00_DB3_MSK								(0xFFUL << CAN_RFIFOMDATA00_DB3_POS)		/** Data byte 3 */
#define CAN_RFIFOMDATA00_DB2_POS								0x10UL		/** Data byte 2 */
#define CAN_RFIFOMDATA00_DB2_MSK								(0xFFUL << CAN_RFIFOMDATA00_DB2_POS)		/** Data byte 2 */
#define CAN_RFIFOMDATA00_DB1_POS								0x08UL		/** Data byte 1 */
#define CAN_RFIFOMDATA00_DB1_MSK								(0xFFUL << CAN_RFIFOMDATA00_DB1_POS)		/** Data byte 1 */
#define CAN_RFIFOMDATA00_DB0_POS								0x00UL		/** Data byte 0 */
#define CAN_RFIFOMDATA00_DB0_MSK								(0xFFUL << CAN_RFIFOMDATA00_DB0_POS)		/** Data byte 0 */
#define CAN_RFIFOMDATA10_DB7_POS								0x18UL		/** Data byte 7 */
#define CAN_RFIFOMDATA10_DB7_MSK								(0xFFUL << CAN_RFIFOMDATA10_DB7_POS)		/** Data byte 7 */
#define CAN_RFIFOMDATA10_DB6_POS								0x10UL		/** Data byte 6 */
#define CAN_RFIFOMDATA10_DB6_MSK								(0xFFUL << CAN_RFIFOMDATA10_DB6_POS)		/** Data byte 6 */
#define CAN_RFIFOMDATA10_DB5_POS								0x08UL		/** Data byte 5 */
#define CAN_RFIFOMDATA10_DB5_MSK								(0xFFUL << CAN_RFIFOMDATA10_DB5_POS)		/** Data byte 5 */
#define CAN_RFIFOMDATA10_DB4_POS								0x00UL		/** Data byte 4 */
#define CAN_RFIFOMDATA10_DB4_MSK								(0xFFUL << CAN_RFIFOMDATA10_DB4_POS)		/** Data byte 4 */
#define CAN_RFIFOMI1_SFID_EFID_POS								0x15UL		/** The frame identifier */
#define CAN_RFIFOMI1_SFID_EFID_MSK								(0x7FFUL << CAN_RFIFOMI1_SFID_EFID_POS)		/** The frame identifier */
#define CAN_RFIFOMI1_EFID_POS								0x03UL		/** The frame identifier */
#define CAN_RFIFOMI1_EFID_MSK								(0x3FFFFUL << CAN_RFIFOMI1_EFID_POS)		/** The frame identifier */
#define CAN_RFIFOMI1_FF_POS								0x02UL		/** Frame format */
#define CAN_RFIFOMI1_FF_MSK								(0x01UL << CAN_RFIFOMI1_FF_POS)		/** Frame format */
#define CAN_RFIFOMI1_FT_POS								0x01UL		/** Frame type */
#define CAN_RFIFOMI1_FT_MSK								(0x01UL << CAN_RFIFOMI1_FT_POS)		/** Frame type */
#define CAN_RFIFOMP1_TS_POS								0x10UL		/** Time stamp */
#define CAN_RFIFOMP1_TS_MSK								(0xFFFFUL << CAN_RFIFOMP1_TS_POS)		/** Time stamp */
#define CAN_RFIFOMP1_FI_POS								0x08UL		/** Filtering index */
#define CAN_RFIFOMP1_FI_MSK								(0xFFUL << CAN_RFIFOMP1_FI_POS)		/** Filtering index */
#define CAN_RFIFOMP1_DLENC_POS								0x00UL		/** Data length code */
#define CAN_RFIFOMP1_DLENC_MSK								(0x0FUL << CAN_RFIFOMP1_DLENC_POS)		/** Data length code */
#define CAN_RFIFOMDATA01_DB3_POS								0x18UL		/** Data byte 3 */
#define CAN_RFIFOMDATA01_DB3_MSK								(0xFFUL << CAN_RFIFOMDATA01_DB3_POS)		/** Data byte 3 */
#define CAN_RFIFOMDATA01_DB2_POS								0x10UL		/** Data byte 2 */
#define CAN_RFIFOMDATA01_DB2_MSK								(0xFFUL << CAN_RFIFOMDATA01_DB2_POS)		/** Data byte 2 */
#define CAN_RFIFOMDATA01_DB1_POS								0x08UL		/** Data byte 1 */
#define CAN_RFIFOMDATA01_DB1_MSK								(0xFFUL << CAN_RFIFOMDATA01_DB1_POS)		/** Data byte 1 */
#define CAN_RFIFOMDATA01_DB0_POS								0x00UL		/** Data byte 0 */
#define CAN_RFIFOMDATA01_DB0_MSK								(0xFFUL << CAN_RFIFOMDATA01_DB0_POS)		/** Data byte 0 */
#define CAN_RFIFOMDATA11_DB7_POS								0x18UL		/** Data byte 7 */
#define CAN_RFIFOMDATA11_DB7_MSK								(0xFFUL << CAN_RFIFOMDATA11_DB7_POS)		/** Data byte 7 */
#define CAN_RFIFOMDATA11_DB6_POS								0x10UL		/** Data byte 6 */
#define CAN_RFIFOMDATA11_DB6_MSK								(0xFFUL << CAN_RFIFOMDATA11_DB6_POS)		/** Data byte 6 */
#define CAN_RFIFOMDATA11_DB5_POS								0x08UL		/** Data byte 5 */
#define CAN_RFIFOMDATA11_DB5_MSK								(0xFFUL << CAN_RFIFOMDATA11_DB5_POS)		/** Data byte 5 */
#define CAN_RFIFOMDATA11_DB4_POS								0x00UL		/** Data byte 4 */
#define CAN_RFIFOMDATA11_DB4_MSK								(0xFFUL << CAN_RFIFOMDATA11_DB4_POS)		/** Data byte 4 */
#define CAN_FCTL_HBC1F_POS								0x08UL		/** Header bank of CAN1 filter */
#define CAN_FCTL_HBC1F_MSK								(0x3FUL << CAN_FCTL_HBC1F_POS)		/** Header bank of CAN1 filter */
#define CAN_FCTL_FLD_POS								0x00UL		/** Filter lock disable */
#define CAN_FCTL_FLD_MSK								(0x01UL << CAN_FCTL_FLD_POS)		/** Filter lock disable */
#define CAN_FMCFG_FMOD27_POS								0x1BUL		/** Filter mode */
#define CAN_FMCFG_FMOD27_MSK								(0x01UL << CAN_FMCFG_FMOD27_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD26_POS								0x1AUL		/** Filter mode */
#define CAN_FMCFG_FMOD26_MSK								(0x01UL << CAN_FMCFG_FMOD26_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD25_POS								0x19UL		/** Filter mode */
#define CAN_FMCFG_FMOD25_MSK								(0x01UL << CAN_FMCFG_FMOD25_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD24_POS								0x18UL		/** Filter mode */
#define CAN_FMCFG_FMOD24_MSK								(0x01UL << CAN_FMCFG_FMOD24_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD23_POS								0x17UL		/** Filter mode */
#define CAN_FMCFG_FMOD23_MSK								(0x01UL << CAN_FMCFG_FMOD23_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD22_POS								0x16UL		/** Filter mode */
#define CAN_FMCFG_FMOD22_MSK								(0x01UL << CAN_FMCFG_FMOD22_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD21_POS								0x15UL		/** Filter mode */
#define CAN_FMCFG_FMOD21_MSK								(0x01UL << CAN_FMCFG_FMOD21_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD20_POS								0x14UL		/** Filter mode */
#define CAN_FMCFG_FMOD20_MSK								(0x01UL << CAN_FMCFG_FMOD20_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD19_POS								0x13UL		/** Filter mode */
#define CAN_FMCFG_FMOD19_MSK								(0x01UL << CAN_FMCFG_FMOD19_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD18_POS								0x12UL		/** Filter mode */
#define CAN_FMCFG_FMOD18_MSK								(0x01UL << CAN_FMCFG_FMOD18_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD17_POS								0x11UL		/** Filter mode */
#define CAN_FMCFG_FMOD17_MSK								(0x01UL << CAN_FMCFG_FMOD17_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD16_POS								0x10UL		/** Filter mode */
#define CAN_FMCFG_FMOD16_MSK								(0x01UL << CAN_FMCFG_FMOD16_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD15_POS								0x0FUL		/** Filter mode */
#define CAN_FMCFG_FMOD15_MSK								(0x01UL << CAN_FMCFG_FMOD15_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD14_POS								0x0EUL		/** Filter mode */
#define CAN_FMCFG_FMOD14_MSK								(0x01UL << CAN_FMCFG_FMOD14_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD13_POS								0x0DUL		/** Filter mode */
#define CAN_FMCFG_FMOD13_MSK								(0x01UL << CAN_FMCFG_FMOD13_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD12_POS								0x0CUL		/** Filter mode */
#define CAN_FMCFG_FMOD12_MSK								(0x01UL << CAN_FMCFG_FMOD12_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD11_POS								0x0BUL		/** Filter mode */
#define CAN_FMCFG_FMOD11_MSK								(0x01UL << CAN_FMCFG_FMOD11_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD10_POS								0x0AUL		/** Filter mode */
#define CAN_FMCFG_FMOD10_MSK								(0x01UL << CAN_FMCFG_FMOD10_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD9_POS								0x09UL		/** Filter mode */
#define CAN_FMCFG_FMOD9_MSK								(0x01UL << CAN_FMCFG_FMOD9_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD8_POS								0x08UL		/** Filter mode */
#define CAN_FMCFG_FMOD8_MSK								(0x01UL << CAN_FMCFG_FMOD8_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD7_POS								0x07UL		/** Filter mode */
#define CAN_FMCFG_FMOD7_MSK								(0x01UL << CAN_FMCFG_FMOD7_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD6_POS								0x06UL		/** Filter mode */
#define CAN_FMCFG_FMOD6_MSK								(0x01UL << CAN_FMCFG_FMOD6_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD5_POS								0x05UL		/** Filter mode */
#define CAN_FMCFG_FMOD5_MSK								(0x01UL << CAN_FMCFG_FMOD5_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD4_POS								0x04UL		/** Filter mode */
#define CAN_FMCFG_FMOD4_MSK								(0x01UL << CAN_FMCFG_FMOD4_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD3_POS								0x03UL		/** Filter mode */
#define CAN_FMCFG_FMOD3_MSK								(0x01UL << CAN_FMCFG_FMOD3_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD2_POS								0x02UL		/** Filter mode */
#define CAN_FMCFG_FMOD2_MSK								(0x01UL << CAN_FMCFG_FMOD2_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD1_POS								0x01UL		/** Filter mode */
#define CAN_FMCFG_FMOD1_MSK								(0x01UL << CAN_FMCFG_FMOD1_POS)		/** Filter mode */
#define CAN_FMCFG_FMOD0_POS								0x00UL		/** Filter mode */
#define CAN_FMCFG_FMOD0_MSK								(0x01UL << CAN_FMCFG_FMOD0_POS)		/** Filter mode */
#define CAN_FSCFG_FS0_POS								0x00UL		/** Filter scale configuration */
#define CAN_FSCFG_FS0_MSK								(0x01UL << CAN_FSCFG_FS0_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS1_POS								0x01UL		/** Filter scale configuration */
#define CAN_FSCFG_FS1_MSK								(0x01UL << CAN_FSCFG_FS1_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS2_POS								0x02UL		/** Filter scale configuration */
#define CAN_FSCFG_FS2_MSK								(0x01UL << CAN_FSCFG_FS2_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS3_POS								0x03UL		/** Filter scale configuration */
#define CAN_FSCFG_FS3_MSK								(0x01UL << CAN_FSCFG_FS3_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS4_POS								0x04UL		/** Filter scale configuration */
#define CAN_FSCFG_FS4_MSK								(0x01UL << CAN_FSCFG_FS4_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS5_POS								0x05UL		/** Filter scale configuration */
#define CAN_FSCFG_FS5_MSK								(0x01UL << CAN_FSCFG_FS5_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS6_POS								0x06UL		/** Filter scale configuration */
#define CAN_FSCFG_FS6_MSK								(0x01UL << CAN_FSCFG_FS6_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS7_POS								0x07UL		/** Filter scale configuration */
#define CAN_FSCFG_FS7_MSK								(0x01UL << CAN_FSCFG_FS7_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS8_POS								0x08UL		/** Filter scale configuration */
#define CAN_FSCFG_FS8_MSK								(0x01UL << CAN_FSCFG_FS8_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS9_POS								0x09UL		/** Filter scale configuration */
#define CAN_FSCFG_FS9_MSK								(0x01UL << CAN_FSCFG_FS9_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS10_POS								0x0AUL		/** Filter scale configuration */
#define CAN_FSCFG_FS10_MSK								(0x01UL << CAN_FSCFG_FS10_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS11_POS								0x0BUL		/** Filter scale configuration */
#define CAN_FSCFG_FS11_MSK								(0x01UL << CAN_FSCFG_FS11_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS12_POS								0x0CUL		/** Filter scale configuration */
#define CAN_FSCFG_FS12_MSK								(0x01UL << CAN_FSCFG_FS12_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS13_POS								0x0DUL		/** Filter scale configuration */
#define CAN_FSCFG_FS13_MSK								(0x01UL << CAN_FSCFG_FS13_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS14_POS								0x0EUL		/** Filter scale configuration */
#define CAN_FSCFG_FS14_MSK								(0x01UL << CAN_FSCFG_FS14_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS15_POS								0x0FUL		/** Filter scale configuration */
#define CAN_FSCFG_FS15_MSK								(0x01UL << CAN_FSCFG_FS15_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS16_POS								0x10UL		/** Filter scale configuration */
#define CAN_FSCFG_FS16_MSK								(0x01UL << CAN_FSCFG_FS16_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS17_POS								0x11UL		/** Filter scale configuration */
#define CAN_FSCFG_FS17_MSK								(0x01UL << CAN_FSCFG_FS17_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS18_POS								0x12UL		/** Filter scale configuration */
#define CAN_FSCFG_FS18_MSK								(0x01UL << CAN_FSCFG_FS18_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS19_POS								0x13UL		/** Filter scale configuration */
#define CAN_FSCFG_FS19_MSK								(0x01UL << CAN_FSCFG_FS19_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS20_POS								0x14UL		/** Filter scale configuration */
#define CAN_FSCFG_FS20_MSK								(0x01UL << CAN_FSCFG_FS20_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS21_POS								0x15UL		/** Filter scale configuration */
#define CAN_FSCFG_FS21_MSK								(0x01UL << CAN_FSCFG_FS21_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS22_POS								0x16UL		/** Filter scale configuration */
#define CAN_FSCFG_FS22_MSK								(0x01UL << CAN_FSCFG_FS22_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS23_POS								0x17UL		/** Filter scale configuration */
#define CAN_FSCFG_FS23_MSK								(0x01UL << CAN_FSCFG_FS23_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS24_POS								0x18UL		/** Filter scale configuration */
#define CAN_FSCFG_FS24_MSK								(0x01UL << CAN_FSCFG_FS24_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS25_POS								0x19UL		/** Filter scale configuration */
#define CAN_FSCFG_FS25_MSK								(0x01UL << CAN_FSCFG_FS25_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS26_POS								0x1AUL		/** Filter scale configuration */
#define CAN_FSCFG_FS26_MSK								(0x01UL << CAN_FSCFG_FS26_POS)		/** Filter scale configuration */
#define CAN_FSCFG_FS27_POS								0x1BUL		/** Filter scale configuration */
#define CAN_FSCFG_FS27_MSK								(0x01UL << CAN_FSCFG_FS27_POS)		/** Filter scale configuration */
#define CAN_FAFIFO_FAF0_POS								0x00UL		/** Filter 0 associated with FIFO */
#define CAN_FAFIFO_FAF0_MSK								(0x01UL << CAN_FAFIFO_FAF0_POS)		/** Filter 0 associated with FIFO */
#define CAN_FAFIFO_FAF1_POS								0x01UL		/** Filter 1 associated with FIFO */
#define CAN_FAFIFO_FAF1_MSK								(0x01UL << CAN_FAFIFO_FAF1_POS)		/** Filter 1 associated with FIFO */
#define CAN_FAFIFO_FAF2_POS								0x02UL		/** Filter 2 associated with FIFO */
#define CAN_FAFIFO_FAF2_MSK								(0x01UL << CAN_FAFIFO_FAF2_POS)		/** Filter 2 associated with FIFO */
#define CAN_FAFIFO_FAF3_POS								0x03UL		/** Filter 3 associated with FIFO */
#define CAN_FAFIFO_FAF3_MSK								(0x01UL << CAN_FAFIFO_FAF3_POS)		/** Filter 3 associated with FIFO */
#define CAN_FAFIFO_FAF4_POS								0x04UL		/** Filter 4 associated with FIFO */
#define CAN_FAFIFO_FAF4_MSK								(0x01UL << CAN_FAFIFO_FAF4_POS)		/** Filter 4 associated with FIFO */
#define CAN_FAFIFO_FAF5_POS								0x05UL		/** Filter 5 associated with FIFO */
#define CAN_FAFIFO_FAF5_MSK								(0x01UL << CAN_FAFIFO_FAF5_POS)		/** Filter 5 associated with FIFO */
#define CAN_FAFIFO_FAF6_POS								0x06UL		/** Filter 6 associated with FIFO */
#define CAN_FAFIFO_FAF6_MSK								(0x01UL << CAN_FAFIFO_FAF6_POS)		/** Filter 6 associated with FIFO */
#define CAN_FAFIFO_FAF7_POS								0x07UL		/** Filter 7 associated with FIFO */
#define CAN_FAFIFO_FAF7_MSK								(0x01UL << CAN_FAFIFO_FAF7_POS)		/** Filter 7 associated with FIFO */
#define CAN_FAFIFO_FAF8_POS								0x08UL		/** Filter 8 associated with FIFO */
#define CAN_FAFIFO_FAF8_MSK								(0x01UL << CAN_FAFIFO_FAF8_POS)		/** Filter 8 associated with FIFO */
#define CAN_FAFIFO_FAF9_POS								0x09UL		/** Filter 9 associated with FIFO */
#define CAN_FAFIFO_FAF9_MSK								(0x01UL << CAN_FAFIFO_FAF9_POS)		/** Filter 9 associated with FIFO */
#define CAN_FAFIFO_FAF10_POS								0x0AUL		/** Filter 10 associated with FIFO */
#define CAN_FAFIFO_FAF10_MSK								(0x01UL << CAN_FAFIFO_FAF10_POS)		/** Filter 10 associated with FIFO */
#define CAN_FAFIFO_FAF11_POS								0x0BUL		/** Filter 11 associated with FIFO */
#define CAN_FAFIFO_FAF11_MSK								(0x01UL << CAN_FAFIFO_FAF11_POS)		/** Filter 11 associated with FIFO */
#define CAN_FAFIFO_FAF12_POS								0x0CUL		/** Filter 12 associated with FIFO */
#define CAN_FAFIFO_FAF12_MSK								(0x01UL << CAN_FAFIFO_FAF12_POS)		/** Filter 12 associated with FIFO */
#define CAN_FAFIFO_FAF13_POS								0x0DUL		/** Filter 13 associated with FIFO */
#define CAN_FAFIFO_FAF13_MSK								(0x01UL << CAN_FAFIFO_FAF13_POS)		/** Filter 13 associated with FIFO */
#define CAN_FAFIFO_FAF14_POS								0x0EUL		/** Filter 14 associated with FIFO */
#define CAN_FAFIFO_FAF14_MSK								(0x01UL << CAN_FAFIFO_FAF14_POS)		/** Filter 14 associated with FIFO */
#define CAN_FAFIFO_FAF15_POS								0x0FUL		/** Filter 15 associated with FIFO */
#define CAN_FAFIFO_FAF15_MSK								(0x01UL << CAN_FAFIFO_FAF15_POS)		/** Filter 15 associated with FIFO */
#define CAN_FAFIFO_FAF16_POS								0x10UL		/** Filter 16 associated with FIFO */
#define CAN_FAFIFO_FAF16_MSK								(0x01UL << CAN_FAFIFO_FAF16_POS)		/** Filter 16 associated with FIFO */
#define CAN_FAFIFO_FAF17_POS								0x11UL		/** Filter 17 associated with FIFO */
#define CAN_FAFIFO_FAF17_MSK								(0x01UL << CAN_FAFIFO_FAF17_POS)		/** Filter 17 associated with FIFO */
#define CAN_FAFIFO_FAF18_POS								0x12UL		/** Filter 18 associated with FIFO */
#define CAN_FAFIFO_FAF18_MSK								(0x01UL << CAN_FAFIFO_FAF18_POS)		/** Filter 18 associated with FIFO */
#define CAN_FAFIFO_FAF19_POS								0x13UL		/** Filter 19 associated with FIFO */
#define CAN_FAFIFO_FAF19_MSK								(0x01UL << CAN_FAFIFO_FAF19_POS)		/** Filter 19 associated with FIFO */
#define CAN_FAFIFO_FAF20_POS								0x14UL		/** Filter 20 associated with FIFO */
#define CAN_FAFIFO_FAF20_MSK								(0x01UL << CAN_FAFIFO_FAF20_POS)		/** Filter 20 associated with FIFO */
#define CAN_FAFIFO_FAF21_POS								0x15UL		/** Filter 21 associated with FIFO */
#define CAN_FAFIFO_FAF21_MSK								(0x01UL << CAN_FAFIFO_FAF21_POS)		/** Filter 21 associated with FIFO */
#define CAN_FAFIFO_FAF22_POS								0x16UL		/** Filter 22 associated with FIFO */
#define CAN_FAFIFO_FAF22_MSK								(0x01UL << CAN_FAFIFO_FAF22_POS)		/** Filter 22 associated with FIFO */
#define CAN_FAFIFO_FAF23_POS								0x17UL		/** Filter 23 associated with FIFO */
#define CAN_FAFIFO_FAF23_MSK								(0x01UL << CAN_FAFIFO_FAF23_POS)		/** Filter 23 associated with FIFO */
#define CAN_FAFIFO_FAF24_POS								0x18UL		/** Filter 24 associated with FIFO */
#define CAN_FAFIFO_FAF24_MSK								(0x01UL << CAN_FAFIFO_FAF24_POS)		/** Filter 24 associated with FIFO */
#define CAN_FAFIFO_FAF25_POS								0x19UL		/** Filter 25 associated with FIFO */
#define CAN_FAFIFO_FAF25_MSK								(0x01UL << CAN_FAFIFO_FAF25_POS)		/** Filter 25 associated with FIFO */
#define CAN_FAFIFO_FAF26_POS								0x1AUL		/** Filter 26 associated with FIFO */
#define CAN_FAFIFO_FAF26_MSK								(0x01UL << CAN_FAFIFO_FAF26_POS)		/** Filter 26 associated with FIFO */
#define CAN_FAFIFO_FAF27_POS								0x1BUL		/** Filter 27 associated with FIFO */
#define CAN_FAFIFO_FAF27_MSK								(0x01UL << CAN_FAFIFO_FAF27_POS)		/** Filter 27 associated with FIFO */
#define CAN_FW_FW0_POS								0x00UL		/** Filter working */
#define CAN_FW_FW0_MSK								(0x01UL << CAN_FW_FW0_POS)		/** Filter working */
#define CAN_FW_FW1_POS								0x01UL		/** Filter working */
#define CAN_FW_FW1_MSK								(0x01UL << CAN_FW_FW1_POS)		/** Filter working */
#define CAN_FW_FW2_POS								0x02UL		/** Filter working */
#define CAN_FW_FW2_MSK								(0x01UL << CAN_FW_FW2_POS)		/** Filter working */
#define CAN_FW_FW3_POS								0x03UL		/** Filter working */
#define CAN_FW_FW3_MSK								(0x01UL << CAN_FW_FW3_POS)		/** Filter working */
#define CAN_FW_FW4_POS								0x04UL		/** Filter working */
#define CAN_FW_FW4_MSK								(0x01UL << CAN_FW_FW4_POS)		/** Filter working */
#define CAN_FW_FW5_POS								0x05UL		/** Filter working */
#define CAN_FW_FW5_MSK								(0x01UL << CAN_FW_FW5_POS)		/** Filter working */
#define CAN_FW_FW6_POS								0x06UL		/** Filter working */
#define CAN_FW_FW6_MSK								(0x01UL << CAN_FW_FW6_POS)		/** Filter working */
#define CAN_FW_FW7_POS								0x07UL		/** Filter working */
#define CAN_FW_FW7_MSK								(0x01UL << CAN_FW_FW7_POS)		/** Filter working */
#define CAN_FW_FW8_POS								0x08UL		/** Filter working */
#define CAN_FW_FW8_MSK								(0x01UL << CAN_FW_FW8_POS)		/** Filter working */
#define CAN_FW_FW9_POS								0x09UL		/** Filter working */
#define CAN_FW_FW9_MSK								(0x01UL << CAN_FW_FW9_POS)		/** Filter working */
#define CAN_FW_FW10_POS								0x0AUL		/** Filter working */
#define CAN_FW_FW10_MSK								(0x01UL << CAN_FW_FW10_POS)		/** Filter working */
#define CAN_FW_FW11_POS								0x0BUL		/** Filter working */
#define CAN_FW_FW11_MSK								(0x01UL << CAN_FW_FW11_POS)		/** Filter working */
#define CAN_FW_FW12_POS								0x0CUL		/** Filter working */
#define CAN_FW_FW12_MSK								(0x01UL << CAN_FW_FW12_POS)		/** Filter working */
#define CAN_FW_FW13_POS								0x0DUL		/** Filter working */
#define CAN_FW_FW13_MSK								(0x01UL << CAN_FW_FW13_POS)		/** Filter working */
#define CAN_FW_FW14_POS								0x0EUL		/** Filter working */
#define CAN_FW_FW14_MSK								(0x01UL << CAN_FW_FW14_POS)		/** Filter working */
#define CAN_FW_FW15_POS								0x0FUL		/** Filter working */
#define CAN_FW_FW15_MSK								(0x01UL << CAN_FW_FW15_POS)		/** Filter working */
#define CAN_FW_FW16_POS								0x10UL		/** Filter working */
#define CAN_FW_FW16_MSK								(0x01UL << CAN_FW_FW16_POS)		/** Filter working */
#define CAN_FW_FW17_POS								0x11UL		/** Filter working */
#define CAN_FW_FW17_MSK								(0x01UL << CAN_FW_FW17_POS)		/** Filter working */
#define CAN_FW_FW18_POS								0x12UL		/** Filter working */
#define CAN_FW_FW18_MSK								(0x01UL << CAN_FW_FW18_POS)		/** Filter working */
#define CAN_FW_FW19_POS								0x13UL		/** Filter working */
#define CAN_FW_FW19_MSK								(0x01UL << CAN_FW_FW19_POS)		/** Filter working */
#define CAN_FW_FW20_POS								0x14UL		/** Filter working */
#define CAN_FW_FW20_MSK								(0x01UL << CAN_FW_FW20_POS)		/** Filter working */
#define CAN_FW_FW21_POS								0x15UL		/** Filter working */
#define CAN_FW_FW21_MSK								(0x01UL << CAN_FW_FW21_POS)		/** Filter working */
#define CAN_FW_FW22_POS								0x16UL		/** Filter working */
#define CAN_FW_FW22_MSK								(0x01UL << CAN_FW_FW22_POS)		/** Filter working */
#define CAN_FW_FW23_POS								0x17UL		/** Filter working */
#define CAN_FW_FW23_MSK								(0x01UL << CAN_FW_FW23_POS)		/** Filter working */
#define CAN_FW_FW24_POS								0x18UL		/** Filter working */
#define CAN_FW_FW24_MSK								(0x01UL << CAN_FW_FW24_POS)		/** Filter working */
#define CAN_FW_FW25_POS								0x19UL		/** Filter working */
#define CAN_FW_FW25_MSK								(0x01UL << CAN_FW_FW25_POS)		/** Filter working */
#define CAN_FW_FW26_POS								0x1AUL		/** Filter working */
#define CAN_FW_FW26_MSK								(0x01UL << CAN_FW_FW26_POS)		/** Filter working */
#define CAN_FW_FW27_POS								0x1BUL		/** Filter working */
#define CAN_FW_FW27_MSK								(0x01UL << CAN_FW_FW27_POS)		/** Filter working */
#define CAN_F0DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F0DATA0_FD0_MSK								(0x01UL << CAN_F0DATA0_FD0_POS)		/** Filter bits */
#define CAN_F0DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F0DATA0_FD1_MSK								(0x01UL << CAN_F0DATA0_FD1_POS)		/** Filter bits */
#define CAN_F0DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F0DATA0_FD2_MSK								(0x01UL << CAN_F0DATA0_FD2_POS)		/** Filter bits */
#define CAN_F0DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F0DATA0_FD3_MSK								(0x01UL << CAN_F0DATA0_FD3_POS)		/** Filter bits */
#define CAN_F0DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F0DATA0_FD4_MSK								(0x01UL << CAN_F0DATA0_FD4_POS)		/** Filter bits */
#define CAN_F0DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F0DATA0_FD5_MSK								(0x01UL << CAN_F0DATA0_FD5_POS)		/** Filter bits */
#define CAN_F0DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F0DATA0_FD6_MSK								(0x01UL << CAN_F0DATA0_FD6_POS)		/** Filter bits */
#define CAN_F0DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F0DATA0_FD7_MSK								(0x01UL << CAN_F0DATA0_FD7_POS)		/** Filter bits */
#define CAN_F0DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F0DATA0_FD8_MSK								(0x01UL << CAN_F0DATA0_FD8_POS)		/** Filter bits */
#define CAN_F0DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F0DATA0_FD9_MSK								(0x01UL << CAN_F0DATA0_FD9_POS)		/** Filter bits */
#define CAN_F0DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F0DATA0_FD10_MSK								(0x01UL << CAN_F0DATA0_FD10_POS)		/** Filter bits */
#define CAN_F0DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F0DATA0_FD11_MSK								(0x01UL << CAN_F0DATA0_FD11_POS)		/** Filter bits */
#define CAN_F0DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F0DATA0_FD12_MSK								(0x01UL << CAN_F0DATA0_FD12_POS)		/** Filter bits */
#define CAN_F0DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F0DATA0_FD13_MSK								(0x01UL << CAN_F0DATA0_FD13_POS)		/** Filter bits */
#define CAN_F0DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F0DATA0_FD14_MSK								(0x01UL << CAN_F0DATA0_FD14_POS)		/** Filter bits */
#define CAN_F0DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F0DATA0_FD15_MSK								(0x01UL << CAN_F0DATA0_FD15_POS)		/** Filter bits */
#define CAN_F0DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F0DATA0_FD16_MSK								(0x01UL << CAN_F0DATA0_FD16_POS)		/** Filter bits */
#define CAN_F0DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F0DATA0_FD17_MSK								(0x01UL << CAN_F0DATA0_FD17_POS)		/** Filter bits */
#define CAN_F0DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F0DATA0_FD18_MSK								(0x01UL << CAN_F0DATA0_FD18_POS)		/** Filter bits */
#define CAN_F0DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F0DATA0_FD19_MSK								(0x01UL << CAN_F0DATA0_FD19_POS)		/** Filter bits */
#define CAN_F0DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F0DATA0_FD20_MSK								(0x01UL << CAN_F0DATA0_FD20_POS)		/** Filter bits */
#define CAN_F0DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F0DATA0_FD21_MSK								(0x01UL << CAN_F0DATA0_FD21_POS)		/** Filter bits */
#define CAN_F0DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F0DATA0_FD22_MSK								(0x01UL << CAN_F0DATA0_FD22_POS)		/** Filter bits */
#define CAN_F0DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F0DATA0_FD23_MSK								(0x01UL << CAN_F0DATA0_FD23_POS)		/** Filter bits */
#define CAN_F0DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F0DATA0_FD24_MSK								(0x01UL << CAN_F0DATA0_FD24_POS)		/** Filter bits */
#define CAN_F0DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F0DATA0_FD25_MSK								(0x01UL << CAN_F0DATA0_FD25_POS)		/** Filter bits */
#define CAN_F0DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F0DATA0_FD26_MSK								(0x01UL << CAN_F0DATA0_FD26_POS)		/** Filter bits */
#define CAN_F0DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F0DATA0_FD27_MSK								(0x01UL << CAN_F0DATA0_FD27_POS)		/** Filter bits */
#define CAN_F0DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F0DATA0_FD28_MSK								(0x01UL << CAN_F0DATA0_FD28_POS)		/** Filter bits */
#define CAN_F0DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F0DATA0_FD29_MSK								(0x01UL << CAN_F0DATA0_FD29_POS)		/** Filter bits */
#define CAN_F0DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F0DATA0_FD30_MSK								(0x01UL << CAN_F0DATA0_FD30_POS)		/** Filter bits */
#define CAN_F0DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F0DATA0_FD31_MSK								(0x01UL << CAN_F0DATA0_FD31_POS)		/** Filter bits */
#define CAN_F0DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F0DATA1_FD0_MSK								(0x01UL << CAN_F0DATA1_FD0_POS)		/** Filter bits */
#define CAN_F0DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F0DATA1_FD1_MSK								(0x01UL << CAN_F0DATA1_FD1_POS)		/** Filter bits */
#define CAN_F0DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F0DATA1_FD2_MSK								(0x01UL << CAN_F0DATA1_FD2_POS)		/** Filter bits */
#define CAN_F0DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F0DATA1_FD3_MSK								(0x01UL << CAN_F0DATA1_FD3_POS)		/** Filter bits */
#define CAN_F0DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F0DATA1_FD4_MSK								(0x01UL << CAN_F0DATA1_FD4_POS)		/** Filter bits */
#define CAN_F0DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F0DATA1_FD5_MSK								(0x01UL << CAN_F0DATA1_FD5_POS)		/** Filter bits */
#define CAN_F0DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F0DATA1_FD6_MSK								(0x01UL << CAN_F0DATA1_FD6_POS)		/** Filter bits */
#define CAN_F0DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F0DATA1_FD7_MSK								(0x01UL << CAN_F0DATA1_FD7_POS)		/** Filter bits */
#define CAN_F0DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F0DATA1_FD8_MSK								(0x01UL << CAN_F0DATA1_FD8_POS)		/** Filter bits */
#define CAN_F0DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F0DATA1_FD9_MSK								(0x01UL << CAN_F0DATA1_FD9_POS)		/** Filter bits */
#define CAN_F0DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F0DATA1_FD10_MSK								(0x01UL << CAN_F0DATA1_FD10_POS)		/** Filter bits */
#define CAN_F0DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F0DATA1_FD11_MSK								(0x01UL << CAN_F0DATA1_FD11_POS)		/** Filter bits */
#define CAN_F0DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F0DATA1_FD12_MSK								(0x01UL << CAN_F0DATA1_FD12_POS)		/** Filter bits */
#define CAN_F0DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F0DATA1_FD13_MSK								(0x01UL << CAN_F0DATA1_FD13_POS)		/** Filter bits */
#define CAN_F0DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F0DATA1_FD14_MSK								(0x01UL << CAN_F0DATA1_FD14_POS)		/** Filter bits */
#define CAN_F0DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F0DATA1_FD15_MSK								(0x01UL << CAN_F0DATA1_FD15_POS)		/** Filter bits */
#define CAN_F0DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F0DATA1_FD16_MSK								(0x01UL << CAN_F0DATA1_FD16_POS)		/** Filter bits */
#define CAN_F0DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F0DATA1_FD17_MSK								(0x01UL << CAN_F0DATA1_FD17_POS)		/** Filter bits */
#define CAN_F0DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F0DATA1_FD18_MSK								(0x01UL << CAN_F0DATA1_FD18_POS)		/** Filter bits */
#define CAN_F0DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F0DATA1_FD19_MSK								(0x01UL << CAN_F0DATA1_FD19_POS)		/** Filter bits */
#define CAN_F0DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F0DATA1_FD20_MSK								(0x01UL << CAN_F0DATA1_FD20_POS)		/** Filter bits */
#define CAN_F0DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F0DATA1_FD21_MSK								(0x01UL << CAN_F0DATA1_FD21_POS)		/** Filter bits */
#define CAN_F0DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F0DATA1_FD22_MSK								(0x01UL << CAN_F0DATA1_FD22_POS)		/** Filter bits */
#define CAN_F0DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F0DATA1_FD23_MSK								(0x01UL << CAN_F0DATA1_FD23_POS)		/** Filter bits */
#define CAN_F0DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F0DATA1_FD24_MSK								(0x01UL << CAN_F0DATA1_FD24_POS)		/** Filter bits */
#define CAN_F0DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F0DATA1_FD25_MSK								(0x01UL << CAN_F0DATA1_FD25_POS)		/** Filter bits */
#define CAN_F0DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F0DATA1_FD26_MSK								(0x01UL << CAN_F0DATA1_FD26_POS)		/** Filter bits */
#define CAN_F0DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F0DATA1_FD27_MSK								(0x01UL << CAN_F0DATA1_FD27_POS)		/** Filter bits */
#define CAN_F0DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F0DATA1_FD28_MSK								(0x01UL << CAN_F0DATA1_FD28_POS)		/** Filter bits */
#define CAN_F0DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F0DATA1_FD29_MSK								(0x01UL << CAN_F0DATA1_FD29_POS)		/** Filter bits */
#define CAN_F0DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F0DATA1_FD30_MSK								(0x01UL << CAN_F0DATA1_FD30_POS)		/** Filter bits */
#define CAN_F0DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F0DATA1_FD31_MSK								(0x01UL << CAN_F0DATA1_FD31_POS)		/** Filter bits */
#define CAN_F1DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F1DATA0_FD0_MSK								(0x01UL << CAN_F1DATA0_FD0_POS)		/** Filter bits */
#define CAN_F1DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F1DATA0_FD1_MSK								(0x01UL << CAN_F1DATA0_FD1_POS)		/** Filter bits */
#define CAN_F1DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F1DATA0_FD2_MSK								(0x01UL << CAN_F1DATA0_FD2_POS)		/** Filter bits */
#define CAN_F1DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F1DATA0_FD3_MSK								(0x01UL << CAN_F1DATA0_FD3_POS)		/** Filter bits */
#define CAN_F1DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F1DATA0_FD4_MSK								(0x01UL << CAN_F1DATA0_FD4_POS)		/** Filter bits */
#define CAN_F1DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F1DATA0_FD5_MSK								(0x01UL << CAN_F1DATA0_FD5_POS)		/** Filter bits */
#define CAN_F1DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F1DATA0_FD6_MSK								(0x01UL << CAN_F1DATA0_FD6_POS)		/** Filter bits */
#define CAN_F1DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F1DATA0_FD7_MSK								(0x01UL << CAN_F1DATA0_FD7_POS)		/** Filter bits */
#define CAN_F1DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F1DATA0_FD8_MSK								(0x01UL << CAN_F1DATA0_FD8_POS)		/** Filter bits */
#define CAN_F1DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F1DATA0_FD9_MSK								(0x01UL << CAN_F1DATA0_FD9_POS)		/** Filter bits */
#define CAN_F1DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F1DATA0_FD10_MSK								(0x01UL << CAN_F1DATA0_FD10_POS)		/** Filter bits */
#define CAN_F1DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F1DATA0_FD11_MSK								(0x01UL << CAN_F1DATA0_FD11_POS)		/** Filter bits */
#define CAN_F1DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F1DATA0_FD12_MSK								(0x01UL << CAN_F1DATA0_FD12_POS)		/** Filter bits */
#define CAN_F1DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F1DATA0_FD13_MSK								(0x01UL << CAN_F1DATA0_FD13_POS)		/** Filter bits */
#define CAN_F1DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F1DATA0_FD14_MSK								(0x01UL << CAN_F1DATA0_FD14_POS)		/** Filter bits */
#define CAN_F1DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F1DATA0_FD15_MSK								(0x01UL << CAN_F1DATA0_FD15_POS)		/** Filter bits */
#define CAN_F1DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F1DATA0_FD16_MSK								(0x01UL << CAN_F1DATA0_FD16_POS)		/** Filter bits */
#define CAN_F1DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F1DATA0_FD17_MSK								(0x01UL << CAN_F1DATA0_FD17_POS)		/** Filter bits */
#define CAN_F1DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F1DATA0_FD18_MSK								(0x01UL << CAN_F1DATA0_FD18_POS)		/** Filter bits */
#define CAN_F1DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F1DATA0_FD19_MSK								(0x01UL << CAN_F1DATA0_FD19_POS)		/** Filter bits */
#define CAN_F1DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F1DATA0_FD20_MSK								(0x01UL << CAN_F1DATA0_FD20_POS)		/** Filter bits */
#define CAN_F1DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F1DATA0_FD21_MSK								(0x01UL << CAN_F1DATA0_FD21_POS)		/** Filter bits */
#define CAN_F1DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F1DATA0_FD22_MSK								(0x01UL << CAN_F1DATA0_FD22_POS)		/** Filter bits */
#define CAN_F1DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F1DATA0_FD23_MSK								(0x01UL << CAN_F1DATA0_FD23_POS)		/** Filter bits */
#define CAN_F1DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F1DATA0_FD24_MSK								(0x01UL << CAN_F1DATA0_FD24_POS)		/** Filter bits */
#define CAN_F1DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F1DATA0_FD25_MSK								(0x01UL << CAN_F1DATA0_FD25_POS)		/** Filter bits */
#define CAN_F1DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F1DATA0_FD26_MSK								(0x01UL << CAN_F1DATA0_FD26_POS)		/** Filter bits */
#define CAN_F1DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F1DATA0_FD27_MSK								(0x01UL << CAN_F1DATA0_FD27_POS)		/** Filter bits */
#define CAN_F1DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F1DATA0_FD28_MSK								(0x01UL << CAN_F1DATA0_FD28_POS)		/** Filter bits */
#define CAN_F1DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F1DATA0_FD29_MSK								(0x01UL << CAN_F1DATA0_FD29_POS)		/** Filter bits */
#define CAN_F1DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F1DATA0_FD30_MSK								(0x01UL << CAN_F1DATA0_FD30_POS)		/** Filter bits */
#define CAN_F1DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F1DATA0_FD31_MSK								(0x01UL << CAN_F1DATA0_FD31_POS)		/** Filter bits */
#define CAN_F1DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F1DATA1_FD0_MSK								(0x01UL << CAN_F1DATA1_FD0_POS)		/** Filter bits */
#define CAN_F1DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F1DATA1_FD1_MSK								(0x01UL << CAN_F1DATA1_FD1_POS)		/** Filter bits */
#define CAN_F1DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F1DATA1_FD2_MSK								(0x01UL << CAN_F1DATA1_FD2_POS)		/** Filter bits */
#define CAN_F1DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F1DATA1_FD3_MSK								(0x01UL << CAN_F1DATA1_FD3_POS)		/** Filter bits */
#define CAN_F1DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F1DATA1_FD4_MSK								(0x01UL << CAN_F1DATA1_FD4_POS)		/** Filter bits */
#define CAN_F1DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F1DATA1_FD5_MSK								(0x01UL << CAN_F1DATA1_FD5_POS)		/** Filter bits */
#define CAN_F1DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F1DATA1_FD6_MSK								(0x01UL << CAN_F1DATA1_FD6_POS)		/** Filter bits */
#define CAN_F1DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F1DATA1_FD7_MSK								(0x01UL << CAN_F1DATA1_FD7_POS)		/** Filter bits */
#define CAN_F1DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F1DATA1_FD8_MSK								(0x01UL << CAN_F1DATA1_FD8_POS)		/** Filter bits */
#define CAN_F1DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F1DATA1_FD9_MSK								(0x01UL << CAN_F1DATA1_FD9_POS)		/** Filter bits */
#define CAN_F1DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F1DATA1_FD10_MSK								(0x01UL << CAN_F1DATA1_FD10_POS)		/** Filter bits */
#define CAN_F1DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F1DATA1_FD11_MSK								(0x01UL << CAN_F1DATA1_FD11_POS)		/** Filter bits */
#define CAN_F1DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F1DATA1_FD12_MSK								(0x01UL << CAN_F1DATA1_FD12_POS)		/** Filter bits */
#define CAN_F1DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F1DATA1_FD13_MSK								(0x01UL << CAN_F1DATA1_FD13_POS)		/** Filter bits */
#define CAN_F1DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F1DATA1_FD14_MSK								(0x01UL << CAN_F1DATA1_FD14_POS)		/** Filter bits */
#define CAN_F1DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F1DATA1_FD15_MSK								(0x01UL << CAN_F1DATA1_FD15_POS)		/** Filter bits */
#define CAN_F1DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F1DATA1_FD16_MSK								(0x01UL << CAN_F1DATA1_FD16_POS)		/** Filter bits */
#define CAN_F1DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F1DATA1_FD17_MSK								(0x01UL << CAN_F1DATA1_FD17_POS)		/** Filter bits */
#define CAN_F1DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F1DATA1_FD18_MSK								(0x01UL << CAN_F1DATA1_FD18_POS)		/** Filter bits */
#define CAN_F1DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F1DATA1_FD19_MSK								(0x01UL << CAN_F1DATA1_FD19_POS)		/** Filter bits */
#define CAN_F1DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F1DATA1_FD20_MSK								(0x01UL << CAN_F1DATA1_FD20_POS)		/** Filter bits */
#define CAN_F1DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F1DATA1_FD21_MSK								(0x01UL << CAN_F1DATA1_FD21_POS)		/** Filter bits */
#define CAN_F1DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F1DATA1_FD22_MSK								(0x01UL << CAN_F1DATA1_FD22_POS)		/** Filter bits */
#define CAN_F1DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F1DATA1_FD23_MSK								(0x01UL << CAN_F1DATA1_FD23_POS)		/** Filter bits */
#define CAN_F1DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F1DATA1_FD24_MSK								(0x01UL << CAN_F1DATA1_FD24_POS)		/** Filter bits */
#define CAN_F1DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F1DATA1_FD25_MSK								(0x01UL << CAN_F1DATA1_FD25_POS)		/** Filter bits */
#define CAN_F1DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F1DATA1_FD26_MSK								(0x01UL << CAN_F1DATA1_FD26_POS)		/** Filter bits */
#define CAN_F1DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F1DATA1_FD27_MSK								(0x01UL << CAN_F1DATA1_FD27_POS)		/** Filter bits */
#define CAN_F1DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F1DATA1_FD28_MSK								(0x01UL << CAN_F1DATA1_FD28_POS)		/** Filter bits */
#define CAN_F1DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F1DATA1_FD29_MSK								(0x01UL << CAN_F1DATA1_FD29_POS)		/** Filter bits */
#define CAN_F1DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F1DATA1_FD30_MSK								(0x01UL << CAN_F1DATA1_FD30_POS)		/** Filter bits */
#define CAN_F1DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F1DATA1_FD31_MSK								(0x01UL << CAN_F1DATA1_FD31_POS)		/** Filter bits */
#define CAN_F2DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F2DATA0_FD0_MSK								(0x01UL << CAN_F2DATA0_FD0_POS)		/** Filter bits */
#define CAN_F2DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F2DATA0_FD1_MSK								(0x01UL << CAN_F2DATA0_FD1_POS)		/** Filter bits */
#define CAN_F2DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F2DATA0_FD2_MSK								(0x01UL << CAN_F2DATA0_FD2_POS)		/** Filter bits */
#define CAN_F2DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F2DATA0_FD3_MSK								(0x01UL << CAN_F2DATA0_FD3_POS)		/** Filter bits */
#define CAN_F2DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F2DATA0_FD4_MSK								(0x01UL << CAN_F2DATA0_FD4_POS)		/** Filter bits */
#define CAN_F2DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F2DATA0_FD5_MSK								(0x01UL << CAN_F2DATA0_FD5_POS)		/** Filter bits */
#define CAN_F2DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F2DATA0_FD6_MSK								(0x01UL << CAN_F2DATA0_FD6_POS)		/** Filter bits */
#define CAN_F2DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F2DATA0_FD7_MSK								(0x01UL << CAN_F2DATA0_FD7_POS)		/** Filter bits */
#define CAN_F2DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F2DATA0_FD8_MSK								(0x01UL << CAN_F2DATA0_FD8_POS)		/** Filter bits */
#define CAN_F2DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F2DATA0_FD9_MSK								(0x01UL << CAN_F2DATA0_FD9_POS)		/** Filter bits */
#define CAN_F2DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F2DATA0_FD10_MSK								(0x01UL << CAN_F2DATA0_FD10_POS)		/** Filter bits */
#define CAN_F2DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F2DATA0_FD11_MSK								(0x01UL << CAN_F2DATA0_FD11_POS)		/** Filter bits */
#define CAN_F2DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F2DATA0_FD12_MSK								(0x01UL << CAN_F2DATA0_FD12_POS)		/** Filter bits */
#define CAN_F2DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F2DATA0_FD13_MSK								(0x01UL << CAN_F2DATA0_FD13_POS)		/** Filter bits */
#define CAN_F2DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F2DATA0_FD14_MSK								(0x01UL << CAN_F2DATA0_FD14_POS)		/** Filter bits */
#define CAN_F2DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F2DATA0_FD15_MSK								(0x01UL << CAN_F2DATA0_FD15_POS)		/** Filter bits */
#define CAN_F2DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F2DATA0_FD16_MSK								(0x01UL << CAN_F2DATA0_FD16_POS)		/** Filter bits */
#define CAN_F2DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F2DATA0_FD17_MSK								(0x01UL << CAN_F2DATA0_FD17_POS)		/** Filter bits */
#define CAN_F2DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F2DATA0_FD18_MSK								(0x01UL << CAN_F2DATA0_FD18_POS)		/** Filter bits */
#define CAN_F2DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F2DATA0_FD19_MSK								(0x01UL << CAN_F2DATA0_FD19_POS)		/** Filter bits */
#define CAN_F2DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F2DATA0_FD20_MSK								(0x01UL << CAN_F2DATA0_FD20_POS)		/** Filter bits */
#define CAN_F2DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F2DATA0_FD21_MSK								(0x01UL << CAN_F2DATA0_FD21_POS)		/** Filter bits */
#define CAN_F2DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F2DATA0_FD22_MSK								(0x01UL << CAN_F2DATA0_FD22_POS)		/** Filter bits */
#define CAN_F2DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F2DATA0_FD23_MSK								(0x01UL << CAN_F2DATA0_FD23_POS)		/** Filter bits */
#define CAN_F2DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F2DATA0_FD24_MSK								(0x01UL << CAN_F2DATA0_FD24_POS)		/** Filter bits */
#define CAN_F2DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F2DATA0_FD25_MSK								(0x01UL << CAN_F2DATA0_FD25_POS)		/** Filter bits */
#define CAN_F2DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F2DATA0_FD26_MSK								(0x01UL << CAN_F2DATA0_FD26_POS)		/** Filter bits */
#define CAN_F2DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F2DATA0_FD27_MSK								(0x01UL << CAN_F2DATA0_FD27_POS)		/** Filter bits */
#define CAN_F2DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F2DATA0_FD28_MSK								(0x01UL << CAN_F2DATA0_FD28_POS)		/** Filter bits */
#define CAN_F2DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F2DATA0_FD29_MSK								(0x01UL << CAN_F2DATA0_FD29_POS)		/** Filter bits */
#define CAN_F2DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F2DATA0_FD30_MSK								(0x01UL << CAN_F2DATA0_FD30_POS)		/** Filter bits */
#define CAN_F2DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F2DATA0_FD31_MSK								(0x01UL << CAN_F2DATA0_FD31_POS)		/** Filter bits */
#define CAN_F2DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F2DATA1_FD0_MSK								(0x01UL << CAN_F2DATA1_FD0_POS)		/** Filter bits */
#define CAN_F2DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F2DATA1_FD1_MSK								(0x01UL << CAN_F2DATA1_FD1_POS)		/** Filter bits */
#define CAN_F2DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F2DATA1_FD2_MSK								(0x01UL << CAN_F2DATA1_FD2_POS)		/** Filter bits */
#define CAN_F2DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F2DATA1_FD3_MSK								(0x01UL << CAN_F2DATA1_FD3_POS)		/** Filter bits */
#define CAN_F2DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F2DATA1_FD4_MSK								(0x01UL << CAN_F2DATA1_FD4_POS)		/** Filter bits */
#define CAN_F2DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F2DATA1_FD5_MSK								(0x01UL << CAN_F2DATA1_FD5_POS)		/** Filter bits */
#define CAN_F2DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F2DATA1_FD6_MSK								(0x01UL << CAN_F2DATA1_FD6_POS)		/** Filter bits */
#define CAN_F2DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F2DATA1_FD7_MSK								(0x01UL << CAN_F2DATA1_FD7_POS)		/** Filter bits */
#define CAN_F2DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F2DATA1_FD8_MSK								(0x01UL << CAN_F2DATA1_FD8_POS)		/** Filter bits */
#define CAN_F2DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F2DATA1_FD9_MSK								(0x01UL << CAN_F2DATA1_FD9_POS)		/** Filter bits */
#define CAN_F2DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F2DATA1_FD10_MSK								(0x01UL << CAN_F2DATA1_FD10_POS)		/** Filter bits */
#define CAN_F2DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F2DATA1_FD11_MSK								(0x01UL << CAN_F2DATA1_FD11_POS)		/** Filter bits */
#define CAN_F2DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F2DATA1_FD12_MSK								(0x01UL << CAN_F2DATA1_FD12_POS)		/** Filter bits */
#define CAN_F2DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F2DATA1_FD13_MSK								(0x01UL << CAN_F2DATA1_FD13_POS)		/** Filter bits */
#define CAN_F2DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F2DATA1_FD14_MSK								(0x01UL << CAN_F2DATA1_FD14_POS)		/** Filter bits */
#define CAN_F2DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F2DATA1_FD15_MSK								(0x01UL << CAN_F2DATA1_FD15_POS)		/** Filter bits */
#define CAN_F2DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F2DATA1_FD16_MSK								(0x01UL << CAN_F2DATA1_FD16_POS)		/** Filter bits */
#define CAN_F2DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F2DATA1_FD17_MSK								(0x01UL << CAN_F2DATA1_FD17_POS)		/** Filter bits */
#define CAN_F2DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F2DATA1_FD18_MSK								(0x01UL << CAN_F2DATA1_FD18_POS)		/** Filter bits */
#define CAN_F2DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F2DATA1_FD19_MSK								(0x01UL << CAN_F2DATA1_FD19_POS)		/** Filter bits */
#define CAN_F2DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F2DATA1_FD20_MSK								(0x01UL << CAN_F2DATA1_FD20_POS)		/** Filter bits */
#define CAN_F2DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F2DATA1_FD21_MSK								(0x01UL << CAN_F2DATA1_FD21_POS)		/** Filter bits */
#define CAN_F2DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F2DATA1_FD22_MSK								(0x01UL << CAN_F2DATA1_FD22_POS)		/** Filter bits */
#define CAN_F2DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F2DATA1_FD23_MSK								(0x01UL << CAN_F2DATA1_FD23_POS)		/** Filter bits */
#define CAN_F2DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F2DATA1_FD24_MSK								(0x01UL << CAN_F2DATA1_FD24_POS)		/** Filter bits */
#define CAN_F2DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F2DATA1_FD25_MSK								(0x01UL << CAN_F2DATA1_FD25_POS)		/** Filter bits */
#define CAN_F2DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F2DATA1_FD26_MSK								(0x01UL << CAN_F2DATA1_FD26_POS)		/** Filter bits */
#define CAN_F2DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F2DATA1_FD27_MSK								(0x01UL << CAN_F2DATA1_FD27_POS)		/** Filter bits */
#define CAN_F2DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F2DATA1_FD28_MSK								(0x01UL << CAN_F2DATA1_FD28_POS)		/** Filter bits */
#define CAN_F2DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F2DATA1_FD29_MSK								(0x01UL << CAN_F2DATA1_FD29_POS)		/** Filter bits */
#define CAN_F2DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F2DATA1_FD30_MSK								(0x01UL << CAN_F2DATA1_FD30_POS)		/** Filter bits */
#define CAN_F2DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F2DATA1_FD31_MSK								(0x01UL << CAN_F2DATA1_FD31_POS)		/** Filter bits */
#define CAN_F3DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F3DATA0_FD0_MSK								(0x01UL << CAN_F3DATA0_FD0_POS)		/** Filter bits */
#define CAN_F3DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F3DATA0_FD1_MSK								(0x01UL << CAN_F3DATA0_FD1_POS)		/** Filter bits */
#define CAN_F3DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F3DATA0_FD2_MSK								(0x01UL << CAN_F3DATA0_FD2_POS)		/** Filter bits */
#define CAN_F3DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F3DATA0_FD3_MSK								(0x01UL << CAN_F3DATA0_FD3_POS)		/** Filter bits */
#define CAN_F3DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F3DATA0_FD4_MSK								(0x01UL << CAN_F3DATA0_FD4_POS)		/** Filter bits */
#define CAN_F3DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F3DATA0_FD5_MSK								(0x01UL << CAN_F3DATA0_FD5_POS)		/** Filter bits */
#define CAN_F3DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F3DATA0_FD6_MSK								(0x01UL << CAN_F3DATA0_FD6_POS)		/** Filter bits */
#define CAN_F3DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F3DATA0_FD7_MSK								(0x01UL << CAN_F3DATA0_FD7_POS)		/** Filter bits */
#define CAN_F3DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F3DATA0_FD8_MSK								(0x01UL << CAN_F3DATA0_FD8_POS)		/** Filter bits */
#define CAN_F3DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F3DATA0_FD9_MSK								(0x01UL << CAN_F3DATA0_FD9_POS)		/** Filter bits */
#define CAN_F3DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F3DATA0_FD10_MSK								(0x01UL << CAN_F3DATA0_FD10_POS)		/** Filter bits */
#define CAN_F3DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F3DATA0_FD11_MSK								(0x01UL << CAN_F3DATA0_FD11_POS)		/** Filter bits */
#define CAN_F3DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F3DATA0_FD12_MSK								(0x01UL << CAN_F3DATA0_FD12_POS)		/** Filter bits */
#define CAN_F3DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F3DATA0_FD13_MSK								(0x01UL << CAN_F3DATA0_FD13_POS)		/** Filter bits */
#define CAN_F3DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F3DATA0_FD14_MSK								(0x01UL << CAN_F3DATA0_FD14_POS)		/** Filter bits */
#define CAN_F3DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F3DATA0_FD15_MSK								(0x01UL << CAN_F3DATA0_FD15_POS)		/** Filter bits */
#define CAN_F3DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F3DATA0_FD16_MSK								(0x01UL << CAN_F3DATA0_FD16_POS)		/** Filter bits */
#define CAN_F3DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F3DATA0_FD17_MSK								(0x01UL << CAN_F3DATA0_FD17_POS)		/** Filter bits */
#define CAN_F3DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F3DATA0_FD18_MSK								(0x01UL << CAN_F3DATA0_FD18_POS)		/** Filter bits */
#define CAN_F3DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F3DATA0_FD19_MSK								(0x01UL << CAN_F3DATA0_FD19_POS)		/** Filter bits */
#define CAN_F3DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F3DATA0_FD20_MSK								(0x01UL << CAN_F3DATA0_FD20_POS)		/** Filter bits */
#define CAN_F3DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F3DATA0_FD21_MSK								(0x01UL << CAN_F3DATA0_FD21_POS)		/** Filter bits */
#define CAN_F3DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F3DATA0_FD22_MSK								(0x01UL << CAN_F3DATA0_FD22_POS)		/** Filter bits */
#define CAN_F3DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F3DATA0_FD23_MSK								(0x01UL << CAN_F3DATA0_FD23_POS)		/** Filter bits */
#define CAN_F3DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F3DATA0_FD24_MSK								(0x01UL << CAN_F3DATA0_FD24_POS)		/** Filter bits */
#define CAN_F3DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F3DATA0_FD25_MSK								(0x01UL << CAN_F3DATA0_FD25_POS)		/** Filter bits */
#define CAN_F3DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F3DATA0_FD26_MSK								(0x01UL << CAN_F3DATA0_FD26_POS)		/** Filter bits */
#define CAN_F3DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F3DATA0_FD27_MSK								(0x01UL << CAN_F3DATA0_FD27_POS)		/** Filter bits */
#define CAN_F3DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F3DATA0_FD28_MSK								(0x01UL << CAN_F3DATA0_FD28_POS)		/** Filter bits */
#define CAN_F3DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F3DATA0_FD29_MSK								(0x01UL << CAN_F3DATA0_FD29_POS)		/** Filter bits */
#define CAN_F3DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F3DATA0_FD30_MSK								(0x01UL << CAN_F3DATA0_FD30_POS)		/** Filter bits */
#define CAN_F3DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F3DATA0_FD31_MSK								(0x01UL << CAN_F3DATA0_FD31_POS)		/** Filter bits */
#define CAN_F3DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F3DATA1_FD0_MSK								(0x01UL << CAN_F3DATA1_FD0_POS)		/** Filter bits */
#define CAN_F3DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F3DATA1_FD1_MSK								(0x01UL << CAN_F3DATA1_FD1_POS)		/** Filter bits */
#define CAN_F3DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F3DATA1_FD2_MSK								(0x01UL << CAN_F3DATA1_FD2_POS)		/** Filter bits */
#define CAN_F3DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F3DATA1_FD3_MSK								(0x01UL << CAN_F3DATA1_FD3_POS)		/** Filter bits */
#define CAN_F3DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F3DATA1_FD4_MSK								(0x01UL << CAN_F3DATA1_FD4_POS)		/** Filter bits */
#define CAN_F3DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F3DATA1_FD5_MSK								(0x01UL << CAN_F3DATA1_FD5_POS)		/** Filter bits */
#define CAN_F3DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F3DATA1_FD6_MSK								(0x01UL << CAN_F3DATA1_FD6_POS)		/** Filter bits */
#define CAN_F3DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F3DATA1_FD7_MSK								(0x01UL << CAN_F3DATA1_FD7_POS)		/** Filter bits */
#define CAN_F3DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F3DATA1_FD8_MSK								(0x01UL << CAN_F3DATA1_FD8_POS)		/** Filter bits */
#define CAN_F3DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F3DATA1_FD9_MSK								(0x01UL << CAN_F3DATA1_FD9_POS)		/** Filter bits */
#define CAN_F3DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F3DATA1_FD10_MSK								(0x01UL << CAN_F3DATA1_FD10_POS)		/** Filter bits */
#define CAN_F3DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F3DATA1_FD11_MSK								(0x01UL << CAN_F3DATA1_FD11_POS)		/** Filter bits */
#define CAN_F3DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F3DATA1_FD12_MSK								(0x01UL << CAN_F3DATA1_FD12_POS)		/** Filter bits */
#define CAN_F3DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F3DATA1_FD13_MSK								(0x01UL << CAN_F3DATA1_FD13_POS)		/** Filter bits */
#define CAN_F3DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F3DATA1_FD14_MSK								(0x01UL << CAN_F3DATA1_FD14_POS)		/** Filter bits */
#define CAN_F3DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F3DATA1_FD15_MSK								(0x01UL << CAN_F3DATA1_FD15_POS)		/** Filter bits */
#define CAN_F3DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F3DATA1_FD16_MSK								(0x01UL << CAN_F3DATA1_FD16_POS)		/** Filter bits */
#define CAN_F3DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F3DATA1_FD17_MSK								(0x01UL << CAN_F3DATA1_FD17_POS)		/** Filter bits */
#define CAN_F3DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F3DATA1_FD18_MSK								(0x01UL << CAN_F3DATA1_FD18_POS)		/** Filter bits */
#define CAN_F3DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F3DATA1_FD19_MSK								(0x01UL << CAN_F3DATA1_FD19_POS)		/** Filter bits */
#define CAN_F3DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F3DATA1_FD20_MSK								(0x01UL << CAN_F3DATA1_FD20_POS)		/** Filter bits */
#define CAN_F3DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F3DATA1_FD21_MSK								(0x01UL << CAN_F3DATA1_FD21_POS)		/** Filter bits */
#define CAN_F3DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F3DATA1_FD22_MSK								(0x01UL << CAN_F3DATA1_FD22_POS)		/** Filter bits */
#define CAN_F3DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F3DATA1_FD23_MSK								(0x01UL << CAN_F3DATA1_FD23_POS)		/** Filter bits */
#define CAN_F3DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F3DATA1_FD24_MSK								(0x01UL << CAN_F3DATA1_FD24_POS)		/** Filter bits */
#define CAN_F3DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F3DATA1_FD25_MSK								(0x01UL << CAN_F3DATA1_FD25_POS)		/** Filter bits */
#define CAN_F3DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F3DATA1_FD26_MSK								(0x01UL << CAN_F3DATA1_FD26_POS)		/** Filter bits */
#define CAN_F3DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F3DATA1_FD27_MSK								(0x01UL << CAN_F3DATA1_FD27_POS)		/** Filter bits */
#define CAN_F3DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F3DATA1_FD28_MSK								(0x01UL << CAN_F3DATA1_FD28_POS)		/** Filter bits */
#define CAN_F3DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F3DATA1_FD29_MSK								(0x01UL << CAN_F3DATA1_FD29_POS)		/** Filter bits */
#define CAN_F3DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F3DATA1_FD30_MSK								(0x01UL << CAN_F3DATA1_FD30_POS)		/** Filter bits */
#define CAN_F3DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F3DATA1_FD31_MSK								(0x01UL << CAN_F3DATA1_FD31_POS)		/** Filter bits */
#define CAN_F4DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F4DATA0_FD0_MSK								(0x01UL << CAN_F4DATA0_FD0_POS)		/** Filter bits */
#define CAN_F4DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F4DATA0_FD1_MSK								(0x01UL << CAN_F4DATA0_FD1_POS)		/** Filter bits */
#define CAN_F4DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F4DATA0_FD2_MSK								(0x01UL << CAN_F4DATA0_FD2_POS)		/** Filter bits */
#define CAN_F4DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F4DATA0_FD3_MSK								(0x01UL << CAN_F4DATA0_FD3_POS)		/** Filter bits */
#define CAN_F4DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F4DATA0_FD4_MSK								(0x01UL << CAN_F4DATA0_FD4_POS)		/** Filter bits */
#define CAN_F4DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F4DATA0_FD5_MSK								(0x01UL << CAN_F4DATA0_FD5_POS)		/** Filter bits */
#define CAN_F4DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F4DATA0_FD6_MSK								(0x01UL << CAN_F4DATA0_FD6_POS)		/** Filter bits */
#define CAN_F4DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F4DATA0_FD7_MSK								(0x01UL << CAN_F4DATA0_FD7_POS)		/** Filter bits */
#define CAN_F4DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F4DATA0_FD8_MSK								(0x01UL << CAN_F4DATA0_FD8_POS)		/** Filter bits */
#define CAN_F4DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F4DATA0_FD9_MSK								(0x01UL << CAN_F4DATA0_FD9_POS)		/** Filter bits */
#define CAN_F4DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F4DATA0_FD10_MSK								(0x01UL << CAN_F4DATA0_FD10_POS)		/** Filter bits */
#define CAN_F4DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F4DATA0_FD11_MSK								(0x01UL << CAN_F4DATA0_FD11_POS)		/** Filter bits */
#define CAN_F4DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F4DATA0_FD12_MSK								(0x01UL << CAN_F4DATA0_FD12_POS)		/** Filter bits */
#define CAN_F4DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F4DATA0_FD13_MSK								(0x01UL << CAN_F4DATA0_FD13_POS)		/** Filter bits */
#define CAN_F4DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F4DATA0_FD14_MSK								(0x01UL << CAN_F4DATA0_FD14_POS)		/** Filter bits */
#define CAN_F4DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F4DATA0_FD15_MSK								(0x01UL << CAN_F4DATA0_FD15_POS)		/** Filter bits */
#define CAN_F4DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F4DATA0_FD16_MSK								(0x01UL << CAN_F4DATA0_FD16_POS)		/** Filter bits */
#define CAN_F4DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F4DATA0_FD17_MSK								(0x01UL << CAN_F4DATA0_FD17_POS)		/** Filter bits */
#define CAN_F4DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F4DATA0_FD18_MSK								(0x01UL << CAN_F4DATA0_FD18_POS)		/** Filter bits */
#define CAN_F4DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F4DATA0_FD19_MSK								(0x01UL << CAN_F4DATA0_FD19_POS)		/** Filter bits */
#define CAN_F4DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F4DATA0_FD20_MSK								(0x01UL << CAN_F4DATA0_FD20_POS)		/** Filter bits */
#define CAN_F4DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F4DATA0_FD21_MSK								(0x01UL << CAN_F4DATA0_FD21_POS)		/** Filter bits */
#define CAN_F4DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F4DATA0_FD22_MSK								(0x01UL << CAN_F4DATA0_FD22_POS)		/** Filter bits */
#define CAN_F4DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F4DATA0_FD23_MSK								(0x01UL << CAN_F4DATA0_FD23_POS)		/** Filter bits */
#define CAN_F4DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F4DATA0_FD24_MSK								(0x01UL << CAN_F4DATA0_FD24_POS)		/** Filter bits */
#define CAN_F4DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F4DATA0_FD25_MSK								(0x01UL << CAN_F4DATA0_FD25_POS)		/** Filter bits */
#define CAN_F4DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F4DATA0_FD26_MSK								(0x01UL << CAN_F4DATA0_FD26_POS)		/** Filter bits */
#define CAN_F4DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F4DATA0_FD27_MSK								(0x01UL << CAN_F4DATA0_FD27_POS)		/** Filter bits */
#define CAN_F4DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F4DATA0_FD28_MSK								(0x01UL << CAN_F4DATA0_FD28_POS)		/** Filter bits */
#define CAN_F4DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F4DATA0_FD29_MSK								(0x01UL << CAN_F4DATA0_FD29_POS)		/** Filter bits */
#define CAN_F4DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F4DATA0_FD30_MSK								(0x01UL << CAN_F4DATA0_FD30_POS)		/** Filter bits */
#define CAN_F4DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F4DATA0_FD31_MSK								(0x01UL << CAN_F4DATA0_FD31_POS)		/** Filter bits */
#define CAN_F4DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F4DATA1_FD0_MSK								(0x01UL << CAN_F4DATA1_FD0_POS)		/** Filter bits */
#define CAN_F4DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F4DATA1_FD1_MSK								(0x01UL << CAN_F4DATA1_FD1_POS)		/** Filter bits */
#define CAN_F4DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F4DATA1_FD2_MSK								(0x01UL << CAN_F4DATA1_FD2_POS)		/** Filter bits */
#define CAN_F4DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F4DATA1_FD3_MSK								(0x01UL << CAN_F4DATA1_FD3_POS)		/** Filter bits */
#define CAN_F4DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F4DATA1_FD4_MSK								(0x01UL << CAN_F4DATA1_FD4_POS)		/** Filter bits */
#define CAN_F4DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F4DATA1_FD5_MSK								(0x01UL << CAN_F4DATA1_FD5_POS)		/** Filter bits */
#define CAN_F4DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F4DATA1_FD6_MSK								(0x01UL << CAN_F4DATA1_FD6_POS)		/** Filter bits */
#define CAN_F4DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F4DATA1_FD7_MSK								(0x01UL << CAN_F4DATA1_FD7_POS)		/** Filter bits */
#define CAN_F4DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F4DATA1_FD8_MSK								(0x01UL << CAN_F4DATA1_FD8_POS)		/** Filter bits */
#define CAN_F4DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F4DATA1_FD9_MSK								(0x01UL << CAN_F4DATA1_FD9_POS)		/** Filter bits */
#define CAN_F4DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F4DATA1_FD10_MSK								(0x01UL << CAN_F4DATA1_FD10_POS)		/** Filter bits */
#define CAN_F4DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F4DATA1_FD11_MSK								(0x01UL << CAN_F4DATA1_FD11_POS)		/** Filter bits */
#define CAN_F4DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F4DATA1_FD12_MSK								(0x01UL << CAN_F4DATA1_FD12_POS)		/** Filter bits */
#define CAN_F4DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F4DATA1_FD13_MSK								(0x01UL << CAN_F4DATA1_FD13_POS)		/** Filter bits */
#define CAN_F4DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F4DATA1_FD14_MSK								(0x01UL << CAN_F4DATA1_FD14_POS)		/** Filter bits */
#define CAN_F4DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F4DATA1_FD15_MSK								(0x01UL << CAN_F4DATA1_FD15_POS)		/** Filter bits */
#define CAN_F4DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F4DATA1_FD16_MSK								(0x01UL << CAN_F4DATA1_FD16_POS)		/** Filter bits */
#define CAN_F4DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F4DATA1_FD17_MSK								(0x01UL << CAN_F4DATA1_FD17_POS)		/** Filter bits */
#define CAN_F4DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F4DATA1_FD18_MSK								(0x01UL << CAN_F4DATA1_FD18_POS)		/** Filter bits */
#define CAN_F4DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F4DATA1_FD19_MSK								(0x01UL << CAN_F4DATA1_FD19_POS)		/** Filter bits */
#define CAN_F4DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F4DATA1_FD20_MSK								(0x01UL << CAN_F4DATA1_FD20_POS)		/** Filter bits */
#define CAN_F4DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F4DATA1_FD21_MSK								(0x01UL << CAN_F4DATA1_FD21_POS)		/** Filter bits */
#define CAN_F4DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F4DATA1_FD22_MSK								(0x01UL << CAN_F4DATA1_FD22_POS)		/** Filter bits */
#define CAN_F4DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F4DATA1_FD23_MSK								(0x01UL << CAN_F4DATA1_FD23_POS)		/** Filter bits */
#define CAN_F4DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F4DATA1_FD24_MSK								(0x01UL << CAN_F4DATA1_FD24_POS)		/** Filter bits */
#define CAN_F4DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F4DATA1_FD25_MSK								(0x01UL << CAN_F4DATA1_FD25_POS)		/** Filter bits */
#define CAN_F4DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F4DATA1_FD26_MSK								(0x01UL << CAN_F4DATA1_FD26_POS)		/** Filter bits */
#define CAN_F4DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F4DATA1_FD27_MSK								(0x01UL << CAN_F4DATA1_FD27_POS)		/** Filter bits */
#define CAN_F4DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F4DATA1_FD28_MSK								(0x01UL << CAN_F4DATA1_FD28_POS)		/** Filter bits */
#define CAN_F4DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F4DATA1_FD29_MSK								(0x01UL << CAN_F4DATA1_FD29_POS)		/** Filter bits */
#define CAN_F4DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F4DATA1_FD30_MSK								(0x01UL << CAN_F4DATA1_FD30_POS)		/** Filter bits */
#define CAN_F4DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F4DATA1_FD31_MSK								(0x01UL << CAN_F4DATA1_FD31_POS)		/** Filter bits */
#define CAN_F5DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F5DATA0_FD0_MSK								(0x01UL << CAN_F5DATA0_FD0_POS)		/** Filter bits */
#define CAN_F5DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F5DATA0_FD1_MSK								(0x01UL << CAN_F5DATA0_FD1_POS)		/** Filter bits */
#define CAN_F5DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F5DATA0_FD2_MSK								(0x01UL << CAN_F5DATA0_FD2_POS)		/** Filter bits */
#define CAN_F5DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F5DATA0_FD3_MSK								(0x01UL << CAN_F5DATA0_FD3_POS)		/** Filter bits */
#define CAN_F5DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F5DATA0_FD4_MSK								(0x01UL << CAN_F5DATA0_FD4_POS)		/** Filter bits */
#define CAN_F5DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F5DATA0_FD5_MSK								(0x01UL << CAN_F5DATA0_FD5_POS)		/** Filter bits */
#define CAN_F5DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F5DATA0_FD6_MSK								(0x01UL << CAN_F5DATA0_FD6_POS)		/** Filter bits */
#define CAN_F5DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F5DATA0_FD7_MSK								(0x01UL << CAN_F5DATA0_FD7_POS)		/** Filter bits */
#define CAN_F5DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F5DATA0_FD8_MSK								(0x01UL << CAN_F5DATA0_FD8_POS)		/** Filter bits */
#define CAN_F5DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F5DATA0_FD9_MSK								(0x01UL << CAN_F5DATA0_FD9_POS)		/** Filter bits */
#define CAN_F5DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F5DATA0_FD10_MSK								(0x01UL << CAN_F5DATA0_FD10_POS)		/** Filter bits */
#define CAN_F5DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F5DATA0_FD11_MSK								(0x01UL << CAN_F5DATA0_FD11_POS)		/** Filter bits */
#define CAN_F5DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F5DATA0_FD12_MSK								(0x01UL << CAN_F5DATA0_FD12_POS)		/** Filter bits */
#define CAN_F5DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F5DATA0_FD13_MSK								(0x01UL << CAN_F5DATA0_FD13_POS)		/** Filter bits */
#define CAN_F5DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F5DATA0_FD14_MSK								(0x01UL << CAN_F5DATA0_FD14_POS)		/** Filter bits */
#define CAN_F5DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F5DATA0_FD15_MSK								(0x01UL << CAN_F5DATA0_FD15_POS)		/** Filter bits */
#define CAN_F5DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F5DATA0_FD16_MSK								(0x01UL << CAN_F5DATA0_FD16_POS)		/** Filter bits */
#define CAN_F5DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F5DATA0_FD17_MSK								(0x01UL << CAN_F5DATA0_FD17_POS)		/** Filter bits */
#define CAN_F5DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F5DATA0_FD18_MSK								(0x01UL << CAN_F5DATA0_FD18_POS)		/** Filter bits */
#define CAN_F5DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F5DATA0_FD19_MSK								(0x01UL << CAN_F5DATA0_FD19_POS)		/** Filter bits */
#define CAN_F5DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F5DATA0_FD20_MSK								(0x01UL << CAN_F5DATA0_FD20_POS)		/** Filter bits */
#define CAN_F5DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F5DATA0_FD21_MSK								(0x01UL << CAN_F5DATA0_FD21_POS)		/** Filter bits */
#define CAN_F5DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F5DATA0_FD22_MSK								(0x01UL << CAN_F5DATA0_FD22_POS)		/** Filter bits */
#define CAN_F5DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F5DATA0_FD23_MSK								(0x01UL << CAN_F5DATA0_FD23_POS)		/** Filter bits */
#define CAN_F5DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F5DATA0_FD24_MSK								(0x01UL << CAN_F5DATA0_FD24_POS)		/** Filter bits */
#define CAN_F5DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F5DATA0_FD25_MSK								(0x01UL << CAN_F5DATA0_FD25_POS)		/** Filter bits */
#define CAN_F5DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F5DATA0_FD26_MSK								(0x01UL << CAN_F5DATA0_FD26_POS)		/** Filter bits */
#define CAN_F5DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F5DATA0_FD27_MSK								(0x01UL << CAN_F5DATA0_FD27_POS)		/** Filter bits */
#define CAN_F5DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F5DATA0_FD28_MSK								(0x01UL << CAN_F5DATA0_FD28_POS)		/** Filter bits */
#define CAN_F5DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F5DATA0_FD29_MSK								(0x01UL << CAN_F5DATA0_FD29_POS)		/** Filter bits */
#define CAN_F5DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F5DATA0_FD30_MSK								(0x01UL << CAN_F5DATA0_FD30_POS)		/** Filter bits */
#define CAN_F5DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F5DATA0_FD31_MSK								(0x01UL << CAN_F5DATA0_FD31_POS)		/** Filter bits */
#define CAN_F5DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F5DATA1_FD0_MSK								(0x01UL << CAN_F5DATA1_FD0_POS)		/** Filter bits */
#define CAN_F5DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F5DATA1_FD1_MSK								(0x01UL << CAN_F5DATA1_FD1_POS)		/** Filter bits */
#define CAN_F5DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F5DATA1_FD2_MSK								(0x01UL << CAN_F5DATA1_FD2_POS)		/** Filter bits */
#define CAN_F5DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F5DATA1_FD3_MSK								(0x01UL << CAN_F5DATA1_FD3_POS)		/** Filter bits */
#define CAN_F5DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F5DATA1_FD4_MSK								(0x01UL << CAN_F5DATA1_FD4_POS)		/** Filter bits */
#define CAN_F5DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F5DATA1_FD5_MSK								(0x01UL << CAN_F5DATA1_FD5_POS)		/** Filter bits */
#define CAN_F5DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F5DATA1_FD6_MSK								(0x01UL << CAN_F5DATA1_FD6_POS)		/** Filter bits */
#define CAN_F5DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F5DATA1_FD7_MSK								(0x01UL << CAN_F5DATA1_FD7_POS)		/** Filter bits */
#define CAN_F5DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F5DATA1_FD8_MSK								(0x01UL << CAN_F5DATA1_FD8_POS)		/** Filter bits */
#define CAN_F5DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F5DATA1_FD9_MSK								(0x01UL << CAN_F5DATA1_FD9_POS)		/** Filter bits */
#define CAN_F5DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F5DATA1_FD10_MSK								(0x01UL << CAN_F5DATA1_FD10_POS)		/** Filter bits */
#define CAN_F5DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F5DATA1_FD11_MSK								(0x01UL << CAN_F5DATA1_FD11_POS)		/** Filter bits */
#define CAN_F5DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F5DATA1_FD12_MSK								(0x01UL << CAN_F5DATA1_FD12_POS)		/** Filter bits */
#define CAN_F5DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F5DATA1_FD13_MSK								(0x01UL << CAN_F5DATA1_FD13_POS)		/** Filter bits */
#define CAN_F5DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F5DATA1_FD14_MSK								(0x01UL << CAN_F5DATA1_FD14_POS)		/** Filter bits */
#define CAN_F5DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F5DATA1_FD15_MSK								(0x01UL << CAN_F5DATA1_FD15_POS)		/** Filter bits */
#define CAN_F5DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F5DATA1_FD16_MSK								(0x01UL << CAN_F5DATA1_FD16_POS)		/** Filter bits */
#define CAN_F5DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F5DATA1_FD17_MSK								(0x01UL << CAN_F5DATA1_FD17_POS)		/** Filter bits */
#define CAN_F5DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F5DATA1_FD18_MSK								(0x01UL << CAN_F5DATA1_FD18_POS)		/** Filter bits */
#define CAN_F5DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F5DATA1_FD19_MSK								(0x01UL << CAN_F5DATA1_FD19_POS)		/** Filter bits */
#define CAN_F5DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F5DATA1_FD20_MSK								(0x01UL << CAN_F5DATA1_FD20_POS)		/** Filter bits */
#define CAN_F5DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F5DATA1_FD21_MSK								(0x01UL << CAN_F5DATA1_FD21_POS)		/** Filter bits */
#define CAN_F5DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F5DATA1_FD22_MSK								(0x01UL << CAN_F5DATA1_FD22_POS)		/** Filter bits */
#define CAN_F5DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F5DATA1_FD23_MSK								(0x01UL << CAN_F5DATA1_FD23_POS)		/** Filter bits */
#define CAN_F5DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F5DATA1_FD24_MSK								(0x01UL << CAN_F5DATA1_FD24_POS)		/** Filter bits */
#define CAN_F5DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F5DATA1_FD25_MSK								(0x01UL << CAN_F5DATA1_FD25_POS)		/** Filter bits */
#define CAN_F5DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F5DATA1_FD26_MSK								(0x01UL << CAN_F5DATA1_FD26_POS)		/** Filter bits */
#define CAN_F5DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F5DATA1_FD27_MSK								(0x01UL << CAN_F5DATA1_FD27_POS)		/** Filter bits */
#define CAN_F5DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F5DATA1_FD28_MSK								(0x01UL << CAN_F5DATA1_FD28_POS)		/** Filter bits */
#define CAN_F5DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F5DATA1_FD29_MSK								(0x01UL << CAN_F5DATA1_FD29_POS)		/** Filter bits */
#define CAN_F5DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F5DATA1_FD30_MSK								(0x01UL << CAN_F5DATA1_FD30_POS)		/** Filter bits */
#define CAN_F5DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F5DATA1_FD31_MSK								(0x01UL << CAN_F5DATA1_FD31_POS)		/** Filter bits */
#define CAN_F6DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F6DATA0_FD0_MSK								(0x01UL << CAN_F6DATA0_FD0_POS)		/** Filter bits */
#define CAN_F6DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F6DATA0_FD1_MSK								(0x01UL << CAN_F6DATA0_FD1_POS)		/** Filter bits */
#define CAN_F6DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F6DATA0_FD2_MSK								(0x01UL << CAN_F6DATA0_FD2_POS)		/** Filter bits */
#define CAN_F6DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F6DATA0_FD3_MSK								(0x01UL << CAN_F6DATA0_FD3_POS)		/** Filter bits */
#define CAN_F6DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F6DATA0_FD4_MSK								(0x01UL << CAN_F6DATA0_FD4_POS)		/** Filter bits */
#define CAN_F6DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F6DATA0_FD5_MSK								(0x01UL << CAN_F6DATA0_FD5_POS)		/** Filter bits */
#define CAN_F6DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F6DATA0_FD6_MSK								(0x01UL << CAN_F6DATA0_FD6_POS)		/** Filter bits */
#define CAN_F6DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F6DATA0_FD7_MSK								(0x01UL << CAN_F6DATA0_FD7_POS)		/** Filter bits */
#define CAN_F6DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F6DATA0_FD8_MSK								(0x01UL << CAN_F6DATA0_FD8_POS)		/** Filter bits */
#define CAN_F6DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F6DATA0_FD9_MSK								(0x01UL << CAN_F6DATA0_FD9_POS)		/** Filter bits */
#define CAN_F6DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F6DATA0_FD10_MSK								(0x01UL << CAN_F6DATA0_FD10_POS)		/** Filter bits */
#define CAN_F6DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F6DATA0_FD11_MSK								(0x01UL << CAN_F6DATA0_FD11_POS)		/** Filter bits */
#define CAN_F6DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F6DATA0_FD12_MSK								(0x01UL << CAN_F6DATA0_FD12_POS)		/** Filter bits */
#define CAN_F6DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F6DATA0_FD13_MSK								(0x01UL << CAN_F6DATA0_FD13_POS)		/** Filter bits */
#define CAN_F6DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F6DATA0_FD14_MSK								(0x01UL << CAN_F6DATA0_FD14_POS)		/** Filter bits */
#define CAN_F6DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F6DATA0_FD15_MSK								(0x01UL << CAN_F6DATA0_FD15_POS)		/** Filter bits */
#define CAN_F6DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F6DATA0_FD16_MSK								(0x01UL << CAN_F6DATA0_FD16_POS)		/** Filter bits */
#define CAN_F6DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F6DATA0_FD17_MSK								(0x01UL << CAN_F6DATA0_FD17_POS)		/** Filter bits */
#define CAN_F6DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F6DATA0_FD18_MSK								(0x01UL << CAN_F6DATA0_FD18_POS)		/** Filter bits */
#define CAN_F6DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F6DATA0_FD19_MSK								(0x01UL << CAN_F6DATA0_FD19_POS)		/** Filter bits */
#define CAN_F6DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F6DATA0_FD20_MSK								(0x01UL << CAN_F6DATA0_FD20_POS)		/** Filter bits */
#define CAN_F6DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F6DATA0_FD21_MSK								(0x01UL << CAN_F6DATA0_FD21_POS)		/** Filter bits */
#define CAN_F6DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F6DATA0_FD22_MSK								(0x01UL << CAN_F6DATA0_FD22_POS)		/** Filter bits */
#define CAN_F6DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F6DATA0_FD23_MSK								(0x01UL << CAN_F6DATA0_FD23_POS)		/** Filter bits */
#define CAN_F6DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F6DATA0_FD24_MSK								(0x01UL << CAN_F6DATA0_FD24_POS)		/** Filter bits */
#define CAN_F6DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F6DATA0_FD25_MSK								(0x01UL << CAN_F6DATA0_FD25_POS)		/** Filter bits */
#define CAN_F6DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F6DATA0_FD26_MSK								(0x01UL << CAN_F6DATA0_FD26_POS)		/** Filter bits */
#define CAN_F6DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F6DATA0_FD27_MSK								(0x01UL << CAN_F6DATA0_FD27_POS)		/** Filter bits */
#define CAN_F6DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F6DATA0_FD28_MSK								(0x01UL << CAN_F6DATA0_FD28_POS)		/** Filter bits */
#define CAN_F6DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F6DATA0_FD29_MSK								(0x01UL << CAN_F6DATA0_FD29_POS)		/** Filter bits */
#define CAN_F6DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F6DATA0_FD30_MSK								(0x01UL << CAN_F6DATA0_FD30_POS)		/** Filter bits */
#define CAN_F6DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F6DATA0_FD31_MSK								(0x01UL << CAN_F6DATA0_FD31_POS)		/** Filter bits */
#define CAN_F6DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F6DATA1_FD0_MSK								(0x01UL << CAN_F6DATA1_FD0_POS)		/** Filter bits */
#define CAN_F6DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F6DATA1_FD1_MSK								(0x01UL << CAN_F6DATA1_FD1_POS)		/** Filter bits */
#define CAN_F6DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F6DATA1_FD2_MSK								(0x01UL << CAN_F6DATA1_FD2_POS)		/** Filter bits */
#define CAN_F6DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F6DATA1_FD3_MSK								(0x01UL << CAN_F6DATA1_FD3_POS)		/** Filter bits */
#define CAN_F6DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F6DATA1_FD4_MSK								(0x01UL << CAN_F6DATA1_FD4_POS)		/** Filter bits */
#define CAN_F6DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F6DATA1_FD5_MSK								(0x01UL << CAN_F6DATA1_FD5_POS)		/** Filter bits */
#define CAN_F6DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F6DATA1_FD6_MSK								(0x01UL << CAN_F6DATA1_FD6_POS)		/** Filter bits */
#define CAN_F6DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F6DATA1_FD7_MSK								(0x01UL << CAN_F6DATA1_FD7_POS)		/** Filter bits */
#define CAN_F6DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F6DATA1_FD8_MSK								(0x01UL << CAN_F6DATA1_FD8_POS)		/** Filter bits */
#define CAN_F6DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F6DATA1_FD9_MSK								(0x01UL << CAN_F6DATA1_FD9_POS)		/** Filter bits */
#define CAN_F6DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F6DATA1_FD10_MSK								(0x01UL << CAN_F6DATA1_FD10_POS)		/** Filter bits */
#define CAN_F6DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F6DATA1_FD11_MSK								(0x01UL << CAN_F6DATA1_FD11_POS)		/** Filter bits */
#define CAN_F6DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F6DATA1_FD12_MSK								(0x01UL << CAN_F6DATA1_FD12_POS)		/** Filter bits */
#define CAN_F6DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F6DATA1_FD13_MSK								(0x01UL << CAN_F6DATA1_FD13_POS)		/** Filter bits */
#define CAN_F6DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F6DATA1_FD14_MSK								(0x01UL << CAN_F6DATA1_FD14_POS)		/** Filter bits */
#define CAN_F6DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F6DATA1_FD15_MSK								(0x01UL << CAN_F6DATA1_FD15_POS)		/** Filter bits */
#define CAN_F6DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F6DATA1_FD16_MSK								(0x01UL << CAN_F6DATA1_FD16_POS)		/** Filter bits */
#define CAN_F6DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F6DATA1_FD17_MSK								(0x01UL << CAN_F6DATA1_FD17_POS)		/** Filter bits */
#define CAN_F6DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F6DATA1_FD18_MSK								(0x01UL << CAN_F6DATA1_FD18_POS)		/** Filter bits */
#define CAN_F6DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F6DATA1_FD19_MSK								(0x01UL << CAN_F6DATA1_FD19_POS)		/** Filter bits */
#define CAN_F6DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F6DATA1_FD20_MSK								(0x01UL << CAN_F6DATA1_FD20_POS)		/** Filter bits */
#define CAN_F6DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F6DATA1_FD21_MSK								(0x01UL << CAN_F6DATA1_FD21_POS)		/** Filter bits */
#define CAN_F6DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F6DATA1_FD22_MSK								(0x01UL << CAN_F6DATA1_FD22_POS)		/** Filter bits */
#define CAN_F6DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F6DATA1_FD23_MSK								(0x01UL << CAN_F6DATA1_FD23_POS)		/** Filter bits */
#define CAN_F6DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F6DATA1_FD24_MSK								(0x01UL << CAN_F6DATA1_FD24_POS)		/** Filter bits */
#define CAN_F6DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F6DATA1_FD25_MSK								(0x01UL << CAN_F6DATA1_FD25_POS)		/** Filter bits */
#define CAN_F6DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F6DATA1_FD26_MSK								(0x01UL << CAN_F6DATA1_FD26_POS)		/** Filter bits */
#define CAN_F6DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F6DATA1_FD27_MSK								(0x01UL << CAN_F6DATA1_FD27_POS)		/** Filter bits */
#define CAN_F6DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F6DATA1_FD28_MSK								(0x01UL << CAN_F6DATA1_FD28_POS)		/** Filter bits */
#define CAN_F6DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F6DATA1_FD29_MSK								(0x01UL << CAN_F6DATA1_FD29_POS)		/** Filter bits */
#define CAN_F6DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F6DATA1_FD30_MSK								(0x01UL << CAN_F6DATA1_FD30_POS)		/** Filter bits */
#define CAN_F6DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F6DATA1_FD31_MSK								(0x01UL << CAN_F6DATA1_FD31_POS)		/** Filter bits */
#define CAN_F7DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F7DATA0_FD0_MSK								(0x01UL << CAN_F7DATA0_FD0_POS)		/** Filter bits */
#define CAN_F7DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F7DATA0_FD1_MSK								(0x01UL << CAN_F7DATA0_FD1_POS)		/** Filter bits */
#define CAN_F7DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F7DATA0_FD2_MSK								(0x01UL << CAN_F7DATA0_FD2_POS)		/** Filter bits */
#define CAN_F7DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F7DATA0_FD3_MSK								(0x01UL << CAN_F7DATA0_FD3_POS)		/** Filter bits */
#define CAN_F7DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F7DATA0_FD4_MSK								(0x01UL << CAN_F7DATA0_FD4_POS)		/** Filter bits */
#define CAN_F7DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F7DATA0_FD5_MSK								(0x01UL << CAN_F7DATA0_FD5_POS)		/** Filter bits */
#define CAN_F7DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F7DATA0_FD6_MSK								(0x01UL << CAN_F7DATA0_FD6_POS)		/** Filter bits */
#define CAN_F7DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F7DATA0_FD7_MSK								(0x01UL << CAN_F7DATA0_FD7_POS)		/** Filter bits */
#define CAN_F7DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F7DATA0_FD8_MSK								(0x01UL << CAN_F7DATA0_FD8_POS)		/** Filter bits */
#define CAN_F7DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F7DATA0_FD9_MSK								(0x01UL << CAN_F7DATA0_FD9_POS)		/** Filter bits */
#define CAN_F7DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F7DATA0_FD10_MSK								(0x01UL << CAN_F7DATA0_FD10_POS)		/** Filter bits */
#define CAN_F7DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F7DATA0_FD11_MSK								(0x01UL << CAN_F7DATA0_FD11_POS)		/** Filter bits */
#define CAN_F7DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F7DATA0_FD12_MSK								(0x01UL << CAN_F7DATA0_FD12_POS)		/** Filter bits */
#define CAN_F7DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F7DATA0_FD13_MSK								(0x01UL << CAN_F7DATA0_FD13_POS)		/** Filter bits */
#define CAN_F7DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F7DATA0_FD14_MSK								(0x01UL << CAN_F7DATA0_FD14_POS)		/** Filter bits */
#define CAN_F7DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F7DATA0_FD15_MSK								(0x01UL << CAN_F7DATA0_FD15_POS)		/** Filter bits */
#define CAN_F7DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F7DATA0_FD16_MSK								(0x01UL << CAN_F7DATA0_FD16_POS)		/** Filter bits */
#define CAN_F7DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F7DATA0_FD17_MSK								(0x01UL << CAN_F7DATA0_FD17_POS)		/** Filter bits */
#define CAN_F7DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F7DATA0_FD18_MSK								(0x01UL << CAN_F7DATA0_FD18_POS)		/** Filter bits */
#define CAN_F7DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F7DATA0_FD19_MSK								(0x01UL << CAN_F7DATA0_FD19_POS)		/** Filter bits */
#define CAN_F7DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F7DATA0_FD20_MSK								(0x01UL << CAN_F7DATA0_FD20_POS)		/** Filter bits */
#define CAN_F7DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F7DATA0_FD21_MSK								(0x01UL << CAN_F7DATA0_FD21_POS)		/** Filter bits */
#define CAN_F7DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F7DATA0_FD22_MSK								(0x01UL << CAN_F7DATA0_FD22_POS)		/** Filter bits */
#define CAN_F7DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F7DATA0_FD23_MSK								(0x01UL << CAN_F7DATA0_FD23_POS)		/** Filter bits */
#define CAN_F7DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F7DATA0_FD24_MSK								(0x01UL << CAN_F7DATA0_FD24_POS)		/** Filter bits */
#define CAN_F7DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F7DATA0_FD25_MSK								(0x01UL << CAN_F7DATA0_FD25_POS)		/** Filter bits */
#define CAN_F7DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F7DATA0_FD26_MSK								(0x01UL << CAN_F7DATA0_FD26_POS)		/** Filter bits */
#define CAN_F7DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F7DATA0_FD27_MSK								(0x01UL << CAN_F7DATA0_FD27_POS)		/** Filter bits */
#define CAN_F7DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F7DATA0_FD28_MSK								(0x01UL << CAN_F7DATA0_FD28_POS)		/** Filter bits */
#define CAN_F7DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F7DATA0_FD29_MSK								(0x01UL << CAN_F7DATA0_FD29_POS)		/** Filter bits */
#define CAN_F7DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F7DATA0_FD30_MSK								(0x01UL << CAN_F7DATA0_FD30_POS)		/** Filter bits */
#define CAN_F7DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F7DATA0_FD31_MSK								(0x01UL << CAN_F7DATA0_FD31_POS)		/** Filter bits */
#define CAN_F7DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F7DATA1_FD0_MSK								(0x01UL << CAN_F7DATA1_FD0_POS)		/** Filter bits */
#define CAN_F7DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F7DATA1_FD1_MSK								(0x01UL << CAN_F7DATA1_FD1_POS)		/** Filter bits */
#define CAN_F7DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F7DATA1_FD2_MSK								(0x01UL << CAN_F7DATA1_FD2_POS)		/** Filter bits */
#define CAN_F7DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F7DATA1_FD3_MSK								(0x01UL << CAN_F7DATA1_FD3_POS)		/** Filter bits */
#define CAN_F7DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F7DATA1_FD4_MSK								(0x01UL << CAN_F7DATA1_FD4_POS)		/** Filter bits */
#define CAN_F7DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F7DATA1_FD5_MSK								(0x01UL << CAN_F7DATA1_FD5_POS)		/** Filter bits */
#define CAN_F7DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F7DATA1_FD6_MSK								(0x01UL << CAN_F7DATA1_FD6_POS)		/** Filter bits */
#define CAN_F7DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F7DATA1_FD7_MSK								(0x01UL << CAN_F7DATA1_FD7_POS)		/** Filter bits */
#define CAN_F7DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F7DATA1_FD8_MSK								(0x01UL << CAN_F7DATA1_FD8_POS)		/** Filter bits */
#define CAN_F7DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F7DATA1_FD9_MSK								(0x01UL << CAN_F7DATA1_FD9_POS)		/** Filter bits */
#define CAN_F7DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F7DATA1_FD10_MSK								(0x01UL << CAN_F7DATA1_FD10_POS)		/** Filter bits */
#define CAN_F7DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F7DATA1_FD11_MSK								(0x01UL << CAN_F7DATA1_FD11_POS)		/** Filter bits */
#define CAN_F7DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F7DATA1_FD12_MSK								(0x01UL << CAN_F7DATA1_FD12_POS)		/** Filter bits */
#define CAN_F7DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F7DATA1_FD13_MSK								(0x01UL << CAN_F7DATA1_FD13_POS)		/** Filter bits */
#define CAN_F7DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F7DATA1_FD14_MSK								(0x01UL << CAN_F7DATA1_FD14_POS)		/** Filter bits */
#define CAN_F7DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F7DATA1_FD15_MSK								(0x01UL << CAN_F7DATA1_FD15_POS)		/** Filter bits */
#define CAN_F7DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F7DATA1_FD16_MSK								(0x01UL << CAN_F7DATA1_FD16_POS)		/** Filter bits */
#define CAN_F7DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F7DATA1_FD17_MSK								(0x01UL << CAN_F7DATA1_FD17_POS)		/** Filter bits */
#define CAN_F7DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F7DATA1_FD18_MSK								(0x01UL << CAN_F7DATA1_FD18_POS)		/** Filter bits */
#define CAN_F7DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F7DATA1_FD19_MSK								(0x01UL << CAN_F7DATA1_FD19_POS)		/** Filter bits */
#define CAN_F7DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F7DATA1_FD20_MSK								(0x01UL << CAN_F7DATA1_FD20_POS)		/** Filter bits */
#define CAN_F7DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F7DATA1_FD21_MSK								(0x01UL << CAN_F7DATA1_FD21_POS)		/** Filter bits */
#define CAN_F7DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F7DATA1_FD22_MSK								(0x01UL << CAN_F7DATA1_FD22_POS)		/** Filter bits */
#define CAN_F7DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F7DATA1_FD23_MSK								(0x01UL << CAN_F7DATA1_FD23_POS)		/** Filter bits */
#define CAN_F7DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F7DATA1_FD24_MSK								(0x01UL << CAN_F7DATA1_FD24_POS)		/** Filter bits */
#define CAN_F7DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F7DATA1_FD25_MSK								(0x01UL << CAN_F7DATA1_FD25_POS)		/** Filter bits */
#define CAN_F7DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F7DATA1_FD26_MSK								(0x01UL << CAN_F7DATA1_FD26_POS)		/** Filter bits */
#define CAN_F7DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F7DATA1_FD27_MSK								(0x01UL << CAN_F7DATA1_FD27_POS)		/** Filter bits */
#define CAN_F7DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F7DATA1_FD28_MSK								(0x01UL << CAN_F7DATA1_FD28_POS)		/** Filter bits */
#define CAN_F7DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F7DATA1_FD29_MSK								(0x01UL << CAN_F7DATA1_FD29_POS)		/** Filter bits */
#define CAN_F7DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F7DATA1_FD30_MSK								(0x01UL << CAN_F7DATA1_FD30_POS)		/** Filter bits */
#define CAN_F7DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F7DATA1_FD31_MSK								(0x01UL << CAN_F7DATA1_FD31_POS)		/** Filter bits */
#define CAN_F8DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F8DATA0_FD0_MSK								(0x01UL << CAN_F8DATA0_FD0_POS)		/** Filter bits */
#define CAN_F8DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F8DATA0_FD1_MSK								(0x01UL << CAN_F8DATA0_FD1_POS)		/** Filter bits */
#define CAN_F8DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F8DATA0_FD2_MSK								(0x01UL << CAN_F8DATA0_FD2_POS)		/** Filter bits */
#define CAN_F8DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F8DATA0_FD3_MSK								(0x01UL << CAN_F8DATA0_FD3_POS)		/** Filter bits */
#define CAN_F8DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F8DATA0_FD4_MSK								(0x01UL << CAN_F8DATA0_FD4_POS)		/** Filter bits */
#define CAN_F8DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F8DATA0_FD5_MSK								(0x01UL << CAN_F8DATA0_FD5_POS)		/** Filter bits */
#define CAN_F8DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F8DATA0_FD6_MSK								(0x01UL << CAN_F8DATA0_FD6_POS)		/** Filter bits */
#define CAN_F8DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F8DATA0_FD7_MSK								(0x01UL << CAN_F8DATA0_FD7_POS)		/** Filter bits */
#define CAN_F8DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F8DATA0_FD8_MSK								(0x01UL << CAN_F8DATA0_FD8_POS)		/** Filter bits */
#define CAN_F8DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F8DATA0_FD9_MSK								(0x01UL << CAN_F8DATA0_FD9_POS)		/** Filter bits */
#define CAN_F8DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F8DATA0_FD10_MSK								(0x01UL << CAN_F8DATA0_FD10_POS)		/** Filter bits */
#define CAN_F8DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F8DATA0_FD11_MSK								(0x01UL << CAN_F8DATA0_FD11_POS)		/** Filter bits */
#define CAN_F8DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F8DATA0_FD12_MSK								(0x01UL << CAN_F8DATA0_FD12_POS)		/** Filter bits */
#define CAN_F8DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F8DATA0_FD13_MSK								(0x01UL << CAN_F8DATA0_FD13_POS)		/** Filter bits */
#define CAN_F8DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F8DATA0_FD14_MSK								(0x01UL << CAN_F8DATA0_FD14_POS)		/** Filter bits */
#define CAN_F8DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F8DATA0_FD15_MSK								(0x01UL << CAN_F8DATA0_FD15_POS)		/** Filter bits */
#define CAN_F8DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F8DATA0_FD16_MSK								(0x01UL << CAN_F8DATA0_FD16_POS)		/** Filter bits */
#define CAN_F8DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F8DATA0_FD17_MSK								(0x01UL << CAN_F8DATA0_FD17_POS)		/** Filter bits */
#define CAN_F8DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F8DATA0_FD18_MSK								(0x01UL << CAN_F8DATA0_FD18_POS)		/** Filter bits */
#define CAN_F8DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F8DATA0_FD19_MSK								(0x01UL << CAN_F8DATA0_FD19_POS)		/** Filter bits */
#define CAN_F8DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F8DATA0_FD20_MSK								(0x01UL << CAN_F8DATA0_FD20_POS)		/** Filter bits */
#define CAN_F8DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F8DATA0_FD21_MSK								(0x01UL << CAN_F8DATA0_FD21_POS)		/** Filter bits */
#define CAN_F8DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F8DATA0_FD22_MSK								(0x01UL << CAN_F8DATA0_FD22_POS)		/** Filter bits */
#define CAN_F8DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F8DATA0_FD23_MSK								(0x01UL << CAN_F8DATA0_FD23_POS)		/** Filter bits */
#define CAN_F8DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F8DATA0_FD24_MSK								(0x01UL << CAN_F8DATA0_FD24_POS)		/** Filter bits */
#define CAN_F8DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F8DATA0_FD25_MSK								(0x01UL << CAN_F8DATA0_FD25_POS)		/** Filter bits */
#define CAN_F8DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F8DATA0_FD26_MSK								(0x01UL << CAN_F8DATA0_FD26_POS)		/** Filter bits */
#define CAN_F8DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F8DATA0_FD27_MSK								(0x01UL << CAN_F8DATA0_FD27_POS)		/** Filter bits */
#define CAN_F8DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F8DATA0_FD28_MSK								(0x01UL << CAN_F8DATA0_FD28_POS)		/** Filter bits */
#define CAN_F8DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F8DATA0_FD29_MSK								(0x01UL << CAN_F8DATA0_FD29_POS)		/** Filter bits */
#define CAN_F8DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F8DATA0_FD30_MSK								(0x01UL << CAN_F8DATA0_FD30_POS)		/** Filter bits */
#define CAN_F8DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F8DATA0_FD31_MSK								(0x01UL << CAN_F8DATA0_FD31_POS)		/** Filter bits */
#define CAN_F8DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F8DATA1_FD0_MSK								(0x01UL << CAN_F8DATA1_FD0_POS)		/** Filter bits */
#define CAN_F8DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F8DATA1_FD1_MSK								(0x01UL << CAN_F8DATA1_FD1_POS)		/** Filter bits */
#define CAN_F8DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F8DATA1_FD2_MSK								(0x01UL << CAN_F8DATA1_FD2_POS)		/** Filter bits */
#define CAN_F8DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F8DATA1_FD3_MSK								(0x01UL << CAN_F8DATA1_FD3_POS)		/** Filter bits */
#define CAN_F8DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F8DATA1_FD4_MSK								(0x01UL << CAN_F8DATA1_FD4_POS)		/** Filter bits */
#define CAN_F8DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F8DATA1_FD5_MSK								(0x01UL << CAN_F8DATA1_FD5_POS)		/** Filter bits */
#define CAN_F8DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F8DATA1_FD6_MSK								(0x01UL << CAN_F8DATA1_FD6_POS)		/** Filter bits */
#define CAN_F8DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F8DATA1_FD7_MSK								(0x01UL << CAN_F8DATA1_FD7_POS)		/** Filter bits */
#define CAN_F8DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F8DATA1_FD8_MSK								(0x01UL << CAN_F8DATA1_FD8_POS)		/** Filter bits */
#define CAN_F8DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F8DATA1_FD9_MSK								(0x01UL << CAN_F8DATA1_FD9_POS)		/** Filter bits */
#define CAN_F8DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F8DATA1_FD10_MSK								(0x01UL << CAN_F8DATA1_FD10_POS)		/** Filter bits */
#define CAN_F8DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F8DATA1_FD11_MSK								(0x01UL << CAN_F8DATA1_FD11_POS)		/** Filter bits */
#define CAN_F8DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F8DATA1_FD12_MSK								(0x01UL << CAN_F8DATA1_FD12_POS)		/** Filter bits */
#define CAN_F8DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F8DATA1_FD13_MSK								(0x01UL << CAN_F8DATA1_FD13_POS)		/** Filter bits */
#define CAN_F8DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F8DATA1_FD14_MSK								(0x01UL << CAN_F8DATA1_FD14_POS)		/** Filter bits */
#define CAN_F8DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F8DATA1_FD15_MSK								(0x01UL << CAN_F8DATA1_FD15_POS)		/** Filter bits */
#define CAN_F8DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F8DATA1_FD16_MSK								(0x01UL << CAN_F8DATA1_FD16_POS)		/** Filter bits */
#define CAN_F8DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F8DATA1_FD17_MSK								(0x01UL << CAN_F8DATA1_FD17_POS)		/** Filter bits */
#define CAN_F8DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F8DATA1_FD18_MSK								(0x01UL << CAN_F8DATA1_FD18_POS)		/** Filter bits */
#define CAN_F8DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F8DATA1_FD19_MSK								(0x01UL << CAN_F8DATA1_FD19_POS)		/** Filter bits */
#define CAN_F8DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F8DATA1_FD20_MSK								(0x01UL << CAN_F8DATA1_FD20_POS)		/** Filter bits */
#define CAN_F8DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F8DATA1_FD21_MSK								(0x01UL << CAN_F8DATA1_FD21_POS)		/** Filter bits */
#define CAN_F8DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F8DATA1_FD22_MSK								(0x01UL << CAN_F8DATA1_FD22_POS)		/** Filter bits */
#define CAN_F8DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F8DATA1_FD23_MSK								(0x01UL << CAN_F8DATA1_FD23_POS)		/** Filter bits */
#define CAN_F8DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F8DATA1_FD24_MSK								(0x01UL << CAN_F8DATA1_FD24_POS)		/** Filter bits */
#define CAN_F8DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F8DATA1_FD25_MSK								(0x01UL << CAN_F8DATA1_FD25_POS)		/** Filter bits */
#define CAN_F8DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F8DATA1_FD26_MSK								(0x01UL << CAN_F8DATA1_FD26_POS)		/** Filter bits */
#define CAN_F8DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F8DATA1_FD27_MSK								(0x01UL << CAN_F8DATA1_FD27_POS)		/** Filter bits */
#define CAN_F8DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F8DATA1_FD28_MSK								(0x01UL << CAN_F8DATA1_FD28_POS)		/** Filter bits */
#define CAN_F8DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F8DATA1_FD29_MSK								(0x01UL << CAN_F8DATA1_FD29_POS)		/** Filter bits */
#define CAN_F8DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F8DATA1_FD30_MSK								(0x01UL << CAN_F8DATA1_FD30_POS)		/** Filter bits */
#define CAN_F8DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F8DATA1_FD31_MSK								(0x01UL << CAN_F8DATA1_FD31_POS)		/** Filter bits */
#define CAN_F9DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F9DATA0_FD0_MSK								(0x01UL << CAN_F9DATA0_FD0_POS)		/** Filter bits */
#define CAN_F9DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F9DATA0_FD1_MSK								(0x01UL << CAN_F9DATA0_FD1_POS)		/** Filter bits */
#define CAN_F9DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F9DATA0_FD2_MSK								(0x01UL << CAN_F9DATA0_FD2_POS)		/** Filter bits */
#define CAN_F9DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F9DATA0_FD3_MSK								(0x01UL << CAN_F9DATA0_FD3_POS)		/** Filter bits */
#define CAN_F9DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F9DATA0_FD4_MSK								(0x01UL << CAN_F9DATA0_FD4_POS)		/** Filter bits */
#define CAN_F9DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F9DATA0_FD5_MSK								(0x01UL << CAN_F9DATA0_FD5_POS)		/** Filter bits */
#define CAN_F9DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F9DATA0_FD6_MSK								(0x01UL << CAN_F9DATA0_FD6_POS)		/** Filter bits */
#define CAN_F9DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F9DATA0_FD7_MSK								(0x01UL << CAN_F9DATA0_FD7_POS)		/** Filter bits */
#define CAN_F9DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F9DATA0_FD8_MSK								(0x01UL << CAN_F9DATA0_FD8_POS)		/** Filter bits */
#define CAN_F9DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F9DATA0_FD9_MSK								(0x01UL << CAN_F9DATA0_FD9_POS)		/** Filter bits */
#define CAN_F9DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F9DATA0_FD10_MSK								(0x01UL << CAN_F9DATA0_FD10_POS)		/** Filter bits */
#define CAN_F9DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F9DATA0_FD11_MSK								(0x01UL << CAN_F9DATA0_FD11_POS)		/** Filter bits */
#define CAN_F9DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F9DATA0_FD12_MSK								(0x01UL << CAN_F9DATA0_FD12_POS)		/** Filter bits */
#define CAN_F9DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F9DATA0_FD13_MSK								(0x01UL << CAN_F9DATA0_FD13_POS)		/** Filter bits */
#define CAN_F9DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F9DATA0_FD14_MSK								(0x01UL << CAN_F9DATA0_FD14_POS)		/** Filter bits */
#define CAN_F9DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F9DATA0_FD15_MSK								(0x01UL << CAN_F9DATA0_FD15_POS)		/** Filter bits */
#define CAN_F9DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F9DATA0_FD16_MSK								(0x01UL << CAN_F9DATA0_FD16_POS)		/** Filter bits */
#define CAN_F9DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F9DATA0_FD17_MSK								(0x01UL << CAN_F9DATA0_FD17_POS)		/** Filter bits */
#define CAN_F9DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F9DATA0_FD18_MSK								(0x01UL << CAN_F9DATA0_FD18_POS)		/** Filter bits */
#define CAN_F9DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F9DATA0_FD19_MSK								(0x01UL << CAN_F9DATA0_FD19_POS)		/** Filter bits */
#define CAN_F9DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F9DATA0_FD20_MSK								(0x01UL << CAN_F9DATA0_FD20_POS)		/** Filter bits */
#define CAN_F9DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F9DATA0_FD21_MSK								(0x01UL << CAN_F9DATA0_FD21_POS)		/** Filter bits */
#define CAN_F9DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F9DATA0_FD22_MSK								(0x01UL << CAN_F9DATA0_FD22_POS)		/** Filter bits */
#define CAN_F9DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F9DATA0_FD23_MSK								(0x01UL << CAN_F9DATA0_FD23_POS)		/** Filter bits */
#define CAN_F9DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F9DATA0_FD24_MSK								(0x01UL << CAN_F9DATA0_FD24_POS)		/** Filter bits */
#define CAN_F9DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F9DATA0_FD25_MSK								(0x01UL << CAN_F9DATA0_FD25_POS)		/** Filter bits */
#define CAN_F9DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F9DATA0_FD26_MSK								(0x01UL << CAN_F9DATA0_FD26_POS)		/** Filter bits */
#define CAN_F9DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F9DATA0_FD27_MSK								(0x01UL << CAN_F9DATA0_FD27_POS)		/** Filter bits */
#define CAN_F9DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F9DATA0_FD28_MSK								(0x01UL << CAN_F9DATA0_FD28_POS)		/** Filter bits */
#define CAN_F9DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F9DATA0_FD29_MSK								(0x01UL << CAN_F9DATA0_FD29_POS)		/** Filter bits */
#define CAN_F9DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F9DATA0_FD30_MSK								(0x01UL << CAN_F9DATA0_FD30_POS)		/** Filter bits */
#define CAN_F9DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F9DATA0_FD31_MSK								(0x01UL << CAN_F9DATA0_FD31_POS)		/** Filter bits */
#define CAN_F9DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F9DATA1_FD0_MSK								(0x01UL << CAN_F9DATA1_FD0_POS)		/** Filter bits */
#define CAN_F9DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F9DATA1_FD1_MSK								(0x01UL << CAN_F9DATA1_FD1_POS)		/** Filter bits */
#define CAN_F9DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F9DATA1_FD2_MSK								(0x01UL << CAN_F9DATA1_FD2_POS)		/** Filter bits */
#define CAN_F9DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F9DATA1_FD3_MSK								(0x01UL << CAN_F9DATA1_FD3_POS)		/** Filter bits */
#define CAN_F9DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F9DATA1_FD4_MSK								(0x01UL << CAN_F9DATA1_FD4_POS)		/** Filter bits */
#define CAN_F9DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F9DATA1_FD5_MSK								(0x01UL << CAN_F9DATA1_FD5_POS)		/** Filter bits */
#define CAN_F9DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F9DATA1_FD6_MSK								(0x01UL << CAN_F9DATA1_FD6_POS)		/** Filter bits */
#define CAN_F9DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F9DATA1_FD7_MSK								(0x01UL << CAN_F9DATA1_FD7_POS)		/** Filter bits */
#define CAN_F9DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F9DATA1_FD8_MSK								(0x01UL << CAN_F9DATA1_FD8_POS)		/** Filter bits */
#define CAN_F9DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F9DATA1_FD9_MSK								(0x01UL << CAN_F9DATA1_FD9_POS)		/** Filter bits */
#define CAN_F9DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F9DATA1_FD10_MSK								(0x01UL << CAN_F9DATA1_FD10_POS)		/** Filter bits */
#define CAN_F9DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F9DATA1_FD11_MSK								(0x01UL << CAN_F9DATA1_FD11_POS)		/** Filter bits */
#define CAN_F9DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F9DATA1_FD12_MSK								(0x01UL << CAN_F9DATA1_FD12_POS)		/** Filter bits */
#define CAN_F9DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F9DATA1_FD13_MSK								(0x01UL << CAN_F9DATA1_FD13_POS)		/** Filter bits */
#define CAN_F9DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F9DATA1_FD14_MSK								(0x01UL << CAN_F9DATA1_FD14_POS)		/** Filter bits */
#define CAN_F9DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F9DATA1_FD15_MSK								(0x01UL << CAN_F9DATA1_FD15_POS)		/** Filter bits */
#define CAN_F9DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F9DATA1_FD16_MSK								(0x01UL << CAN_F9DATA1_FD16_POS)		/** Filter bits */
#define CAN_F9DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F9DATA1_FD17_MSK								(0x01UL << CAN_F9DATA1_FD17_POS)		/** Filter bits */
#define CAN_F9DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F9DATA1_FD18_MSK								(0x01UL << CAN_F9DATA1_FD18_POS)		/** Filter bits */
#define CAN_F9DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F9DATA1_FD19_MSK								(0x01UL << CAN_F9DATA1_FD19_POS)		/** Filter bits */
#define CAN_F9DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F9DATA1_FD20_MSK								(0x01UL << CAN_F9DATA1_FD20_POS)		/** Filter bits */
#define CAN_F9DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F9DATA1_FD21_MSK								(0x01UL << CAN_F9DATA1_FD21_POS)		/** Filter bits */
#define CAN_F9DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F9DATA1_FD22_MSK								(0x01UL << CAN_F9DATA1_FD22_POS)		/** Filter bits */
#define CAN_F9DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F9DATA1_FD23_MSK								(0x01UL << CAN_F9DATA1_FD23_POS)		/** Filter bits */
#define CAN_F9DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F9DATA1_FD24_MSK								(0x01UL << CAN_F9DATA1_FD24_POS)		/** Filter bits */
#define CAN_F9DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F9DATA1_FD25_MSK								(0x01UL << CAN_F9DATA1_FD25_POS)		/** Filter bits */
#define CAN_F9DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F9DATA1_FD26_MSK								(0x01UL << CAN_F9DATA1_FD26_POS)		/** Filter bits */
#define CAN_F9DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F9DATA1_FD27_MSK								(0x01UL << CAN_F9DATA1_FD27_POS)		/** Filter bits */
#define CAN_F9DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F9DATA1_FD28_MSK								(0x01UL << CAN_F9DATA1_FD28_POS)		/** Filter bits */
#define CAN_F9DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F9DATA1_FD29_MSK								(0x01UL << CAN_F9DATA1_FD29_POS)		/** Filter bits */
#define CAN_F9DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F9DATA1_FD30_MSK								(0x01UL << CAN_F9DATA1_FD30_POS)		/** Filter bits */
#define CAN_F9DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F9DATA1_FD31_MSK								(0x01UL << CAN_F9DATA1_FD31_POS)		/** Filter bits */
#define CAN_F10DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F10DATA0_FD0_MSK								(0x01UL << CAN_F10DATA0_FD0_POS)		/** Filter bits */
#define CAN_F10DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F10DATA0_FD1_MSK								(0x01UL << CAN_F10DATA0_FD1_POS)		/** Filter bits */
#define CAN_F10DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F10DATA0_FD2_MSK								(0x01UL << CAN_F10DATA0_FD2_POS)		/** Filter bits */
#define CAN_F10DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F10DATA0_FD3_MSK								(0x01UL << CAN_F10DATA0_FD3_POS)		/** Filter bits */
#define CAN_F10DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F10DATA0_FD4_MSK								(0x01UL << CAN_F10DATA0_FD4_POS)		/** Filter bits */
#define CAN_F10DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F10DATA0_FD5_MSK								(0x01UL << CAN_F10DATA0_FD5_POS)		/** Filter bits */
#define CAN_F10DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F10DATA0_FD6_MSK								(0x01UL << CAN_F10DATA0_FD6_POS)		/** Filter bits */
#define CAN_F10DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F10DATA0_FD7_MSK								(0x01UL << CAN_F10DATA0_FD7_POS)		/** Filter bits */
#define CAN_F10DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F10DATA0_FD8_MSK								(0x01UL << CAN_F10DATA0_FD8_POS)		/** Filter bits */
#define CAN_F10DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F10DATA0_FD9_MSK								(0x01UL << CAN_F10DATA0_FD9_POS)		/** Filter bits */
#define CAN_F10DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F10DATA0_FD10_MSK								(0x01UL << CAN_F10DATA0_FD10_POS)		/** Filter bits */
#define CAN_F10DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F10DATA0_FD11_MSK								(0x01UL << CAN_F10DATA0_FD11_POS)		/** Filter bits */
#define CAN_F10DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F10DATA0_FD12_MSK								(0x01UL << CAN_F10DATA0_FD12_POS)		/** Filter bits */
#define CAN_F10DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F10DATA0_FD13_MSK								(0x01UL << CAN_F10DATA0_FD13_POS)		/** Filter bits */
#define CAN_F10DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F10DATA0_FD14_MSK								(0x01UL << CAN_F10DATA0_FD14_POS)		/** Filter bits */
#define CAN_F10DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F10DATA0_FD15_MSK								(0x01UL << CAN_F10DATA0_FD15_POS)		/** Filter bits */
#define CAN_F10DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F10DATA0_FD16_MSK								(0x01UL << CAN_F10DATA0_FD16_POS)		/** Filter bits */
#define CAN_F10DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F10DATA0_FD17_MSK								(0x01UL << CAN_F10DATA0_FD17_POS)		/** Filter bits */
#define CAN_F10DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F10DATA0_FD18_MSK								(0x01UL << CAN_F10DATA0_FD18_POS)		/** Filter bits */
#define CAN_F10DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F10DATA0_FD19_MSK								(0x01UL << CAN_F10DATA0_FD19_POS)		/** Filter bits */
#define CAN_F10DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F10DATA0_FD20_MSK								(0x01UL << CAN_F10DATA0_FD20_POS)		/** Filter bits */
#define CAN_F10DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F10DATA0_FD21_MSK								(0x01UL << CAN_F10DATA0_FD21_POS)		/** Filter bits */
#define CAN_F10DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F10DATA0_FD22_MSK								(0x01UL << CAN_F10DATA0_FD22_POS)		/** Filter bits */
#define CAN_F10DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F10DATA0_FD23_MSK								(0x01UL << CAN_F10DATA0_FD23_POS)		/** Filter bits */
#define CAN_F10DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F10DATA0_FD24_MSK								(0x01UL << CAN_F10DATA0_FD24_POS)		/** Filter bits */
#define CAN_F10DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F10DATA0_FD25_MSK								(0x01UL << CAN_F10DATA0_FD25_POS)		/** Filter bits */
#define CAN_F10DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F10DATA0_FD26_MSK								(0x01UL << CAN_F10DATA0_FD26_POS)		/** Filter bits */
#define CAN_F10DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F10DATA0_FD27_MSK								(0x01UL << CAN_F10DATA0_FD27_POS)		/** Filter bits */
#define CAN_F10DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F10DATA0_FD28_MSK								(0x01UL << CAN_F10DATA0_FD28_POS)		/** Filter bits */
#define CAN_F10DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F10DATA0_FD29_MSK								(0x01UL << CAN_F10DATA0_FD29_POS)		/** Filter bits */
#define CAN_F10DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F10DATA0_FD30_MSK								(0x01UL << CAN_F10DATA0_FD30_POS)		/** Filter bits */
#define CAN_F10DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F10DATA0_FD31_MSK								(0x01UL << CAN_F10DATA0_FD31_POS)		/** Filter bits */
#define CAN_F10DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F10DATA1_FD0_MSK								(0x01UL << CAN_F10DATA1_FD0_POS)		/** Filter bits */
#define CAN_F10DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F10DATA1_FD1_MSK								(0x01UL << CAN_F10DATA1_FD1_POS)		/** Filter bits */
#define CAN_F10DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F10DATA1_FD2_MSK								(0x01UL << CAN_F10DATA1_FD2_POS)		/** Filter bits */
#define CAN_F10DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F10DATA1_FD3_MSK								(0x01UL << CAN_F10DATA1_FD3_POS)		/** Filter bits */
#define CAN_F10DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F10DATA1_FD4_MSK								(0x01UL << CAN_F10DATA1_FD4_POS)		/** Filter bits */
#define CAN_F10DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F10DATA1_FD5_MSK								(0x01UL << CAN_F10DATA1_FD5_POS)		/** Filter bits */
#define CAN_F10DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F10DATA1_FD6_MSK								(0x01UL << CAN_F10DATA1_FD6_POS)		/** Filter bits */
#define CAN_F10DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F10DATA1_FD7_MSK								(0x01UL << CAN_F10DATA1_FD7_POS)		/** Filter bits */
#define CAN_F10DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F10DATA1_FD8_MSK								(0x01UL << CAN_F10DATA1_FD8_POS)		/** Filter bits */
#define CAN_F10DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F10DATA1_FD9_MSK								(0x01UL << CAN_F10DATA1_FD9_POS)		/** Filter bits */
#define CAN_F10DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F10DATA1_FD10_MSK								(0x01UL << CAN_F10DATA1_FD10_POS)		/** Filter bits */
#define CAN_F10DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F10DATA1_FD11_MSK								(0x01UL << CAN_F10DATA1_FD11_POS)		/** Filter bits */
#define CAN_F10DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F10DATA1_FD12_MSK								(0x01UL << CAN_F10DATA1_FD12_POS)		/** Filter bits */
#define CAN_F10DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F10DATA1_FD13_MSK								(0x01UL << CAN_F10DATA1_FD13_POS)		/** Filter bits */
#define CAN_F10DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F10DATA1_FD14_MSK								(0x01UL << CAN_F10DATA1_FD14_POS)		/** Filter bits */
#define CAN_F10DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F10DATA1_FD15_MSK								(0x01UL << CAN_F10DATA1_FD15_POS)		/** Filter bits */
#define CAN_F10DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F10DATA1_FD16_MSK								(0x01UL << CAN_F10DATA1_FD16_POS)		/** Filter bits */
#define CAN_F10DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F10DATA1_FD17_MSK								(0x01UL << CAN_F10DATA1_FD17_POS)		/** Filter bits */
#define CAN_F10DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F10DATA1_FD18_MSK								(0x01UL << CAN_F10DATA1_FD18_POS)		/** Filter bits */
#define CAN_F10DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F10DATA1_FD19_MSK								(0x01UL << CAN_F10DATA1_FD19_POS)		/** Filter bits */
#define CAN_F10DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F10DATA1_FD20_MSK								(0x01UL << CAN_F10DATA1_FD20_POS)		/** Filter bits */
#define CAN_F10DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F10DATA1_FD21_MSK								(0x01UL << CAN_F10DATA1_FD21_POS)		/** Filter bits */
#define CAN_F10DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F10DATA1_FD22_MSK								(0x01UL << CAN_F10DATA1_FD22_POS)		/** Filter bits */
#define CAN_F10DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F10DATA1_FD23_MSK								(0x01UL << CAN_F10DATA1_FD23_POS)		/** Filter bits */
#define CAN_F10DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F10DATA1_FD24_MSK								(0x01UL << CAN_F10DATA1_FD24_POS)		/** Filter bits */
#define CAN_F10DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F10DATA1_FD25_MSK								(0x01UL << CAN_F10DATA1_FD25_POS)		/** Filter bits */
#define CAN_F10DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F10DATA1_FD26_MSK								(0x01UL << CAN_F10DATA1_FD26_POS)		/** Filter bits */
#define CAN_F10DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F10DATA1_FD27_MSK								(0x01UL << CAN_F10DATA1_FD27_POS)		/** Filter bits */
#define CAN_F10DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F10DATA1_FD28_MSK								(0x01UL << CAN_F10DATA1_FD28_POS)		/** Filter bits */
#define CAN_F10DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F10DATA1_FD29_MSK								(0x01UL << CAN_F10DATA1_FD29_POS)		/** Filter bits */
#define CAN_F10DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F10DATA1_FD30_MSK								(0x01UL << CAN_F10DATA1_FD30_POS)		/** Filter bits */
#define CAN_F10DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F10DATA1_FD31_MSK								(0x01UL << CAN_F10DATA1_FD31_POS)		/** Filter bits */
#define CAN_F11DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F11DATA0_FD0_MSK								(0x01UL << CAN_F11DATA0_FD0_POS)		/** Filter bits */
#define CAN_F11DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F11DATA0_FD1_MSK								(0x01UL << CAN_F11DATA0_FD1_POS)		/** Filter bits */
#define CAN_F11DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F11DATA0_FD2_MSK								(0x01UL << CAN_F11DATA0_FD2_POS)		/** Filter bits */
#define CAN_F11DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F11DATA0_FD3_MSK								(0x01UL << CAN_F11DATA0_FD3_POS)		/** Filter bits */
#define CAN_F11DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F11DATA0_FD4_MSK								(0x01UL << CAN_F11DATA0_FD4_POS)		/** Filter bits */
#define CAN_F11DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F11DATA0_FD5_MSK								(0x01UL << CAN_F11DATA0_FD5_POS)		/** Filter bits */
#define CAN_F11DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F11DATA0_FD6_MSK								(0x01UL << CAN_F11DATA0_FD6_POS)		/** Filter bits */
#define CAN_F11DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F11DATA0_FD7_MSK								(0x01UL << CAN_F11DATA0_FD7_POS)		/** Filter bits */
#define CAN_F11DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F11DATA0_FD8_MSK								(0x01UL << CAN_F11DATA0_FD8_POS)		/** Filter bits */
#define CAN_F11DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F11DATA0_FD9_MSK								(0x01UL << CAN_F11DATA0_FD9_POS)		/** Filter bits */
#define CAN_F11DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F11DATA0_FD10_MSK								(0x01UL << CAN_F11DATA0_FD10_POS)		/** Filter bits */
#define CAN_F11DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F11DATA0_FD11_MSK								(0x01UL << CAN_F11DATA0_FD11_POS)		/** Filter bits */
#define CAN_F11DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F11DATA0_FD12_MSK								(0x01UL << CAN_F11DATA0_FD12_POS)		/** Filter bits */
#define CAN_F11DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F11DATA0_FD13_MSK								(0x01UL << CAN_F11DATA0_FD13_POS)		/** Filter bits */
#define CAN_F11DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F11DATA0_FD14_MSK								(0x01UL << CAN_F11DATA0_FD14_POS)		/** Filter bits */
#define CAN_F11DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F11DATA0_FD15_MSK								(0x01UL << CAN_F11DATA0_FD15_POS)		/** Filter bits */
#define CAN_F11DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F11DATA0_FD16_MSK								(0x01UL << CAN_F11DATA0_FD16_POS)		/** Filter bits */
#define CAN_F11DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F11DATA0_FD17_MSK								(0x01UL << CAN_F11DATA0_FD17_POS)		/** Filter bits */
#define CAN_F11DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F11DATA0_FD18_MSK								(0x01UL << CAN_F11DATA0_FD18_POS)		/** Filter bits */
#define CAN_F11DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F11DATA0_FD19_MSK								(0x01UL << CAN_F11DATA0_FD19_POS)		/** Filter bits */
#define CAN_F11DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F11DATA0_FD20_MSK								(0x01UL << CAN_F11DATA0_FD20_POS)		/** Filter bits */
#define CAN_F11DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F11DATA0_FD21_MSK								(0x01UL << CAN_F11DATA0_FD21_POS)		/** Filter bits */
#define CAN_F11DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F11DATA0_FD22_MSK								(0x01UL << CAN_F11DATA0_FD22_POS)		/** Filter bits */
#define CAN_F11DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F11DATA0_FD23_MSK								(0x01UL << CAN_F11DATA0_FD23_POS)		/** Filter bits */
#define CAN_F11DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F11DATA0_FD24_MSK								(0x01UL << CAN_F11DATA0_FD24_POS)		/** Filter bits */
#define CAN_F11DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F11DATA0_FD25_MSK								(0x01UL << CAN_F11DATA0_FD25_POS)		/** Filter bits */
#define CAN_F11DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F11DATA0_FD26_MSK								(0x01UL << CAN_F11DATA0_FD26_POS)		/** Filter bits */
#define CAN_F11DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F11DATA0_FD27_MSK								(0x01UL << CAN_F11DATA0_FD27_POS)		/** Filter bits */
#define CAN_F11DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F11DATA0_FD28_MSK								(0x01UL << CAN_F11DATA0_FD28_POS)		/** Filter bits */
#define CAN_F11DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F11DATA0_FD29_MSK								(0x01UL << CAN_F11DATA0_FD29_POS)		/** Filter bits */
#define CAN_F11DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F11DATA0_FD30_MSK								(0x01UL << CAN_F11DATA0_FD30_POS)		/** Filter bits */
#define CAN_F11DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F11DATA0_FD31_MSK								(0x01UL << CAN_F11DATA0_FD31_POS)		/** Filter bits */
#define CAN_F11DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F11DATA1_FD0_MSK								(0x01UL << CAN_F11DATA1_FD0_POS)		/** Filter bits */
#define CAN_F11DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F11DATA1_FD1_MSK								(0x01UL << CAN_F11DATA1_FD1_POS)		/** Filter bits */
#define CAN_F11DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F11DATA1_FD2_MSK								(0x01UL << CAN_F11DATA1_FD2_POS)		/** Filter bits */
#define CAN_F11DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F11DATA1_FD3_MSK								(0x01UL << CAN_F11DATA1_FD3_POS)		/** Filter bits */
#define CAN_F11DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F11DATA1_FD4_MSK								(0x01UL << CAN_F11DATA1_FD4_POS)		/** Filter bits */
#define CAN_F11DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F11DATA1_FD5_MSK								(0x01UL << CAN_F11DATA1_FD5_POS)		/** Filter bits */
#define CAN_F11DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F11DATA1_FD6_MSK								(0x01UL << CAN_F11DATA1_FD6_POS)		/** Filter bits */
#define CAN_F11DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F11DATA1_FD7_MSK								(0x01UL << CAN_F11DATA1_FD7_POS)		/** Filter bits */
#define CAN_F11DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F11DATA1_FD8_MSK								(0x01UL << CAN_F11DATA1_FD8_POS)		/** Filter bits */
#define CAN_F11DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F11DATA1_FD9_MSK								(0x01UL << CAN_F11DATA1_FD9_POS)		/** Filter bits */
#define CAN_F11DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F11DATA1_FD10_MSK								(0x01UL << CAN_F11DATA1_FD10_POS)		/** Filter bits */
#define CAN_F11DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F11DATA1_FD11_MSK								(0x01UL << CAN_F11DATA1_FD11_POS)		/** Filter bits */
#define CAN_F11DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F11DATA1_FD12_MSK								(0x01UL << CAN_F11DATA1_FD12_POS)		/** Filter bits */
#define CAN_F11DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F11DATA1_FD13_MSK								(0x01UL << CAN_F11DATA1_FD13_POS)		/** Filter bits */
#define CAN_F11DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F11DATA1_FD14_MSK								(0x01UL << CAN_F11DATA1_FD14_POS)		/** Filter bits */
#define CAN_F11DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F11DATA1_FD15_MSK								(0x01UL << CAN_F11DATA1_FD15_POS)		/** Filter bits */
#define CAN_F11DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F11DATA1_FD16_MSK								(0x01UL << CAN_F11DATA1_FD16_POS)		/** Filter bits */
#define CAN_F11DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F11DATA1_FD17_MSK								(0x01UL << CAN_F11DATA1_FD17_POS)		/** Filter bits */
#define CAN_F11DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F11DATA1_FD18_MSK								(0x01UL << CAN_F11DATA1_FD18_POS)		/** Filter bits */
#define CAN_F11DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F11DATA1_FD19_MSK								(0x01UL << CAN_F11DATA1_FD19_POS)		/** Filter bits */
#define CAN_F11DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F11DATA1_FD20_MSK								(0x01UL << CAN_F11DATA1_FD20_POS)		/** Filter bits */
#define CAN_F11DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F11DATA1_FD21_MSK								(0x01UL << CAN_F11DATA1_FD21_POS)		/** Filter bits */
#define CAN_F11DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F11DATA1_FD22_MSK								(0x01UL << CAN_F11DATA1_FD22_POS)		/** Filter bits */
#define CAN_F11DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F11DATA1_FD23_MSK								(0x01UL << CAN_F11DATA1_FD23_POS)		/** Filter bits */
#define CAN_F11DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F11DATA1_FD24_MSK								(0x01UL << CAN_F11DATA1_FD24_POS)		/** Filter bits */
#define CAN_F11DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F11DATA1_FD25_MSK								(0x01UL << CAN_F11DATA1_FD25_POS)		/** Filter bits */
#define CAN_F11DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F11DATA1_FD26_MSK								(0x01UL << CAN_F11DATA1_FD26_POS)		/** Filter bits */
#define CAN_F11DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F11DATA1_FD27_MSK								(0x01UL << CAN_F11DATA1_FD27_POS)		/** Filter bits */
#define CAN_F11DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F11DATA1_FD28_MSK								(0x01UL << CAN_F11DATA1_FD28_POS)		/** Filter bits */
#define CAN_F11DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F11DATA1_FD29_MSK								(0x01UL << CAN_F11DATA1_FD29_POS)		/** Filter bits */
#define CAN_F11DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F11DATA1_FD30_MSK								(0x01UL << CAN_F11DATA1_FD30_POS)		/** Filter bits */
#define CAN_F11DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F11DATA1_FD31_MSK								(0x01UL << CAN_F11DATA1_FD31_POS)		/** Filter bits */
#define CAN_F12DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F12DATA0_FD0_MSK								(0x01UL << CAN_F12DATA0_FD0_POS)		/** Filter bits */
#define CAN_F12DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F12DATA0_FD1_MSK								(0x01UL << CAN_F12DATA0_FD1_POS)		/** Filter bits */
#define CAN_F12DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F12DATA0_FD2_MSK								(0x01UL << CAN_F12DATA0_FD2_POS)		/** Filter bits */
#define CAN_F12DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F12DATA0_FD3_MSK								(0x01UL << CAN_F12DATA0_FD3_POS)		/** Filter bits */
#define CAN_F12DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F12DATA0_FD4_MSK								(0x01UL << CAN_F12DATA0_FD4_POS)		/** Filter bits */
#define CAN_F12DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F12DATA0_FD5_MSK								(0x01UL << CAN_F12DATA0_FD5_POS)		/** Filter bits */
#define CAN_F12DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F12DATA0_FD6_MSK								(0x01UL << CAN_F12DATA0_FD6_POS)		/** Filter bits */
#define CAN_F12DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F12DATA0_FD7_MSK								(0x01UL << CAN_F12DATA0_FD7_POS)		/** Filter bits */
#define CAN_F12DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F12DATA0_FD8_MSK								(0x01UL << CAN_F12DATA0_FD8_POS)		/** Filter bits */
#define CAN_F12DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F12DATA0_FD9_MSK								(0x01UL << CAN_F12DATA0_FD9_POS)		/** Filter bits */
#define CAN_F12DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F12DATA0_FD10_MSK								(0x01UL << CAN_F12DATA0_FD10_POS)		/** Filter bits */
#define CAN_F12DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F12DATA0_FD11_MSK								(0x01UL << CAN_F12DATA0_FD11_POS)		/** Filter bits */
#define CAN_F12DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F12DATA0_FD12_MSK								(0x01UL << CAN_F12DATA0_FD12_POS)		/** Filter bits */
#define CAN_F12DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F12DATA0_FD13_MSK								(0x01UL << CAN_F12DATA0_FD13_POS)		/** Filter bits */
#define CAN_F12DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F12DATA0_FD14_MSK								(0x01UL << CAN_F12DATA0_FD14_POS)		/** Filter bits */
#define CAN_F12DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F12DATA0_FD15_MSK								(0x01UL << CAN_F12DATA0_FD15_POS)		/** Filter bits */
#define CAN_F12DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F12DATA0_FD16_MSK								(0x01UL << CAN_F12DATA0_FD16_POS)		/** Filter bits */
#define CAN_F12DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F12DATA0_FD17_MSK								(0x01UL << CAN_F12DATA0_FD17_POS)		/** Filter bits */
#define CAN_F12DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F12DATA0_FD18_MSK								(0x01UL << CAN_F12DATA0_FD18_POS)		/** Filter bits */
#define CAN_F12DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F12DATA0_FD19_MSK								(0x01UL << CAN_F12DATA0_FD19_POS)		/** Filter bits */
#define CAN_F12DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F12DATA0_FD20_MSK								(0x01UL << CAN_F12DATA0_FD20_POS)		/** Filter bits */
#define CAN_F12DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F12DATA0_FD21_MSK								(0x01UL << CAN_F12DATA0_FD21_POS)		/** Filter bits */
#define CAN_F12DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F12DATA0_FD22_MSK								(0x01UL << CAN_F12DATA0_FD22_POS)		/** Filter bits */
#define CAN_F12DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F12DATA0_FD23_MSK								(0x01UL << CAN_F12DATA0_FD23_POS)		/** Filter bits */
#define CAN_F12DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F12DATA0_FD24_MSK								(0x01UL << CAN_F12DATA0_FD24_POS)		/** Filter bits */
#define CAN_F12DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F12DATA0_FD25_MSK								(0x01UL << CAN_F12DATA0_FD25_POS)		/** Filter bits */
#define CAN_F12DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F12DATA0_FD26_MSK								(0x01UL << CAN_F12DATA0_FD26_POS)		/** Filter bits */
#define CAN_F12DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F12DATA0_FD27_MSK								(0x01UL << CAN_F12DATA0_FD27_POS)		/** Filter bits */
#define CAN_F12DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F12DATA0_FD28_MSK								(0x01UL << CAN_F12DATA0_FD28_POS)		/** Filter bits */
#define CAN_F12DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F12DATA0_FD29_MSK								(0x01UL << CAN_F12DATA0_FD29_POS)		/** Filter bits */
#define CAN_F12DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F12DATA0_FD30_MSK								(0x01UL << CAN_F12DATA0_FD30_POS)		/** Filter bits */
#define CAN_F12DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F12DATA0_FD31_MSK								(0x01UL << CAN_F12DATA0_FD31_POS)		/** Filter bits */
#define CAN_F12DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F12DATA1_FD0_MSK								(0x01UL << CAN_F12DATA1_FD0_POS)		/** Filter bits */
#define CAN_F12DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F12DATA1_FD1_MSK								(0x01UL << CAN_F12DATA1_FD1_POS)		/** Filter bits */
#define CAN_F12DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F12DATA1_FD2_MSK								(0x01UL << CAN_F12DATA1_FD2_POS)		/** Filter bits */
#define CAN_F12DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F12DATA1_FD3_MSK								(0x01UL << CAN_F12DATA1_FD3_POS)		/** Filter bits */
#define CAN_F12DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F12DATA1_FD4_MSK								(0x01UL << CAN_F12DATA1_FD4_POS)		/** Filter bits */
#define CAN_F12DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F12DATA1_FD5_MSK								(0x01UL << CAN_F12DATA1_FD5_POS)		/** Filter bits */
#define CAN_F12DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F12DATA1_FD6_MSK								(0x01UL << CAN_F12DATA1_FD6_POS)		/** Filter bits */
#define CAN_F12DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F12DATA1_FD7_MSK								(0x01UL << CAN_F12DATA1_FD7_POS)		/** Filter bits */
#define CAN_F12DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F12DATA1_FD8_MSK								(0x01UL << CAN_F12DATA1_FD8_POS)		/** Filter bits */
#define CAN_F12DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F12DATA1_FD9_MSK								(0x01UL << CAN_F12DATA1_FD9_POS)		/** Filter bits */
#define CAN_F12DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F12DATA1_FD10_MSK								(0x01UL << CAN_F12DATA1_FD10_POS)		/** Filter bits */
#define CAN_F12DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F12DATA1_FD11_MSK								(0x01UL << CAN_F12DATA1_FD11_POS)		/** Filter bits */
#define CAN_F12DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F12DATA1_FD12_MSK								(0x01UL << CAN_F12DATA1_FD12_POS)		/** Filter bits */
#define CAN_F12DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F12DATA1_FD13_MSK								(0x01UL << CAN_F12DATA1_FD13_POS)		/** Filter bits */
#define CAN_F12DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F12DATA1_FD14_MSK								(0x01UL << CAN_F12DATA1_FD14_POS)		/** Filter bits */
#define CAN_F12DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F12DATA1_FD15_MSK								(0x01UL << CAN_F12DATA1_FD15_POS)		/** Filter bits */
#define CAN_F12DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F12DATA1_FD16_MSK								(0x01UL << CAN_F12DATA1_FD16_POS)		/** Filter bits */
#define CAN_F12DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F12DATA1_FD17_MSK								(0x01UL << CAN_F12DATA1_FD17_POS)		/** Filter bits */
#define CAN_F12DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F12DATA1_FD18_MSK								(0x01UL << CAN_F12DATA1_FD18_POS)		/** Filter bits */
#define CAN_F12DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F12DATA1_FD19_MSK								(0x01UL << CAN_F12DATA1_FD19_POS)		/** Filter bits */
#define CAN_F12DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F12DATA1_FD20_MSK								(0x01UL << CAN_F12DATA1_FD20_POS)		/** Filter bits */
#define CAN_F12DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F12DATA1_FD21_MSK								(0x01UL << CAN_F12DATA1_FD21_POS)		/** Filter bits */
#define CAN_F12DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F12DATA1_FD22_MSK								(0x01UL << CAN_F12DATA1_FD22_POS)		/** Filter bits */
#define CAN_F12DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F12DATA1_FD23_MSK								(0x01UL << CAN_F12DATA1_FD23_POS)		/** Filter bits */
#define CAN_F12DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F12DATA1_FD24_MSK								(0x01UL << CAN_F12DATA1_FD24_POS)		/** Filter bits */
#define CAN_F12DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F12DATA1_FD25_MSK								(0x01UL << CAN_F12DATA1_FD25_POS)		/** Filter bits */
#define CAN_F12DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F12DATA1_FD26_MSK								(0x01UL << CAN_F12DATA1_FD26_POS)		/** Filter bits */
#define CAN_F12DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F12DATA1_FD27_MSK								(0x01UL << CAN_F12DATA1_FD27_POS)		/** Filter bits */
#define CAN_F12DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F12DATA1_FD28_MSK								(0x01UL << CAN_F12DATA1_FD28_POS)		/** Filter bits */
#define CAN_F12DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F12DATA1_FD29_MSK								(0x01UL << CAN_F12DATA1_FD29_POS)		/** Filter bits */
#define CAN_F12DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F12DATA1_FD30_MSK								(0x01UL << CAN_F12DATA1_FD30_POS)		/** Filter bits */
#define CAN_F12DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F12DATA1_FD31_MSK								(0x01UL << CAN_F12DATA1_FD31_POS)		/** Filter bits */
#define CAN_F13DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F13DATA0_FD0_MSK								(0x01UL << CAN_F13DATA0_FD0_POS)		/** Filter bits */
#define CAN_F13DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F13DATA0_FD1_MSK								(0x01UL << CAN_F13DATA0_FD1_POS)		/** Filter bits */
#define CAN_F13DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F13DATA0_FD2_MSK								(0x01UL << CAN_F13DATA0_FD2_POS)		/** Filter bits */
#define CAN_F13DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F13DATA0_FD3_MSK								(0x01UL << CAN_F13DATA0_FD3_POS)		/** Filter bits */
#define CAN_F13DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F13DATA0_FD4_MSK								(0x01UL << CAN_F13DATA0_FD4_POS)		/** Filter bits */
#define CAN_F13DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F13DATA0_FD5_MSK								(0x01UL << CAN_F13DATA0_FD5_POS)		/** Filter bits */
#define CAN_F13DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F13DATA0_FD6_MSK								(0x01UL << CAN_F13DATA0_FD6_POS)		/** Filter bits */
#define CAN_F13DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F13DATA0_FD7_MSK								(0x01UL << CAN_F13DATA0_FD7_POS)		/** Filter bits */
#define CAN_F13DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F13DATA0_FD8_MSK								(0x01UL << CAN_F13DATA0_FD8_POS)		/** Filter bits */
#define CAN_F13DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F13DATA0_FD9_MSK								(0x01UL << CAN_F13DATA0_FD9_POS)		/** Filter bits */
#define CAN_F13DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F13DATA0_FD10_MSK								(0x01UL << CAN_F13DATA0_FD10_POS)		/** Filter bits */
#define CAN_F13DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F13DATA0_FD11_MSK								(0x01UL << CAN_F13DATA0_FD11_POS)		/** Filter bits */
#define CAN_F13DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F13DATA0_FD12_MSK								(0x01UL << CAN_F13DATA0_FD12_POS)		/** Filter bits */
#define CAN_F13DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F13DATA0_FD13_MSK								(0x01UL << CAN_F13DATA0_FD13_POS)		/** Filter bits */
#define CAN_F13DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F13DATA0_FD14_MSK								(0x01UL << CAN_F13DATA0_FD14_POS)		/** Filter bits */
#define CAN_F13DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F13DATA0_FD15_MSK								(0x01UL << CAN_F13DATA0_FD15_POS)		/** Filter bits */
#define CAN_F13DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F13DATA0_FD16_MSK								(0x01UL << CAN_F13DATA0_FD16_POS)		/** Filter bits */
#define CAN_F13DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F13DATA0_FD17_MSK								(0x01UL << CAN_F13DATA0_FD17_POS)		/** Filter bits */
#define CAN_F13DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F13DATA0_FD18_MSK								(0x01UL << CAN_F13DATA0_FD18_POS)		/** Filter bits */
#define CAN_F13DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F13DATA0_FD19_MSK								(0x01UL << CAN_F13DATA0_FD19_POS)		/** Filter bits */
#define CAN_F13DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F13DATA0_FD20_MSK								(0x01UL << CAN_F13DATA0_FD20_POS)		/** Filter bits */
#define CAN_F13DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F13DATA0_FD21_MSK								(0x01UL << CAN_F13DATA0_FD21_POS)		/** Filter bits */
#define CAN_F13DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F13DATA0_FD22_MSK								(0x01UL << CAN_F13DATA0_FD22_POS)		/** Filter bits */
#define CAN_F13DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F13DATA0_FD23_MSK								(0x01UL << CAN_F13DATA0_FD23_POS)		/** Filter bits */
#define CAN_F13DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F13DATA0_FD24_MSK								(0x01UL << CAN_F13DATA0_FD24_POS)		/** Filter bits */
#define CAN_F13DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F13DATA0_FD25_MSK								(0x01UL << CAN_F13DATA0_FD25_POS)		/** Filter bits */
#define CAN_F13DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F13DATA0_FD26_MSK								(0x01UL << CAN_F13DATA0_FD26_POS)		/** Filter bits */
#define CAN_F13DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F13DATA0_FD27_MSK								(0x01UL << CAN_F13DATA0_FD27_POS)		/** Filter bits */
#define CAN_F13DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F13DATA0_FD28_MSK								(0x01UL << CAN_F13DATA0_FD28_POS)		/** Filter bits */
#define CAN_F13DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F13DATA0_FD29_MSK								(0x01UL << CAN_F13DATA0_FD29_POS)		/** Filter bits */
#define CAN_F13DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F13DATA0_FD30_MSK								(0x01UL << CAN_F13DATA0_FD30_POS)		/** Filter bits */
#define CAN_F13DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F13DATA0_FD31_MSK								(0x01UL << CAN_F13DATA0_FD31_POS)		/** Filter bits */
#define CAN_F13DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F13DATA1_FD0_MSK								(0x01UL << CAN_F13DATA1_FD0_POS)		/** Filter bits */
#define CAN_F13DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F13DATA1_FD1_MSK								(0x01UL << CAN_F13DATA1_FD1_POS)		/** Filter bits */
#define CAN_F13DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F13DATA1_FD2_MSK								(0x01UL << CAN_F13DATA1_FD2_POS)		/** Filter bits */
#define CAN_F13DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F13DATA1_FD3_MSK								(0x01UL << CAN_F13DATA1_FD3_POS)		/** Filter bits */
#define CAN_F13DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F13DATA1_FD4_MSK								(0x01UL << CAN_F13DATA1_FD4_POS)		/** Filter bits */
#define CAN_F13DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F13DATA1_FD5_MSK								(0x01UL << CAN_F13DATA1_FD5_POS)		/** Filter bits */
#define CAN_F13DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F13DATA1_FD6_MSK								(0x01UL << CAN_F13DATA1_FD6_POS)		/** Filter bits */
#define CAN_F13DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F13DATA1_FD7_MSK								(0x01UL << CAN_F13DATA1_FD7_POS)		/** Filter bits */
#define CAN_F13DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F13DATA1_FD8_MSK								(0x01UL << CAN_F13DATA1_FD8_POS)		/** Filter bits */
#define CAN_F13DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F13DATA1_FD9_MSK								(0x01UL << CAN_F13DATA1_FD9_POS)		/** Filter bits */
#define CAN_F13DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F13DATA1_FD10_MSK								(0x01UL << CAN_F13DATA1_FD10_POS)		/** Filter bits */
#define CAN_F13DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F13DATA1_FD11_MSK								(0x01UL << CAN_F13DATA1_FD11_POS)		/** Filter bits */
#define CAN_F13DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F13DATA1_FD12_MSK								(0x01UL << CAN_F13DATA1_FD12_POS)		/** Filter bits */
#define CAN_F13DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F13DATA1_FD13_MSK								(0x01UL << CAN_F13DATA1_FD13_POS)		/** Filter bits */
#define CAN_F13DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F13DATA1_FD14_MSK								(0x01UL << CAN_F13DATA1_FD14_POS)		/** Filter bits */
#define CAN_F13DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F13DATA1_FD15_MSK								(0x01UL << CAN_F13DATA1_FD15_POS)		/** Filter bits */
#define CAN_F13DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F13DATA1_FD16_MSK								(0x01UL << CAN_F13DATA1_FD16_POS)		/** Filter bits */
#define CAN_F13DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F13DATA1_FD17_MSK								(0x01UL << CAN_F13DATA1_FD17_POS)		/** Filter bits */
#define CAN_F13DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F13DATA1_FD18_MSK								(0x01UL << CAN_F13DATA1_FD18_POS)		/** Filter bits */
#define CAN_F13DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F13DATA1_FD19_MSK								(0x01UL << CAN_F13DATA1_FD19_POS)		/** Filter bits */
#define CAN_F13DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F13DATA1_FD20_MSK								(0x01UL << CAN_F13DATA1_FD20_POS)		/** Filter bits */
#define CAN_F13DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F13DATA1_FD21_MSK								(0x01UL << CAN_F13DATA1_FD21_POS)		/** Filter bits */
#define CAN_F13DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F13DATA1_FD22_MSK								(0x01UL << CAN_F13DATA1_FD22_POS)		/** Filter bits */
#define CAN_F13DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F13DATA1_FD23_MSK								(0x01UL << CAN_F13DATA1_FD23_POS)		/** Filter bits */
#define CAN_F13DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F13DATA1_FD24_MSK								(0x01UL << CAN_F13DATA1_FD24_POS)		/** Filter bits */
#define CAN_F13DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F13DATA1_FD25_MSK								(0x01UL << CAN_F13DATA1_FD25_POS)		/** Filter bits */
#define CAN_F13DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F13DATA1_FD26_MSK								(0x01UL << CAN_F13DATA1_FD26_POS)		/** Filter bits */
#define CAN_F13DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F13DATA1_FD27_MSK								(0x01UL << CAN_F13DATA1_FD27_POS)		/** Filter bits */
#define CAN_F13DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F13DATA1_FD28_MSK								(0x01UL << CAN_F13DATA1_FD28_POS)		/** Filter bits */
#define CAN_F13DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F13DATA1_FD29_MSK								(0x01UL << CAN_F13DATA1_FD29_POS)		/** Filter bits */
#define CAN_F13DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F13DATA1_FD30_MSK								(0x01UL << CAN_F13DATA1_FD30_POS)		/** Filter bits */
#define CAN_F13DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F13DATA1_FD31_MSK								(0x01UL << CAN_F13DATA1_FD31_POS)		/** Filter bits */
#define CAN_F14DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F14DATA0_FD0_MSK								(0x01UL << CAN_F14DATA0_FD0_POS)		/** Filter bits */
#define CAN_F14DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F14DATA0_FD1_MSK								(0x01UL << CAN_F14DATA0_FD1_POS)		/** Filter bits */
#define CAN_F14DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F14DATA0_FD2_MSK								(0x01UL << CAN_F14DATA0_FD2_POS)		/** Filter bits */
#define CAN_F14DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F14DATA0_FD3_MSK								(0x01UL << CAN_F14DATA0_FD3_POS)		/** Filter bits */
#define CAN_F14DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F14DATA0_FD4_MSK								(0x01UL << CAN_F14DATA0_FD4_POS)		/** Filter bits */
#define CAN_F14DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F14DATA0_FD5_MSK								(0x01UL << CAN_F14DATA0_FD5_POS)		/** Filter bits */
#define CAN_F14DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F14DATA0_FD6_MSK								(0x01UL << CAN_F14DATA0_FD6_POS)		/** Filter bits */
#define CAN_F14DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F14DATA0_FD7_MSK								(0x01UL << CAN_F14DATA0_FD7_POS)		/** Filter bits */
#define CAN_F14DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F14DATA0_FD8_MSK								(0x01UL << CAN_F14DATA0_FD8_POS)		/** Filter bits */
#define CAN_F14DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F14DATA0_FD9_MSK								(0x01UL << CAN_F14DATA0_FD9_POS)		/** Filter bits */
#define CAN_F14DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F14DATA0_FD10_MSK								(0x01UL << CAN_F14DATA0_FD10_POS)		/** Filter bits */
#define CAN_F14DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F14DATA0_FD11_MSK								(0x01UL << CAN_F14DATA0_FD11_POS)		/** Filter bits */
#define CAN_F14DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F14DATA0_FD12_MSK								(0x01UL << CAN_F14DATA0_FD12_POS)		/** Filter bits */
#define CAN_F14DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F14DATA0_FD13_MSK								(0x01UL << CAN_F14DATA0_FD13_POS)		/** Filter bits */
#define CAN_F14DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F14DATA0_FD14_MSK								(0x01UL << CAN_F14DATA0_FD14_POS)		/** Filter bits */
#define CAN_F14DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F14DATA0_FD15_MSK								(0x01UL << CAN_F14DATA0_FD15_POS)		/** Filter bits */
#define CAN_F14DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F14DATA0_FD16_MSK								(0x01UL << CAN_F14DATA0_FD16_POS)		/** Filter bits */
#define CAN_F14DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F14DATA0_FD17_MSK								(0x01UL << CAN_F14DATA0_FD17_POS)		/** Filter bits */
#define CAN_F14DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F14DATA0_FD18_MSK								(0x01UL << CAN_F14DATA0_FD18_POS)		/** Filter bits */
#define CAN_F14DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F14DATA0_FD19_MSK								(0x01UL << CAN_F14DATA0_FD19_POS)		/** Filter bits */
#define CAN_F14DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F14DATA0_FD20_MSK								(0x01UL << CAN_F14DATA0_FD20_POS)		/** Filter bits */
#define CAN_F14DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F14DATA0_FD21_MSK								(0x01UL << CAN_F14DATA0_FD21_POS)		/** Filter bits */
#define CAN_F14DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F14DATA0_FD22_MSK								(0x01UL << CAN_F14DATA0_FD22_POS)		/** Filter bits */
#define CAN_F14DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F14DATA0_FD23_MSK								(0x01UL << CAN_F14DATA0_FD23_POS)		/** Filter bits */
#define CAN_F14DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F14DATA0_FD24_MSK								(0x01UL << CAN_F14DATA0_FD24_POS)		/** Filter bits */
#define CAN_F14DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F14DATA0_FD25_MSK								(0x01UL << CAN_F14DATA0_FD25_POS)		/** Filter bits */
#define CAN_F14DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F14DATA0_FD26_MSK								(0x01UL << CAN_F14DATA0_FD26_POS)		/** Filter bits */
#define CAN_F14DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F14DATA0_FD27_MSK								(0x01UL << CAN_F14DATA0_FD27_POS)		/** Filter bits */
#define CAN_F14DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F14DATA0_FD28_MSK								(0x01UL << CAN_F14DATA0_FD28_POS)		/** Filter bits */
#define CAN_F14DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F14DATA0_FD29_MSK								(0x01UL << CAN_F14DATA0_FD29_POS)		/** Filter bits */
#define CAN_F14DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F14DATA0_FD30_MSK								(0x01UL << CAN_F14DATA0_FD30_POS)		/** Filter bits */
#define CAN_F14DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F14DATA0_FD31_MSK								(0x01UL << CAN_F14DATA0_FD31_POS)		/** Filter bits */
#define CAN_F14DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F14DATA1_FD0_MSK								(0x01UL << CAN_F14DATA1_FD0_POS)		/** Filter bits */
#define CAN_F14DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F14DATA1_FD1_MSK								(0x01UL << CAN_F14DATA1_FD1_POS)		/** Filter bits */
#define CAN_F14DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F14DATA1_FD2_MSK								(0x01UL << CAN_F14DATA1_FD2_POS)		/** Filter bits */
#define CAN_F14DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F14DATA1_FD3_MSK								(0x01UL << CAN_F14DATA1_FD3_POS)		/** Filter bits */
#define CAN_F14DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F14DATA1_FD4_MSK								(0x01UL << CAN_F14DATA1_FD4_POS)		/** Filter bits */
#define CAN_F14DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F14DATA1_FD5_MSK								(0x01UL << CAN_F14DATA1_FD5_POS)		/** Filter bits */
#define CAN_F14DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F14DATA1_FD6_MSK								(0x01UL << CAN_F14DATA1_FD6_POS)		/** Filter bits */
#define CAN_F14DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F14DATA1_FD7_MSK								(0x01UL << CAN_F14DATA1_FD7_POS)		/** Filter bits */
#define CAN_F14DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F14DATA1_FD8_MSK								(0x01UL << CAN_F14DATA1_FD8_POS)		/** Filter bits */
#define CAN_F14DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F14DATA1_FD9_MSK								(0x01UL << CAN_F14DATA1_FD9_POS)		/** Filter bits */
#define CAN_F14DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F14DATA1_FD10_MSK								(0x01UL << CAN_F14DATA1_FD10_POS)		/** Filter bits */
#define CAN_F14DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F14DATA1_FD11_MSK								(0x01UL << CAN_F14DATA1_FD11_POS)		/** Filter bits */
#define CAN_F14DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F14DATA1_FD12_MSK								(0x01UL << CAN_F14DATA1_FD12_POS)		/** Filter bits */
#define CAN_F14DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F14DATA1_FD13_MSK								(0x01UL << CAN_F14DATA1_FD13_POS)		/** Filter bits */
#define CAN_F14DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F14DATA1_FD14_MSK								(0x01UL << CAN_F14DATA1_FD14_POS)		/** Filter bits */
#define CAN_F14DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F14DATA1_FD15_MSK								(0x01UL << CAN_F14DATA1_FD15_POS)		/** Filter bits */
#define CAN_F14DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F14DATA1_FD16_MSK								(0x01UL << CAN_F14DATA1_FD16_POS)		/** Filter bits */
#define CAN_F14DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F14DATA1_FD17_MSK								(0x01UL << CAN_F14DATA1_FD17_POS)		/** Filter bits */
#define CAN_F14DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F14DATA1_FD18_MSK								(0x01UL << CAN_F14DATA1_FD18_POS)		/** Filter bits */
#define CAN_F14DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F14DATA1_FD19_MSK								(0x01UL << CAN_F14DATA1_FD19_POS)		/** Filter bits */
#define CAN_F14DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F14DATA1_FD20_MSK								(0x01UL << CAN_F14DATA1_FD20_POS)		/** Filter bits */
#define CAN_F14DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F14DATA1_FD21_MSK								(0x01UL << CAN_F14DATA1_FD21_POS)		/** Filter bits */
#define CAN_F14DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F14DATA1_FD22_MSK								(0x01UL << CAN_F14DATA1_FD22_POS)		/** Filter bits */
#define CAN_F14DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F14DATA1_FD23_MSK								(0x01UL << CAN_F14DATA1_FD23_POS)		/** Filter bits */
#define CAN_F14DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F14DATA1_FD24_MSK								(0x01UL << CAN_F14DATA1_FD24_POS)		/** Filter bits */
#define CAN_F14DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F14DATA1_FD25_MSK								(0x01UL << CAN_F14DATA1_FD25_POS)		/** Filter bits */
#define CAN_F14DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F14DATA1_FD26_MSK								(0x01UL << CAN_F14DATA1_FD26_POS)		/** Filter bits */
#define CAN_F14DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F14DATA1_FD27_MSK								(0x01UL << CAN_F14DATA1_FD27_POS)		/** Filter bits */
#define CAN_F14DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F14DATA1_FD28_MSK								(0x01UL << CAN_F14DATA1_FD28_POS)		/** Filter bits */
#define CAN_F14DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F14DATA1_FD29_MSK								(0x01UL << CAN_F14DATA1_FD29_POS)		/** Filter bits */
#define CAN_F14DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F14DATA1_FD30_MSK								(0x01UL << CAN_F14DATA1_FD30_POS)		/** Filter bits */
#define CAN_F14DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F14DATA1_FD31_MSK								(0x01UL << CAN_F14DATA1_FD31_POS)		/** Filter bits */
#define CAN_F15DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F15DATA0_FD0_MSK								(0x01UL << CAN_F15DATA0_FD0_POS)		/** Filter bits */
#define CAN_F15DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F15DATA0_FD1_MSK								(0x01UL << CAN_F15DATA0_FD1_POS)		/** Filter bits */
#define CAN_F15DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F15DATA0_FD2_MSK								(0x01UL << CAN_F15DATA0_FD2_POS)		/** Filter bits */
#define CAN_F15DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F15DATA0_FD3_MSK								(0x01UL << CAN_F15DATA0_FD3_POS)		/** Filter bits */
#define CAN_F15DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F15DATA0_FD4_MSK								(0x01UL << CAN_F15DATA0_FD4_POS)		/** Filter bits */
#define CAN_F15DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F15DATA0_FD5_MSK								(0x01UL << CAN_F15DATA0_FD5_POS)		/** Filter bits */
#define CAN_F15DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F15DATA0_FD6_MSK								(0x01UL << CAN_F15DATA0_FD6_POS)		/** Filter bits */
#define CAN_F15DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F15DATA0_FD7_MSK								(0x01UL << CAN_F15DATA0_FD7_POS)		/** Filter bits */
#define CAN_F15DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F15DATA0_FD8_MSK								(0x01UL << CAN_F15DATA0_FD8_POS)		/** Filter bits */
#define CAN_F15DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F15DATA0_FD9_MSK								(0x01UL << CAN_F15DATA0_FD9_POS)		/** Filter bits */
#define CAN_F15DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F15DATA0_FD10_MSK								(0x01UL << CAN_F15DATA0_FD10_POS)		/** Filter bits */
#define CAN_F15DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F15DATA0_FD11_MSK								(0x01UL << CAN_F15DATA0_FD11_POS)		/** Filter bits */
#define CAN_F15DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F15DATA0_FD12_MSK								(0x01UL << CAN_F15DATA0_FD12_POS)		/** Filter bits */
#define CAN_F15DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F15DATA0_FD13_MSK								(0x01UL << CAN_F15DATA0_FD13_POS)		/** Filter bits */
#define CAN_F15DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F15DATA0_FD14_MSK								(0x01UL << CAN_F15DATA0_FD14_POS)		/** Filter bits */
#define CAN_F15DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F15DATA0_FD15_MSK								(0x01UL << CAN_F15DATA0_FD15_POS)		/** Filter bits */
#define CAN_F15DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F15DATA0_FD16_MSK								(0x01UL << CAN_F15DATA0_FD16_POS)		/** Filter bits */
#define CAN_F15DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F15DATA0_FD17_MSK								(0x01UL << CAN_F15DATA0_FD17_POS)		/** Filter bits */
#define CAN_F15DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F15DATA0_FD18_MSK								(0x01UL << CAN_F15DATA0_FD18_POS)		/** Filter bits */
#define CAN_F15DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F15DATA0_FD19_MSK								(0x01UL << CAN_F15DATA0_FD19_POS)		/** Filter bits */
#define CAN_F15DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F15DATA0_FD20_MSK								(0x01UL << CAN_F15DATA0_FD20_POS)		/** Filter bits */
#define CAN_F15DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F15DATA0_FD21_MSK								(0x01UL << CAN_F15DATA0_FD21_POS)		/** Filter bits */
#define CAN_F15DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F15DATA0_FD22_MSK								(0x01UL << CAN_F15DATA0_FD22_POS)		/** Filter bits */
#define CAN_F15DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F15DATA0_FD23_MSK								(0x01UL << CAN_F15DATA0_FD23_POS)		/** Filter bits */
#define CAN_F15DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F15DATA0_FD24_MSK								(0x01UL << CAN_F15DATA0_FD24_POS)		/** Filter bits */
#define CAN_F15DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F15DATA0_FD25_MSK								(0x01UL << CAN_F15DATA0_FD25_POS)		/** Filter bits */
#define CAN_F15DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F15DATA0_FD26_MSK								(0x01UL << CAN_F15DATA0_FD26_POS)		/** Filter bits */
#define CAN_F15DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F15DATA0_FD27_MSK								(0x01UL << CAN_F15DATA0_FD27_POS)		/** Filter bits */
#define CAN_F15DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F15DATA0_FD28_MSK								(0x01UL << CAN_F15DATA0_FD28_POS)		/** Filter bits */
#define CAN_F15DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F15DATA0_FD29_MSK								(0x01UL << CAN_F15DATA0_FD29_POS)		/** Filter bits */
#define CAN_F15DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F15DATA0_FD30_MSK								(0x01UL << CAN_F15DATA0_FD30_POS)		/** Filter bits */
#define CAN_F15DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F15DATA0_FD31_MSK								(0x01UL << CAN_F15DATA0_FD31_POS)		/** Filter bits */
#define CAN_F15DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F15DATA1_FD0_MSK								(0x01UL << CAN_F15DATA1_FD0_POS)		/** Filter bits */
#define CAN_F15DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F15DATA1_FD1_MSK								(0x01UL << CAN_F15DATA1_FD1_POS)		/** Filter bits */
#define CAN_F15DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F15DATA1_FD2_MSK								(0x01UL << CAN_F15DATA1_FD2_POS)		/** Filter bits */
#define CAN_F15DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F15DATA1_FD3_MSK								(0x01UL << CAN_F15DATA1_FD3_POS)		/** Filter bits */
#define CAN_F15DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F15DATA1_FD4_MSK								(0x01UL << CAN_F15DATA1_FD4_POS)		/** Filter bits */
#define CAN_F15DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F15DATA1_FD5_MSK								(0x01UL << CAN_F15DATA1_FD5_POS)		/** Filter bits */
#define CAN_F15DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F15DATA1_FD6_MSK								(0x01UL << CAN_F15DATA1_FD6_POS)		/** Filter bits */
#define CAN_F15DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F15DATA1_FD7_MSK								(0x01UL << CAN_F15DATA1_FD7_POS)		/** Filter bits */
#define CAN_F15DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F15DATA1_FD8_MSK								(0x01UL << CAN_F15DATA1_FD8_POS)		/** Filter bits */
#define CAN_F15DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F15DATA1_FD9_MSK								(0x01UL << CAN_F15DATA1_FD9_POS)		/** Filter bits */
#define CAN_F15DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F15DATA1_FD10_MSK								(0x01UL << CAN_F15DATA1_FD10_POS)		/** Filter bits */
#define CAN_F15DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F15DATA1_FD11_MSK								(0x01UL << CAN_F15DATA1_FD11_POS)		/** Filter bits */
#define CAN_F15DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F15DATA1_FD12_MSK								(0x01UL << CAN_F15DATA1_FD12_POS)		/** Filter bits */
#define CAN_F15DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F15DATA1_FD13_MSK								(0x01UL << CAN_F15DATA1_FD13_POS)		/** Filter bits */
#define CAN_F15DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F15DATA1_FD14_MSK								(0x01UL << CAN_F15DATA1_FD14_POS)		/** Filter bits */
#define CAN_F15DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F15DATA1_FD15_MSK								(0x01UL << CAN_F15DATA1_FD15_POS)		/** Filter bits */
#define CAN_F15DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F15DATA1_FD16_MSK								(0x01UL << CAN_F15DATA1_FD16_POS)		/** Filter bits */
#define CAN_F15DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F15DATA1_FD17_MSK								(0x01UL << CAN_F15DATA1_FD17_POS)		/** Filter bits */
#define CAN_F15DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F15DATA1_FD18_MSK								(0x01UL << CAN_F15DATA1_FD18_POS)		/** Filter bits */
#define CAN_F15DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F15DATA1_FD19_MSK								(0x01UL << CAN_F15DATA1_FD19_POS)		/** Filter bits */
#define CAN_F15DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F15DATA1_FD20_MSK								(0x01UL << CAN_F15DATA1_FD20_POS)		/** Filter bits */
#define CAN_F15DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F15DATA1_FD21_MSK								(0x01UL << CAN_F15DATA1_FD21_POS)		/** Filter bits */
#define CAN_F15DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F15DATA1_FD22_MSK								(0x01UL << CAN_F15DATA1_FD22_POS)		/** Filter bits */
#define CAN_F15DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F15DATA1_FD23_MSK								(0x01UL << CAN_F15DATA1_FD23_POS)		/** Filter bits */
#define CAN_F15DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F15DATA1_FD24_MSK								(0x01UL << CAN_F15DATA1_FD24_POS)		/** Filter bits */
#define CAN_F15DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F15DATA1_FD25_MSK								(0x01UL << CAN_F15DATA1_FD25_POS)		/** Filter bits */
#define CAN_F15DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F15DATA1_FD26_MSK								(0x01UL << CAN_F15DATA1_FD26_POS)		/** Filter bits */
#define CAN_F15DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F15DATA1_FD27_MSK								(0x01UL << CAN_F15DATA1_FD27_POS)		/** Filter bits */
#define CAN_F15DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F15DATA1_FD28_MSK								(0x01UL << CAN_F15DATA1_FD28_POS)		/** Filter bits */
#define CAN_F15DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F15DATA1_FD29_MSK								(0x01UL << CAN_F15DATA1_FD29_POS)		/** Filter bits */
#define CAN_F15DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F15DATA1_FD30_MSK								(0x01UL << CAN_F15DATA1_FD30_POS)		/** Filter bits */
#define CAN_F15DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F15DATA1_FD31_MSK								(0x01UL << CAN_F15DATA1_FD31_POS)		/** Filter bits */
#define CAN_F16DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F16DATA0_FD0_MSK								(0x01UL << CAN_F16DATA0_FD0_POS)		/** Filter bits */
#define CAN_F16DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F16DATA0_FD1_MSK								(0x01UL << CAN_F16DATA0_FD1_POS)		/** Filter bits */
#define CAN_F16DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F16DATA0_FD2_MSK								(0x01UL << CAN_F16DATA0_FD2_POS)		/** Filter bits */
#define CAN_F16DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F16DATA0_FD3_MSK								(0x01UL << CAN_F16DATA0_FD3_POS)		/** Filter bits */
#define CAN_F16DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F16DATA0_FD4_MSK								(0x01UL << CAN_F16DATA0_FD4_POS)		/** Filter bits */
#define CAN_F16DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F16DATA0_FD5_MSK								(0x01UL << CAN_F16DATA0_FD5_POS)		/** Filter bits */
#define CAN_F16DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F16DATA0_FD6_MSK								(0x01UL << CAN_F16DATA0_FD6_POS)		/** Filter bits */
#define CAN_F16DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F16DATA0_FD7_MSK								(0x01UL << CAN_F16DATA0_FD7_POS)		/** Filter bits */
#define CAN_F16DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F16DATA0_FD8_MSK								(0x01UL << CAN_F16DATA0_FD8_POS)		/** Filter bits */
#define CAN_F16DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F16DATA0_FD9_MSK								(0x01UL << CAN_F16DATA0_FD9_POS)		/** Filter bits */
#define CAN_F16DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F16DATA0_FD10_MSK								(0x01UL << CAN_F16DATA0_FD10_POS)		/** Filter bits */
#define CAN_F16DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F16DATA0_FD11_MSK								(0x01UL << CAN_F16DATA0_FD11_POS)		/** Filter bits */
#define CAN_F16DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F16DATA0_FD12_MSK								(0x01UL << CAN_F16DATA0_FD12_POS)		/** Filter bits */
#define CAN_F16DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F16DATA0_FD13_MSK								(0x01UL << CAN_F16DATA0_FD13_POS)		/** Filter bits */
#define CAN_F16DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F16DATA0_FD14_MSK								(0x01UL << CAN_F16DATA0_FD14_POS)		/** Filter bits */
#define CAN_F16DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F16DATA0_FD15_MSK								(0x01UL << CAN_F16DATA0_FD15_POS)		/** Filter bits */
#define CAN_F16DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F16DATA0_FD16_MSK								(0x01UL << CAN_F16DATA0_FD16_POS)		/** Filter bits */
#define CAN_F16DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F16DATA0_FD17_MSK								(0x01UL << CAN_F16DATA0_FD17_POS)		/** Filter bits */
#define CAN_F16DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F16DATA0_FD18_MSK								(0x01UL << CAN_F16DATA0_FD18_POS)		/** Filter bits */
#define CAN_F16DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F16DATA0_FD19_MSK								(0x01UL << CAN_F16DATA0_FD19_POS)		/** Filter bits */
#define CAN_F16DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F16DATA0_FD20_MSK								(0x01UL << CAN_F16DATA0_FD20_POS)		/** Filter bits */
#define CAN_F16DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F16DATA0_FD21_MSK								(0x01UL << CAN_F16DATA0_FD21_POS)		/** Filter bits */
#define CAN_F16DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F16DATA0_FD22_MSK								(0x01UL << CAN_F16DATA0_FD22_POS)		/** Filter bits */
#define CAN_F16DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F16DATA0_FD23_MSK								(0x01UL << CAN_F16DATA0_FD23_POS)		/** Filter bits */
#define CAN_F16DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F16DATA0_FD24_MSK								(0x01UL << CAN_F16DATA0_FD24_POS)		/** Filter bits */
#define CAN_F16DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F16DATA0_FD25_MSK								(0x01UL << CAN_F16DATA0_FD25_POS)		/** Filter bits */
#define CAN_F16DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F16DATA0_FD26_MSK								(0x01UL << CAN_F16DATA0_FD26_POS)		/** Filter bits */
#define CAN_F16DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F16DATA0_FD27_MSK								(0x01UL << CAN_F16DATA0_FD27_POS)		/** Filter bits */
#define CAN_F16DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F16DATA0_FD28_MSK								(0x01UL << CAN_F16DATA0_FD28_POS)		/** Filter bits */
#define CAN_F16DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F16DATA0_FD29_MSK								(0x01UL << CAN_F16DATA0_FD29_POS)		/** Filter bits */
#define CAN_F16DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F16DATA0_FD30_MSK								(0x01UL << CAN_F16DATA0_FD30_POS)		/** Filter bits */
#define CAN_F16DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F16DATA0_FD31_MSK								(0x01UL << CAN_F16DATA0_FD31_POS)		/** Filter bits */
#define CAN_F16DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F16DATA1_FD0_MSK								(0x01UL << CAN_F16DATA1_FD0_POS)		/** Filter bits */
#define CAN_F16DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F16DATA1_FD1_MSK								(0x01UL << CAN_F16DATA1_FD1_POS)		/** Filter bits */
#define CAN_F16DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F16DATA1_FD2_MSK								(0x01UL << CAN_F16DATA1_FD2_POS)		/** Filter bits */
#define CAN_F16DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F16DATA1_FD3_MSK								(0x01UL << CAN_F16DATA1_FD3_POS)		/** Filter bits */
#define CAN_F16DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F16DATA1_FD4_MSK								(0x01UL << CAN_F16DATA1_FD4_POS)		/** Filter bits */
#define CAN_F16DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F16DATA1_FD5_MSK								(0x01UL << CAN_F16DATA1_FD5_POS)		/** Filter bits */
#define CAN_F16DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F16DATA1_FD6_MSK								(0x01UL << CAN_F16DATA1_FD6_POS)		/** Filter bits */
#define CAN_F16DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F16DATA1_FD7_MSK								(0x01UL << CAN_F16DATA1_FD7_POS)		/** Filter bits */
#define CAN_F16DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F16DATA1_FD8_MSK								(0x01UL << CAN_F16DATA1_FD8_POS)		/** Filter bits */
#define CAN_F16DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F16DATA1_FD9_MSK								(0x01UL << CAN_F16DATA1_FD9_POS)		/** Filter bits */
#define CAN_F16DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F16DATA1_FD10_MSK								(0x01UL << CAN_F16DATA1_FD10_POS)		/** Filter bits */
#define CAN_F16DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F16DATA1_FD11_MSK								(0x01UL << CAN_F16DATA1_FD11_POS)		/** Filter bits */
#define CAN_F16DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F16DATA1_FD12_MSK								(0x01UL << CAN_F16DATA1_FD12_POS)		/** Filter bits */
#define CAN_F16DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F16DATA1_FD13_MSK								(0x01UL << CAN_F16DATA1_FD13_POS)		/** Filter bits */
#define CAN_F16DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F16DATA1_FD14_MSK								(0x01UL << CAN_F16DATA1_FD14_POS)		/** Filter bits */
#define CAN_F16DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F16DATA1_FD15_MSK								(0x01UL << CAN_F16DATA1_FD15_POS)		/** Filter bits */
#define CAN_F16DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F16DATA1_FD16_MSK								(0x01UL << CAN_F16DATA1_FD16_POS)		/** Filter bits */
#define CAN_F16DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F16DATA1_FD17_MSK								(0x01UL << CAN_F16DATA1_FD17_POS)		/** Filter bits */
#define CAN_F16DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F16DATA1_FD18_MSK								(0x01UL << CAN_F16DATA1_FD18_POS)		/** Filter bits */
#define CAN_F16DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F16DATA1_FD19_MSK								(0x01UL << CAN_F16DATA1_FD19_POS)		/** Filter bits */
#define CAN_F16DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F16DATA1_FD20_MSK								(0x01UL << CAN_F16DATA1_FD20_POS)		/** Filter bits */
#define CAN_F16DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F16DATA1_FD21_MSK								(0x01UL << CAN_F16DATA1_FD21_POS)		/** Filter bits */
#define CAN_F16DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F16DATA1_FD22_MSK								(0x01UL << CAN_F16DATA1_FD22_POS)		/** Filter bits */
#define CAN_F16DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F16DATA1_FD23_MSK								(0x01UL << CAN_F16DATA1_FD23_POS)		/** Filter bits */
#define CAN_F16DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F16DATA1_FD24_MSK								(0x01UL << CAN_F16DATA1_FD24_POS)		/** Filter bits */
#define CAN_F16DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F16DATA1_FD25_MSK								(0x01UL << CAN_F16DATA1_FD25_POS)		/** Filter bits */
#define CAN_F16DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F16DATA1_FD26_MSK								(0x01UL << CAN_F16DATA1_FD26_POS)		/** Filter bits */
#define CAN_F16DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F16DATA1_FD27_MSK								(0x01UL << CAN_F16DATA1_FD27_POS)		/** Filter bits */
#define CAN_F16DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F16DATA1_FD28_MSK								(0x01UL << CAN_F16DATA1_FD28_POS)		/** Filter bits */
#define CAN_F16DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F16DATA1_FD29_MSK								(0x01UL << CAN_F16DATA1_FD29_POS)		/** Filter bits */
#define CAN_F16DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F16DATA1_FD30_MSK								(0x01UL << CAN_F16DATA1_FD30_POS)		/** Filter bits */
#define CAN_F16DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F16DATA1_FD31_MSK								(0x01UL << CAN_F16DATA1_FD31_POS)		/** Filter bits */
#define CAN_F17DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F17DATA0_FD0_MSK								(0x01UL << CAN_F17DATA0_FD0_POS)		/** Filter bits */
#define CAN_F17DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F17DATA0_FD1_MSK								(0x01UL << CAN_F17DATA0_FD1_POS)		/** Filter bits */
#define CAN_F17DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F17DATA0_FD2_MSK								(0x01UL << CAN_F17DATA0_FD2_POS)		/** Filter bits */
#define CAN_F17DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F17DATA0_FD3_MSK								(0x01UL << CAN_F17DATA0_FD3_POS)		/** Filter bits */
#define CAN_F17DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F17DATA0_FD4_MSK								(0x01UL << CAN_F17DATA0_FD4_POS)		/** Filter bits */
#define CAN_F17DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F17DATA0_FD5_MSK								(0x01UL << CAN_F17DATA0_FD5_POS)		/** Filter bits */
#define CAN_F17DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F17DATA0_FD6_MSK								(0x01UL << CAN_F17DATA0_FD6_POS)		/** Filter bits */
#define CAN_F17DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F17DATA0_FD7_MSK								(0x01UL << CAN_F17DATA0_FD7_POS)		/** Filter bits */
#define CAN_F17DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F17DATA0_FD8_MSK								(0x01UL << CAN_F17DATA0_FD8_POS)		/** Filter bits */
#define CAN_F17DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F17DATA0_FD9_MSK								(0x01UL << CAN_F17DATA0_FD9_POS)		/** Filter bits */
#define CAN_F17DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F17DATA0_FD10_MSK								(0x01UL << CAN_F17DATA0_FD10_POS)		/** Filter bits */
#define CAN_F17DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F17DATA0_FD11_MSK								(0x01UL << CAN_F17DATA0_FD11_POS)		/** Filter bits */
#define CAN_F17DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F17DATA0_FD12_MSK								(0x01UL << CAN_F17DATA0_FD12_POS)		/** Filter bits */
#define CAN_F17DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F17DATA0_FD13_MSK								(0x01UL << CAN_F17DATA0_FD13_POS)		/** Filter bits */
#define CAN_F17DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F17DATA0_FD14_MSK								(0x01UL << CAN_F17DATA0_FD14_POS)		/** Filter bits */
#define CAN_F17DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F17DATA0_FD15_MSK								(0x01UL << CAN_F17DATA0_FD15_POS)		/** Filter bits */
#define CAN_F17DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F17DATA0_FD16_MSK								(0x01UL << CAN_F17DATA0_FD16_POS)		/** Filter bits */
#define CAN_F17DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F17DATA0_FD17_MSK								(0x01UL << CAN_F17DATA0_FD17_POS)		/** Filter bits */
#define CAN_F17DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F17DATA0_FD18_MSK								(0x01UL << CAN_F17DATA0_FD18_POS)		/** Filter bits */
#define CAN_F17DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F17DATA0_FD19_MSK								(0x01UL << CAN_F17DATA0_FD19_POS)		/** Filter bits */
#define CAN_F17DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F17DATA0_FD20_MSK								(0x01UL << CAN_F17DATA0_FD20_POS)		/** Filter bits */
#define CAN_F17DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F17DATA0_FD21_MSK								(0x01UL << CAN_F17DATA0_FD21_POS)		/** Filter bits */
#define CAN_F17DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F17DATA0_FD22_MSK								(0x01UL << CAN_F17DATA0_FD22_POS)		/** Filter bits */
#define CAN_F17DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F17DATA0_FD23_MSK								(0x01UL << CAN_F17DATA0_FD23_POS)		/** Filter bits */
#define CAN_F17DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F17DATA0_FD24_MSK								(0x01UL << CAN_F17DATA0_FD24_POS)		/** Filter bits */
#define CAN_F17DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F17DATA0_FD25_MSK								(0x01UL << CAN_F17DATA0_FD25_POS)		/** Filter bits */
#define CAN_F17DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F17DATA0_FD26_MSK								(0x01UL << CAN_F17DATA0_FD26_POS)		/** Filter bits */
#define CAN_F17DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F17DATA0_FD27_MSK								(0x01UL << CAN_F17DATA0_FD27_POS)		/** Filter bits */
#define CAN_F17DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F17DATA0_FD28_MSK								(0x01UL << CAN_F17DATA0_FD28_POS)		/** Filter bits */
#define CAN_F17DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F17DATA0_FD29_MSK								(0x01UL << CAN_F17DATA0_FD29_POS)		/** Filter bits */
#define CAN_F17DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F17DATA0_FD30_MSK								(0x01UL << CAN_F17DATA0_FD30_POS)		/** Filter bits */
#define CAN_F17DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F17DATA0_FD31_MSK								(0x01UL << CAN_F17DATA0_FD31_POS)		/** Filter bits */
#define CAN_F17DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F17DATA1_FD0_MSK								(0x01UL << CAN_F17DATA1_FD0_POS)		/** Filter bits */
#define CAN_F17DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F17DATA1_FD1_MSK								(0x01UL << CAN_F17DATA1_FD1_POS)		/** Filter bits */
#define CAN_F17DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F17DATA1_FD2_MSK								(0x01UL << CAN_F17DATA1_FD2_POS)		/** Filter bits */
#define CAN_F17DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F17DATA1_FD3_MSK								(0x01UL << CAN_F17DATA1_FD3_POS)		/** Filter bits */
#define CAN_F17DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F17DATA1_FD4_MSK								(0x01UL << CAN_F17DATA1_FD4_POS)		/** Filter bits */
#define CAN_F17DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F17DATA1_FD5_MSK								(0x01UL << CAN_F17DATA1_FD5_POS)		/** Filter bits */
#define CAN_F17DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F17DATA1_FD6_MSK								(0x01UL << CAN_F17DATA1_FD6_POS)		/** Filter bits */
#define CAN_F17DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F17DATA1_FD7_MSK								(0x01UL << CAN_F17DATA1_FD7_POS)		/** Filter bits */
#define CAN_F17DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F17DATA1_FD8_MSK								(0x01UL << CAN_F17DATA1_FD8_POS)		/** Filter bits */
#define CAN_F17DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F17DATA1_FD9_MSK								(0x01UL << CAN_F17DATA1_FD9_POS)		/** Filter bits */
#define CAN_F17DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F17DATA1_FD10_MSK								(0x01UL << CAN_F17DATA1_FD10_POS)		/** Filter bits */
#define CAN_F17DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F17DATA1_FD11_MSK								(0x01UL << CAN_F17DATA1_FD11_POS)		/** Filter bits */
#define CAN_F17DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F17DATA1_FD12_MSK								(0x01UL << CAN_F17DATA1_FD12_POS)		/** Filter bits */
#define CAN_F17DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F17DATA1_FD13_MSK								(0x01UL << CAN_F17DATA1_FD13_POS)		/** Filter bits */
#define CAN_F17DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F17DATA1_FD14_MSK								(0x01UL << CAN_F17DATA1_FD14_POS)		/** Filter bits */
#define CAN_F17DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F17DATA1_FD15_MSK								(0x01UL << CAN_F17DATA1_FD15_POS)		/** Filter bits */
#define CAN_F17DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F17DATA1_FD16_MSK								(0x01UL << CAN_F17DATA1_FD16_POS)		/** Filter bits */
#define CAN_F17DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F17DATA1_FD17_MSK								(0x01UL << CAN_F17DATA1_FD17_POS)		/** Filter bits */
#define CAN_F17DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F17DATA1_FD18_MSK								(0x01UL << CAN_F17DATA1_FD18_POS)		/** Filter bits */
#define CAN_F17DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F17DATA1_FD19_MSK								(0x01UL << CAN_F17DATA1_FD19_POS)		/** Filter bits */
#define CAN_F17DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F17DATA1_FD20_MSK								(0x01UL << CAN_F17DATA1_FD20_POS)		/** Filter bits */
#define CAN_F17DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F17DATA1_FD21_MSK								(0x01UL << CAN_F17DATA1_FD21_POS)		/** Filter bits */
#define CAN_F17DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F17DATA1_FD22_MSK								(0x01UL << CAN_F17DATA1_FD22_POS)		/** Filter bits */
#define CAN_F17DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F17DATA1_FD23_MSK								(0x01UL << CAN_F17DATA1_FD23_POS)		/** Filter bits */
#define CAN_F17DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F17DATA1_FD24_MSK								(0x01UL << CAN_F17DATA1_FD24_POS)		/** Filter bits */
#define CAN_F17DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F17DATA1_FD25_MSK								(0x01UL << CAN_F17DATA1_FD25_POS)		/** Filter bits */
#define CAN_F17DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F17DATA1_FD26_MSK								(0x01UL << CAN_F17DATA1_FD26_POS)		/** Filter bits */
#define CAN_F17DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F17DATA1_FD27_MSK								(0x01UL << CAN_F17DATA1_FD27_POS)		/** Filter bits */
#define CAN_F17DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F17DATA1_FD28_MSK								(0x01UL << CAN_F17DATA1_FD28_POS)		/** Filter bits */
#define CAN_F17DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F17DATA1_FD29_MSK								(0x01UL << CAN_F17DATA1_FD29_POS)		/** Filter bits */
#define CAN_F17DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F17DATA1_FD30_MSK								(0x01UL << CAN_F17DATA1_FD30_POS)		/** Filter bits */
#define CAN_F17DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F17DATA1_FD31_MSK								(0x01UL << CAN_F17DATA1_FD31_POS)		/** Filter bits */
#define CAN_F18DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F18DATA0_FD0_MSK								(0x01UL << CAN_F18DATA0_FD0_POS)		/** Filter bits */
#define CAN_F18DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F18DATA0_FD1_MSK								(0x01UL << CAN_F18DATA0_FD1_POS)		/** Filter bits */
#define CAN_F18DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F18DATA0_FD2_MSK								(0x01UL << CAN_F18DATA0_FD2_POS)		/** Filter bits */
#define CAN_F18DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F18DATA0_FD3_MSK								(0x01UL << CAN_F18DATA0_FD3_POS)		/** Filter bits */
#define CAN_F18DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F18DATA0_FD4_MSK								(0x01UL << CAN_F18DATA0_FD4_POS)		/** Filter bits */
#define CAN_F18DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F18DATA0_FD5_MSK								(0x01UL << CAN_F18DATA0_FD5_POS)		/** Filter bits */
#define CAN_F18DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F18DATA0_FD6_MSK								(0x01UL << CAN_F18DATA0_FD6_POS)		/** Filter bits */
#define CAN_F18DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F18DATA0_FD7_MSK								(0x01UL << CAN_F18DATA0_FD7_POS)		/** Filter bits */
#define CAN_F18DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F18DATA0_FD8_MSK								(0x01UL << CAN_F18DATA0_FD8_POS)		/** Filter bits */
#define CAN_F18DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F18DATA0_FD9_MSK								(0x01UL << CAN_F18DATA0_FD9_POS)		/** Filter bits */
#define CAN_F18DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F18DATA0_FD10_MSK								(0x01UL << CAN_F18DATA0_FD10_POS)		/** Filter bits */
#define CAN_F18DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F18DATA0_FD11_MSK								(0x01UL << CAN_F18DATA0_FD11_POS)		/** Filter bits */
#define CAN_F18DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F18DATA0_FD12_MSK								(0x01UL << CAN_F18DATA0_FD12_POS)		/** Filter bits */
#define CAN_F18DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F18DATA0_FD13_MSK								(0x01UL << CAN_F18DATA0_FD13_POS)		/** Filter bits */
#define CAN_F18DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F18DATA0_FD14_MSK								(0x01UL << CAN_F18DATA0_FD14_POS)		/** Filter bits */
#define CAN_F18DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F18DATA0_FD15_MSK								(0x01UL << CAN_F18DATA0_FD15_POS)		/** Filter bits */
#define CAN_F18DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F18DATA0_FD16_MSK								(0x01UL << CAN_F18DATA0_FD16_POS)		/** Filter bits */
#define CAN_F18DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F18DATA0_FD17_MSK								(0x01UL << CAN_F18DATA0_FD17_POS)		/** Filter bits */
#define CAN_F18DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F18DATA0_FD18_MSK								(0x01UL << CAN_F18DATA0_FD18_POS)		/** Filter bits */
#define CAN_F18DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F18DATA0_FD19_MSK								(0x01UL << CAN_F18DATA0_FD19_POS)		/** Filter bits */
#define CAN_F18DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F18DATA0_FD20_MSK								(0x01UL << CAN_F18DATA0_FD20_POS)		/** Filter bits */
#define CAN_F18DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F18DATA0_FD21_MSK								(0x01UL << CAN_F18DATA0_FD21_POS)		/** Filter bits */
#define CAN_F18DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F18DATA0_FD22_MSK								(0x01UL << CAN_F18DATA0_FD22_POS)		/** Filter bits */
#define CAN_F18DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F18DATA0_FD23_MSK								(0x01UL << CAN_F18DATA0_FD23_POS)		/** Filter bits */
#define CAN_F18DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F18DATA0_FD24_MSK								(0x01UL << CAN_F18DATA0_FD24_POS)		/** Filter bits */
#define CAN_F18DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F18DATA0_FD25_MSK								(0x01UL << CAN_F18DATA0_FD25_POS)		/** Filter bits */
#define CAN_F18DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F18DATA0_FD26_MSK								(0x01UL << CAN_F18DATA0_FD26_POS)		/** Filter bits */
#define CAN_F18DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F18DATA0_FD27_MSK								(0x01UL << CAN_F18DATA0_FD27_POS)		/** Filter bits */
#define CAN_F18DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F18DATA0_FD28_MSK								(0x01UL << CAN_F18DATA0_FD28_POS)		/** Filter bits */
#define CAN_F18DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F18DATA0_FD29_MSK								(0x01UL << CAN_F18DATA0_FD29_POS)		/** Filter bits */
#define CAN_F18DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F18DATA0_FD30_MSK								(0x01UL << CAN_F18DATA0_FD30_POS)		/** Filter bits */
#define CAN_F18DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F18DATA0_FD31_MSK								(0x01UL << CAN_F18DATA0_FD31_POS)		/** Filter bits */
#define CAN_F18DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F18DATA1_FD0_MSK								(0x01UL << CAN_F18DATA1_FD0_POS)		/** Filter bits */
#define CAN_F18DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F18DATA1_FD1_MSK								(0x01UL << CAN_F18DATA1_FD1_POS)		/** Filter bits */
#define CAN_F18DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F18DATA1_FD2_MSK								(0x01UL << CAN_F18DATA1_FD2_POS)		/** Filter bits */
#define CAN_F18DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F18DATA1_FD3_MSK								(0x01UL << CAN_F18DATA1_FD3_POS)		/** Filter bits */
#define CAN_F18DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F18DATA1_FD4_MSK								(0x01UL << CAN_F18DATA1_FD4_POS)		/** Filter bits */
#define CAN_F18DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F18DATA1_FD5_MSK								(0x01UL << CAN_F18DATA1_FD5_POS)		/** Filter bits */
#define CAN_F18DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F18DATA1_FD6_MSK								(0x01UL << CAN_F18DATA1_FD6_POS)		/** Filter bits */
#define CAN_F18DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F18DATA1_FD7_MSK								(0x01UL << CAN_F18DATA1_FD7_POS)		/** Filter bits */
#define CAN_F18DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F18DATA1_FD8_MSK								(0x01UL << CAN_F18DATA1_FD8_POS)		/** Filter bits */
#define CAN_F18DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F18DATA1_FD9_MSK								(0x01UL << CAN_F18DATA1_FD9_POS)		/** Filter bits */
#define CAN_F18DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F18DATA1_FD10_MSK								(0x01UL << CAN_F18DATA1_FD10_POS)		/** Filter bits */
#define CAN_F18DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F18DATA1_FD11_MSK								(0x01UL << CAN_F18DATA1_FD11_POS)		/** Filter bits */
#define CAN_F18DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F18DATA1_FD12_MSK								(0x01UL << CAN_F18DATA1_FD12_POS)		/** Filter bits */
#define CAN_F18DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F18DATA1_FD13_MSK								(0x01UL << CAN_F18DATA1_FD13_POS)		/** Filter bits */
#define CAN_F18DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F18DATA1_FD14_MSK								(0x01UL << CAN_F18DATA1_FD14_POS)		/** Filter bits */
#define CAN_F18DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F18DATA1_FD15_MSK								(0x01UL << CAN_F18DATA1_FD15_POS)		/** Filter bits */
#define CAN_F18DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F18DATA1_FD16_MSK								(0x01UL << CAN_F18DATA1_FD16_POS)		/** Filter bits */
#define CAN_F18DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F18DATA1_FD17_MSK								(0x01UL << CAN_F18DATA1_FD17_POS)		/** Filter bits */
#define CAN_F18DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F18DATA1_FD18_MSK								(0x01UL << CAN_F18DATA1_FD18_POS)		/** Filter bits */
#define CAN_F18DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F18DATA1_FD19_MSK								(0x01UL << CAN_F18DATA1_FD19_POS)		/** Filter bits */
#define CAN_F18DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F18DATA1_FD20_MSK								(0x01UL << CAN_F18DATA1_FD20_POS)		/** Filter bits */
#define CAN_F18DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F18DATA1_FD21_MSK								(0x01UL << CAN_F18DATA1_FD21_POS)		/** Filter bits */
#define CAN_F18DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F18DATA1_FD22_MSK								(0x01UL << CAN_F18DATA1_FD22_POS)		/** Filter bits */
#define CAN_F18DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F18DATA1_FD23_MSK								(0x01UL << CAN_F18DATA1_FD23_POS)		/** Filter bits */
#define CAN_F18DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F18DATA1_FD24_MSK								(0x01UL << CAN_F18DATA1_FD24_POS)		/** Filter bits */
#define CAN_F18DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F18DATA1_FD25_MSK								(0x01UL << CAN_F18DATA1_FD25_POS)		/** Filter bits */
#define CAN_F18DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F18DATA1_FD26_MSK								(0x01UL << CAN_F18DATA1_FD26_POS)		/** Filter bits */
#define CAN_F18DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F18DATA1_FD27_MSK								(0x01UL << CAN_F18DATA1_FD27_POS)		/** Filter bits */
#define CAN_F18DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F18DATA1_FD28_MSK								(0x01UL << CAN_F18DATA1_FD28_POS)		/** Filter bits */
#define CAN_F18DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F18DATA1_FD29_MSK								(0x01UL << CAN_F18DATA1_FD29_POS)		/** Filter bits */
#define CAN_F18DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F18DATA1_FD30_MSK								(0x01UL << CAN_F18DATA1_FD30_POS)		/** Filter bits */
#define CAN_F18DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F18DATA1_FD31_MSK								(0x01UL << CAN_F18DATA1_FD31_POS)		/** Filter bits */
#define CAN_F19DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F19DATA0_FD0_MSK								(0x01UL << CAN_F19DATA0_FD0_POS)		/** Filter bits */
#define CAN_F19DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F19DATA0_FD1_MSK								(0x01UL << CAN_F19DATA0_FD1_POS)		/** Filter bits */
#define CAN_F19DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F19DATA0_FD2_MSK								(0x01UL << CAN_F19DATA0_FD2_POS)		/** Filter bits */
#define CAN_F19DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F19DATA0_FD3_MSK								(0x01UL << CAN_F19DATA0_FD3_POS)		/** Filter bits */
#define CAN_F19DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F19DATA0_FD4_MSK								(0x01UL << CAN_F19DATA0_FD4_POS)		/** Filter bits */
#define CAN_F19DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F19DATA0_FD5_MSK								(0x01UL << CAN_F19DATA0_FD5_POS)		/** Filter bits */
#define CAN_F19DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F19DATA0_FD6_MSK								(0x01UL << CAN_F19DATA0_FD6_POS)		/** Filter bits */
#define CAN_F19DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F19DATA0_FD7_MSK								(0x01UL << CAN_F19DATA0_FD7_POS)		/** Filter bits */
#define CAN_F19DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F19DATA0_FD8_MSK								(0x01UL << CAN_F19DATA0_FD8_POS)		/** Filter bits */
#define CAN_F19DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F19DATA0_FD9_MSK								(0x01UL << CAN_F19DATA0_FD9_POS)		/** Filter bits */
#define CAN_F19DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F19DATA0_FD10_MSK								(0x01UL << CAN_F19DATA0_FD10_POS)		/** Filter bits */
#define CAN_F19DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F19DATA0_FD11_MSK								(0x01UL << CAN_F19DATA0_FD11_POS)		/** Filter bits */
#define CAN_F19DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F19DATA0_FD12_MSK								(0x01UL << CAN_F19DATA0_FD12_POS)		/** Filter bits */
#define CAN_F19DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F19DATA0_FD13_MSK								(0x01UL << CAN_F19DATA0_FD13_POS)		/** Filter bits */
#define CAN_F19DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F19DATA0_FD14_MSK								(0x01UL << CAN_F19DATA0_FD14_POS)		/** Filter bits */
#define CAN_F19DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F19DATA0_FD15_MSK								(0x01UL << CAN_F19DATA0_FD15_POS)		/** Filter bits */
#define CAN_F19DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F19DATA0_FD16_MSK								(0x01UL << CAN_F19DATA0_FD16_POS)		/** Filter bits */
#define CAN_F19DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F19DATA0_FD17_MSK								(0x01UL << CAN_F19DATA0_FD17_POS)		/** Filter bits */
#define CAN_F19DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F19DATA0_FD18_MSK								(0x01UL << CAN_F19DATA0_FD18_POS)		/** Filter bits */
#define CAN_F19DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F19DATA0_FD19_MSK								(0x01UL << CAN_F19DATA0_FD19_POS)		/** Filter bits */
#define CAN_F19DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F19DATA0_FD20_MSK								(0x01UL << CAN_F19DATA0_FD20_POS)		/** Filter bits */
#define CAN_F19DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F19DATA0_FD21_MSK								(0x01UL << CAN_F19DATA0_FD21_POS)		/** Filter bits */
#define CAN_F19DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F19DATA0_FD22_MSK								(0x01UL << CAN_F19DATA0_FD22_POS)		/** Filter bits */
#define CAN_F19DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F19DATA0_FD23_MSK								(0x01UL << CAN_F19DATA0_FD23_POS)		/** Filter bits */
#define CAN_F19DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F19DATA0_FD24_MSK								(0x01UL << CAN_F19DATA0_FD24_POS)		/** Filter bits */
#define CAN_F19DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F19DATA0_FD25_MSK								(0x01UL << CAN_F19DATA0_FD25_POS)		/** Filter bits */
#define CAN_F19DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F19DATA0_FD26_MSK								(0x01UL << CAN_F19DATA0_FD26_POS)		/** Filter bits */
#define CAN_F19DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F19DATA0_FD27_MSK								(0x01UL << CAN_F19DATA0_FD27_POS)		/** Filter bits */
#define CAN_F19DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F19DATA0_FD28_MSK								(0x01UL << CAN_F19DATA0_FD28_POS)		/** Filter bits */
#define CAN_F19DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F19DATA0_FD29_MSK								(0x01UL << CAN_F19DATA0_FD29_POS)		/** Filter bits */
#define CAN_F19DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F19DATA0_FD30_MSK								(0x01UL << CAN_F19DATA0_FD30_POS)		/** Filter bits */
#define CAN_F19DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F19DATA0_FD31_MSK								(0x01UL << CAN_F19DATA0_FD31_POS)		/** Filter bits */
#define CAN_F19DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F19DATA1_FD0_MSK								(0x01UL << CAN_F19DATA1_FD0_POS)		/** Filter bits */
#define CAN_F19DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F19DATA1_FD1_MSK								(0x01UL << CAN_F19DATA1_FD1_POS)		/** Filter bits */
#define CAN_F19DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F19DATA1_FD2_MSK								(0x01UL << CAN_F19DATA1_FD2_POS)		/** Filter bits */
#define CAN_F19DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F19DATA1_FD3_MSK								(0x01UL << CAN_F19DATA1_FD3_POS)		/** Filter bits */
#define CAN_F19DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F19DATA1_FD4_MSK								(0x01UL << CAN_F19DATA1_FD4_POS)		/** Filter bits */
#define CAN_F19DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F19DATA1_FD5_MSK								(0x01UL << CAN_F19DATA1_FD5_POS)		/** Filter bits */
#define CAN_F19DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F19DATA1_FD6_MSK								(0x01UL << CAN_F19DATA1_FD6_POS)		/** Filter bits */
#define CAN_F19DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F19DATA1_FD7_MSK								(0x01UL << CAN_F19DATA1_FD7_POS)		/** Filter bits */
#define CAN_F19DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F19DATA1_FD8_MSK								(0x01UL << CAN_F19DATA1_FD8_POS)		/** Filter bits */
#define CAN_F19DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F19DATA1_FD9_MSK								(0x01UL << CAN_F19DATA1_FD9_POS)		/** Filter bits */
#define CAN_F19DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F19DATA1_FD10_MSK								(0x01UL << CAN_F19DATA1_FD10_POS)		/** Filter bits */
#define CAN_F19DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F19DATA1_FD11_MSK								(0x01UL << CAN_F19DATA1_FD11_POS)		/** Filter bits */
#define CAN_F19DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F19DATA1_FD12_MSK								(0x01UL << CAN_F19DATA1_FD12_POS)		/** Filter bits */
#define CAN_F19DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F19DATA1_FD13_MSK								(0x01UL << CAN_F19DATA1_FD13_POS)		/** Filter bits */
#define CAN_F19DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F19DATA1_FD14_MSK								(0x01UL << CAN_F19DATA1_FD14_POS)		/** Filter bits */
#define CAN_F19DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F19DATA1_FD15_MSK								(0x01UL << CAN_F19DATA1_FD15_POS)		/** Filter bits */
#define CAN_F19DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F19DATA1_FD16_MSK								(0x01UL << CAN_F19DATA1_FD16_POS)		/** Filter bits */
#define CAN_F19DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F19DATA1_FD17_MSK								(0x01UL << CAN_F19DATA1_FD17_POS)		/** Filter bits */
#define CAN_F19DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F19DATA1_FD18_MSK								(0x01UL << CAN_F19DATA1_FD18_POS)		/** Filter bits */
#define CAN_F19DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F19DATA1_FD19_MSK								(0x01UL << CAN_F19DATA1_FD19_POS)		/** Filter bits */
#define CAN_F19DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F19DATA1_FD20_MSK								(0x01UL << CAN_F19DATA1_FD20_POS)		/** Filter bits */
#define CAN_F19DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F19DATA1_FD21_MSK								(0x01UL << CAN_F19DATA1_FD21_POS)		/** Filter bits */
#define CAN_F19DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F19DATA1_FD22_MSK								(0x01UL << CAN_F19DATA1_FD22_POS)		/** Filter bits */
#define CAN_F19DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F19DATA1_FD23_MSK								(0x01UL << CAN_F19DATA1_FD23_POS)		/** Filter bits */
#define CAN_F19DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F19DATA1_FD24_MSK								(0x01UL << CAN_F19DATA1_FD24_POS)		/** Filter bits */
#define CAN_F19DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F19DATA1_FD25_MSK								(0x01UL << CAN_F19DATA1_FD25_POS)		/** Filter bits */
#define CAN_F19DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F19DATA1_FD26_MSK								(0x01UL << CAN_F19DATA1_FD26_POS)		/** Filter bits */
#define CAN_F19DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F19DATA1_FD27_MSK								(0x01UL << CAN_F19DATA1_FD27_POS)		/** Filter bits */
#define CAN_F19DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F19DATA1_FD28_MSK								(0x01UL << CAN_F19DATA1_FD28_POS)		/** Filter bits */
#define CAN_F19DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F19DATA1_FD29_MSK								(0x01UL << CAN_F19DATA1_FD29_POS)		/** Filter bits */
#define CAN_F19DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F19DATA1_FD30_MSK								(0x01UL << CAN_F19DATA1_FD30_POS)		/** Filter bits */
#define CAN_F19DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F19DATA1_FD31_MSK								(0x01UL << CAN_F19DATA1_FD31_POS)		/** Filter bits */
#define CAN_F20DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F20DATA0_FD0_MSK								(0x01UL << CAN_F20DATA0_FD0_POS)		/** Filter bits */
#define CAN_F20DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F20DATA0_FD1_MSK								(0x01UL << CAN_F20DATA0_FD1_POS)		/** Filter bits */
#define CAN_F20DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F20DATA0_FD2_MSK								(0x01UL << CAN_F20DATA0_FD2_POS)		/** Filter bits */
#define CAN_F20DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F20DATA0_FD3_MSK								(0x01UL << CAN_F20DATA0_FD3_POS)		/** Filter bits */
#define CAN_F20DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F20DATA0_FD4_MSK								(0x01UL << CAN_F20DATA0_FD4_POS)		/** Filter bits */
#define CAN_F20DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F20DATA0_FD5_MSK								(0x01UL << CAN_F20DATA0_FD5_POS)		/** Filter bits */
#define CAN_F20DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F20DATA0_FD6_MSK								(0x01UL << CAN_F20DATA0_FD6_POS)		/** Filter bits */
#define CAN_F20DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F20DATA0_FD7_MSK								(0x01UL << CAN_F20DATA0_FD7_POS)		/** Filter bits */
#define CAN_F20DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F20DATA0_FD8_MSK								(0x01UL << CAN_F20DATA0_FD8_POS)		/** Filter bits */
#define CAN_F20DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F20DATA0_FD9_MSK								(0x01UL << CAN_F20DATA0_FD9_POS)		/** Filter bits */
#define CAN_F20DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F20DATA0_FD10_MSK								(0x01UL << CAN_F20DATA0_FD10_POS)		/** Filter bits */
#define CAN_F20DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F20DATA0_FD11_MSK								(0x01UL << CAN_F20DATA0_FD11_POS)		/** Filter bits */
#define CAN_F20DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F20DATA0_FD12_MSK								(0x01UL << CAN_F20DATA0_FD12_POS)		/** Filter bits */
#define CAN_F20DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F20DATA0_FD13_MSK								(0x01UL << CAN_F20DATA0_FD13_POS)		/** Filter bits */
#define CAN_F20DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F20DATA0_FD14_MSK								(0x01UL << CAN_F20DATA0_FD14_POS)		/** Filter bits */
#define CAN_F20DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F20DATA0_FD15_MSK								(0x01UL << CAN_F20DATA0_FD15_POS)		/** Filter bits */
#define CAN_F20DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F20DATA0_FD16_MSK								(0x01UL << CAN_F20DATA0_FD16_POS)		/** Filter bits */
#define CAN_F20DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F20DATA0_FD17_MSK								(0x01UL << CAN_F20DATA0_FD17_POS)		/** Filter bits */
#define CAN_F20DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F20DATA0_FD18_MSK								(0x01UL << CAN_F20DATA0_FD18_POS)		/** Filter bits */
#define CAN_F20DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F20DATA0_FD19_MSK								(0x01UL << CAN_F20DATA0_FD19_POS)		/** Filter bits */
#define CAN_F20DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F20DATA0_FD20_MSK								(0x01UL << CAN_F20DATA0_FD20_POS)		/** Filter bits */
#define CAN_F20DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F20DATA0_FD21_MSK								(0x01UL << CAN_F20DATA0_FD21_POS)		/** Filter bits */
#define CAN_F20DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F20DATA0_FD22_MSK								(0x01UL << CAN_F20DATA0_FD22_POS)		/** Filter bits */
#define CAN_F20DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F20DATA0_FD23_MSK								(0x01UL << CAN_F20DATA0_FD23_POS)		/** Filter bits */
#define CAN_F20DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F20DATA0_FD24_MSK								(0x01UL << CAN_F20DATA0_FD24_POS)		/** Filter bits */
#define CAN_F20DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F20DATA0_FD25_MSK								(0x01UL << CAN_F20DATA0_FD25_POS)		/** Filter bits */
#define CAN_F20DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F20DATA0_FD26_MSK								(0x01UL << CAN_F20DATA0_FD26_POS)		/** Filter bits */
#define CAN_F20DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F20DATA0_FD27_MSK								(0x01UL << CAN_F20DATA0_FD27_POS)		/** Filter bits */
#define CAN_F20DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F20DATA0_FD28_MSK								(0x01UL << CAN_F20DATA0_FD28_POS)		/** Filter bits */
#define CAN_F20DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F20DATA0_FD29_MSK								(0x01UL << CAN_F20DATA0_FD29_POS)		/** Filter bits */
#define CAN_F20DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F20DATA0_FD30_MSK								(0x01UL << CAN_F20DATA0_FD30_POS)		/** Filter bits */
#define CAN_F20DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F20DATA0_FD31_MSK								(0x01UL << CAN_F20DATA0_FD31_POS)		/** Filter bits */
#define CAN_F20DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F20DATA1_FD0_MSK								(0x01UL << CAN_F20DATA1_FD0_POS)		/** Filter bits */
#define CAN_F20DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F20DATA1_FD1_MSK								(0x01UL << CAN_F20DATA1_FD1_POS)		/** Filter bits */
#define CAN_F20DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F20DATA1_FD2_MSK								(0x01UL << CAN_F20DATA1_FD2_POS)		/** Filter bits */
#define CAN_F20DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F20DATA1_FD3_MSK								(0x01UL << CAN_F20DATA1_FD3_POS)		/** Filter bits */
#define CAN_F20DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F20DATA1_FD4_MSK								(0x01UL << CAN_F20DATA1_FD4_POS)		/** Filter bits */
#define CAN_F20DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F20DATA1_FD5_MSK								(0x01UL << CAN_F20DATA1_FD5_POS)		/** Filter bits */
#define CAN_F20DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F20DATA1_FD6_MSK								(0x01UL << CAN_F20DATA1_FD6_POS)		/** Filter bits */
#define CAN_F20DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F20DATA1_FD7_MSK								(0x01UL << CAN_F20DATA1_FD7_POS)		/** Filter bits */
#define CAN_F20DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F20DATA1_FD8_MSK								(0x01UL << CAN_F20DATA1_FD8_POS)		/** Filter bits */
#define CAN_F20DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F20DATA1_FD9_MSK								(0x01UL << CAN_F20DATA1_FD9_POS)		/** Filter bits */
#define CAN_F20DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F20DATA1_FD10_MSK								(0x01UL << CAN_F20DATA1_FD10_POS)		/** Filter bits */
#define CAN_F20DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F20DATA1_FD11_MSK								(0x01UL << CAN_F20DATA1_FD11_POS)		/** Filter bits */
#define CAN_F20DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F20DATA1_FD12_MSK								(0x01UL << CAN_F20DATA1_FD12_POS)		/** Filter bits */
#define CAN_F20DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F20DATA1_FD13_MSK								(0x01UL << CAN_F20DATA1_FD13_POS)		/** Filter bits */
#define CAN_F20DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F20DATA1_FD14_MSK								(0x01UL << CAN_F20DATA1_FD14_POS)		/** Filter bits */
#define CAN_F20DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F20DATA1_FD15_MSK								(0x01UL << CAN_F20DATA1_FD15_POS)		/** Filter bits */
#define CAN_F20DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F20DATA1_FD16_MSK								(0x01UL << CAN_F20DATA1_FD16_POS)		/** Filter bits */
#define CAN_F20DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F20DATA1_FD17_MSK								(0x01UL << CAN_F20DATA1_FD17_POS)		/** Filter bits */
#define CAN_F20DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F20DATA1_FD18_MSK								(0x01UL << CAN_F20DATA1_FD18_POS)		/** Filter bits */
#define CAN_F20DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F20DATA1_FD19_MSK								(0x01UL << CAN_F20DATA1_FD19_POS)		/** Filter bits */
#define CAN_F20DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F20DATA1_FD20_MSK								(0x01UL << CAN_F20DATA1_FD20_POS)		/** Filter bits */
#define CAN_F20DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F20DATA1_FD21_MSK								(0x01UL << CAN_F20DATA1_FD21_POS)		/** Filter bits */
#define CAN_F20DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F20DATA1_FD22_MSK								(0x01UL << CAN_F20DATA1_FD22_POS)		/** Filter bits */
#define CAN_F20DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F20DATA1_FD23_MSK								(0x01UL << CAN_F20DATA1_FD23_POS)		/** Filter bits */
#define CAN_F20DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F20DATA1_FD24_MSK								(0x01UL << CAN_F20DATA1_FD24_POS)		/** Filter bits */
#define CAN_F20DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F20DATA1_FD25_MSK								(0x01UL << CAN_F20DATA1_FD25_POS)		/** Filter bits */
#define CAN_F20DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F20DATA1_FD26_MSK								(0x01UL << CAN_F20DATA1_FD26_POS)		/** Filter bits */
#define CAN_F20DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F20DATA1_FD27_MSK								(0x01UL << CAN_F20DATA1_FD27_POS)		/** Filter bits */
#define CAN_F20DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F20DATA1_FD28_MSK								(0x01UL << CAN_F20DATA1_FD28_POS)		/** Filter bits */
#define CAN_F20DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F20DATA1_FD29_MSK								(0x01UL << CAN_F20DATA1_FD29_POS)		/** Filter bits */
#define CAN_F20DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F20DATA1_FD30_MSK								(0x01UL << CAN_F20DATA1_FD30_POS)		/** Filter bits */
#define CAN_F20DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F20DATA1_FD31_MSK								(0x01UL << CAN_F20DATA1_FD31_POS)		/** Filter bits */
#define CAN_F21DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F21DATA0_FD0_MSK								(0x01UL << CAN_F21DATA0_FD0_POS)		/** Filter bits */
#define CAN_F21DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F21DATA0_FD1_MSK								(0x01UL << CAN_F21DATA0_FD1_POS)		/** Filter bits */
#define CAN_F21DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F21DATA0_FD2_MSK								(0x01UL << CAN_F21DATA0_FD2_POS)		/** Filter bits */
#define CAN_F21DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F21DATA0_FD3_MSK								(0x01UL << CAN_F21DATA0_FD3_POS)		/** Filter bits */
#define CAN_F21DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F21DATA0_FD4_MSK								(0x01UL << CAN_F21DATA0_FD4_POS)		/** Filter bits */
#define CAN_F21DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F21DATA0_FD5_MSK								(0x01UL << CAN_F21DATA0_FD5_POS)		/** Filter bits */
#define CAN_F21DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F21DATA0_FD6_MSK								(0x01UL << CAN_F21DATA0_FD6_POS)		/** Filter bits */
#define CAN_F21DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F21DATA0_FD7_MSK								(0x01UL << CAN_F21DATA0_FD7_POS)		/** Filter bits */
#define CAN_F21DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F21DATA0_FD8_MSK								(0x01UL << CAN_F21DATA0_FD8_POS)		/** Filter bits */
#define CAN_F21DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F21DATA0_FD9_MSK								(0x01UL << CAN_F21DATA0_FD9_POS)		/** Filter bits */
#define CAN_F21DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F21DATA0_FD10_MSK								(0x01UL << CAN_F21DATA0_FD10_POS)		/** Filter bits */
#define CAN_F21DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F21DATA0_FD11_MSK								(0x01UL << CAN_F21DATA0_FD11_POS)		/** Filter bits */
#define CAN_F21DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F21DATA0_FD12_MSK								(0x01UL << CAN_F21DATA0_FD12_POS)		/** Filter bits */
#define CAN_F21DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F21DATA0_FD13_MSK								(0x01UL << CAN_F21DATA0_FD13_POS)		/** Filter bits */
#define CAN_F21DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F21DATA0_FD14_MSK								(0x01UL << CAN_F21DATA0_FD14_POS)		/** Filter bits */
#define CAN_F21DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F21DATA0_FD15_MSK								(0x01UL << CAN_F21DATA0_FD15_POS)		/** Filter bits */
#define CAN_F21DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F21DATA0_FD16_MSK								(0x01UL << CAN_F21DATA0_FD16_POS)		/** Filter bits */
#define CAN_F21DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F21DATA0_FD17_MSK								(0x01UL << CAN_F21DATA0_FD17_POS)		/** Filter bits */
#define CAN_F21DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F21DATA0_FD18_MSK								(0x01UL << CAN_F21DATA0_FD18_POS)		/** Filter bits */
#define CAN_F21DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F21DATA0_FD19_MSK								(0x01UL << CAN_F21DATA0_FD19_POS)		/** Filter bits */
#define CAN_F21DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F21DATA0_FD20_MSK								(0x01UL << CAN_F21DATA0_FD20_POS)		/** Filter bits */
#define CAN_F21DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F21DATA0_FD21_MSK								(0x01UL << CAN_F21DATA0_FD21_POS)		/** Filter bits */
#define CAN_F21DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F21DATA0_FD22_MSK								(0x01UL << CAN_F21DATA0_FD22_POS)		/** Filter bits */
#define CAN_F21DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F21DATA0_FD23_MSK								(0x01UL << CAN_F21DATA0_FD23_POS)		/** Filter bits */
#define CAN_F21DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F21DATA0_FD24_MSK								(0x01UL << CAN_F21DATA0_FD24_POS)		/** Filter bits */
#define CAN_F21DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F21DATA0_FD25_MSK								(0x01UL << CAN_F21DATA0_FD25_POS)		/** Filter bits */
#define CAN_F21DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F21DATA0_FD26_MSK								(0x01UL << CAN_F21DATA0_FD26_POS)		/** Filter bits */
#define CAN_F21DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F21DATA0_FD27_MSK								(0x01UL << CAN_F21DATA0_FD27_POS)		/** Filter bits */
#define CAN_F21DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F21DATA0_FD28_MSK								(0x01UL << CAN_F21DATA0_FD28_POS)		/** Filter bits */
#define CAN_F21DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F21DATA0_FD29_MSK								(0x01UL << CAN_F21DATA0_FD29_POS)		/** Filter bits */
#define CAN_F21DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F21DATA0_FD30_MSK								(0x01UL << CAN_F21DATA0_FD30_POS)		/** Filter bits */
#define CAN_F21DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F21DATA0_FD31_MSK								(0x01UL << CAN_F21DATA0_FD31_POS)		/** Filter bits */
#define CAN_F21DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F21DATA1_FD0_MSK								(0x01UL << CAN_F21DATA1_FD0_POS)		/** Filter bits */
#define CAN_F21DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F21DATA1_FD1_MSK								(0x01UL << CAN_F21DATA1_FD1_POS)		/** Filter bits */
#define CAN_F21DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F21DATA1_FD2_MSK								(0x01UL << CAN_F21DATA1_FD2_POS)		/** Filter bits */
#define CAN_F21DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F21DATA1_FD3_MSK								(0x01UL << CAN_F21DATA1_FD3_POS)		/** Filter bits */
#define CAN_F21DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F21DATA1_FD4_MSK								(0x01UL << CAN_F21DATA1_FD4_POS)		/** Filter bits */
#define CAN_F21DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F21DATA1_FD5_MSK								(0x01UL << CAN_F21DATA1_FD5_POS)		/** Filter bits */
#define CAN_F21DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F21DATA1_FD6_MSK								(0x01UL << CAN_F21DATA1_FD6_POS)		/** Filter bits */
#define CAN_F21DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F21DATA1_FD7_MSK								(0x01UL << CAN_F21DATA1_FD7_POS)		/** Filter bits */
#define CAN_F21DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F21DATA1_FD8_MSK								(0x01UL << CAN_F21DATA1_FD8_POS)		/** Filter bits */
#define CAN_F21DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F21DATA1_FD9_MSK								(0x01UL << CAN_F21DATA1_FD9_POS)		/** Filter bits */
#define CAN_F21DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F21DATA1_FD10_MSK								(0x01UL << CAN_F21DATA1_FD10_POS)		/** Filter bits */
#define CAN_F21DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F21DATA1_FD11_MSK								(0x01UL << CAN_F21DATA1_FD11_POS)		/** Filter bits */
#define CAN_F21DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F21DATA1_FD12_MSK								(0x01UL << CAN_F21DATA1_FD12_POS)		/** Filter bits */
#define CAN_F21DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F21DATA1_FD13_MSK								(0x01UL << CAN_F21DATA1_FD13_POS)		/** Filter bits */
#define CAN_F21DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F21DATA1_FD14_MSK								(0x01UL << CAN_F21DATA1_FD14_POS)		/** Filter bits */
#define CAN_F21DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F21DATA1_FD15_MSK								(0x01UL << CAN_F21DATA1_FD15_POS)		/** Filter bits */
#define CAN_F21DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F21DATA1_FD16_MSK								(0x01UL << CAN_F21DATA1_FD16_POS)		/** Filter bits */
#define CAN_F21DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F21DATA1_FD17_MSK								(0x01UL << CAN_F21DATA1_FD17_POS)		/** Filter bits */
#define CAN_F21DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F21DATA1_FD18_MSK								(0x01UL << CAN_F21DATA1_FD18_POS)		/** Filter bits */
#define CAN_F21DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F21DATA1_FD19_MSK								(0x01UL << CAN_F21DATA1_FD19_POS)		/** Filter bits */
#define CAN_F21DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F21DATA1_FD20_MSK								(0x01UL << CAN_F21DATA1_FD20_POS)		/** Filter bits */
#define CAN_F21DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F21DATA1_FD21_MSK								(0x01UL << CAN_F21DATA1_FD21_POS)		/** Filter bits */
#define CAN_F21DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F21DATA1_FD22_MSK								(0x01UL << CAN_F21DATA1_FD22_POS)		/** Filter bits */
#define CAN_F21DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F21DATA1_FD23_MSK								(0x01UL << CAN_F21DATA1_FD23_POS)		/** Filter bits */
#define CAN_F21DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F21DATA1_FD24_MSK								(0x01UL << CAN_F21DATA1_FD24_POS)		/** Filter bits */
#define CAN_F21DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F21DATA1_FD25_MSK								(0x01UL << CAN_F21DATA1_FD25_POS)		/** Filter bits */
#define CAN_F21DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F21DATA1_FD26_MSK								(0x01UL << CAN_F21DATA1_FD26_POS)		/** Filter bits */
#define CAN_F21DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F21DATA1_FD27_MSK								(0x01UL << CAN_F21DATA1_FD27_POS)		/** Filter bits */
#define CAN_F21DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F21DATA1_FD28_MSK								(0x01UL << CAN_F21DATA1_FD28_POS)		/** Filter bits */
#define CAN_F21DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F21DATA1_FD29_MSK								(0x01UL << CAN_F21DATA1_FD29_POS)		/** Filter bits */
#define CAN_F21DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F21DATA1_FD30_MSK								(0x01UL << CAN_F21DATA1_FD30_POS)		/** Filter bits */
#define CAN_F21DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F21DATA1_FD31_MSK								(0x01UL << CAN_F21DATA1_FD31_POS)		/** Filter bits */
#define CAN_F22DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F22DATA0_FD0_MSK								(0x01UL << CAN_F22DATA0_FD0_POS)		/** Filter bits */
#define CAN_F22DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F22DATA0_FD1_MSK								(0x01UL << CAN_F22DATA0_FD1_POS)		/** Filter bits */
#define CAN_F22DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F22DATA0_FD2_MSK								(0x01UL << CAN_F22DATA0_FD2_POS)		/** Filter bits */
#define CAN_F22DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F22DATA0_FD3_MSK								(0x01UL << CAN_F22DATA0_FD3_POS)		/** Filter bits */
#define CAN_F22DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F22DATA0_FD4_MSK								(0x01UL << CAN_F22DATA0_FD4_POS)		/** Filter bits */
#define CAN_F22DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F22DATA0_FD5_MSK								(0x01UL << CAN_F22DATA0_FD5_POS)		/** Filter bits */
#define CAN_F22DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F22DATA0_FD6_MSK								(0x01UL << CAN_F22DATA0_FD6_POS)		/** Filter bits */
#define CAN_F22DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F22DATA0_FD7_MSK								(0x01UL << CAN_F22DATA0_FD7_POS)		/** Filter bits */
#define CAN_F22DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F22DATA0_FD8_MSK								(0x01UL << CAN_F22DATA0_FD8_POS)		/** Filter bits */
#define CAN_F22DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F22DATA0_FD9_MSK								(0x01UL << CAN_F22DATA0_FD9_POS)		/** Filter bits */
#define CAN_F22DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F22DATA0_FD10_MSK								(0x01UL << CAN_F22DATA0_FD10_POS)		/** Filter bits */
#define CAN_F22DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F22DATA0_FD11_MSK								(0x01UL << CAN_F22DATA0_FD11_POS)		/** Filter bits */
#define CAN_F22DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F22DATA0_FD12_MSK								(0x01UL << CAN_F22DATA0_FD12_POS)		/** Filter bits */
#define CAN_F22DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F22DATA0_FD13_MSK								(0x01UL << CAN_F22DATA0_FD13_POS)		/** Filter bits */
#define CAN_F22DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F22DATA0_FD14_MSK								(0x01UL << CAN_F22DATA0_FD14_POS)		/** Filter bits */
#define CAN_F22DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F22DATA0_FD15_MSK								(0x01UL << CAN_F22DATA0_FD15_POS)		/** Filter bits */
#define CAN_F22DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F22DATA0_FD16_MSK								(0x01UL << CAN_F22DATA0_FD16_POS)		/** Filter bits */
#define CAN_F22DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F22DATA0_FD17_MSK								(0x01UL << CAN_F22DATA0_FD17_POS)		/** Filter bits */
#define CAN_F22DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F22DATA0_FD18_MSK								(0x01UL << CAN_F22DATA0_FD18_POS)		/** Filter bits */
#define CAN_F22DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F22DATA0_FD19_MSK								(0x01UL << CAN_F22DATA0_FD19_POS)		/** Filter bits */
#define CAN_F22DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F22DATA0_FD20_MSK								(0x01UL << CAN_F22DATA0_FD20_POS)		/** Filter bits */
#define CAN_F22DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F22DATA0_FD21_MSK								(0x01UL << CAN_F22DATA0_FD21_POS)		/** Filter bits */
#define CAN_F22DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F22DATA0_FD22_MSK								(0x01UL << CAN_F22DATA0_FD22_POS)		/** Filter bits */
#define CAN_F22DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F22DATA0_FD23_MSK								(0x01UL << CAN_F22DATA0_FD23_POS)		/** Filter bits */
#define CAN_F22DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F22DATA0_FD24_MSK								(0x01UL << CAN_F22DATA0_FD24_POS)		/** Filter bits */
#define CAN_F22DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F22DATA0_FD25_MSK								(0x01UL << CAN_F22DATA0_FD25_POS)		/** Filter bits */
#define CAN_F22DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F22DATA0_FD26_MSK								(0x01UL << CAN_F22DATA0_FD26_POS)		/** Filter bits */
#define CAN_F22DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F22DATA0_FD27_MSK								(0x01UL << CAN_F22DATA0_FD27_POS)		/** Filter bits */
#define CAN_F22DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F22DATA0_FD28_MSK								(0x01UL << CAN_F22DATA0_FD28_POS)		/** Filter bits */
#define CAN_F22DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F22DATA0_FD29_MSK								(0x01UL << CAN_F22DATA0_FD29_POS)		/** Filter bits */
#define CAN_F22DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F22DATA0_FD30_MSK								(0x01UL << CAN_F22DATA0_FD30_POS)		/** Filter bits */
#define CAN_F22DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F22DATA0_FD31_MSK								(0x01UL << CAN_F22DATA0_FD31_POS)		/** Filter bits */
#define CAN_F22DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F22DATA1_FD0_MSK								(0x01UL << CAN_F22DATA1_FD0_POS)		/** Filter bits */
#define CAN_F22DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F22DATA1_FD1_MSK								(0x01UL << CAN_F22DATA1_FD1_POS)		/** Filter bits */
#define CAN_F22DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F22DATA1_FD2_MSK								(0x01UL << CAN_F22DATA1_FD2_POS)		/** Filter bits */
#define CAN_F22DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F22DATA1_FD3_MSK								(0x01UL << CAN_F22DATA1_FD3_POS)		/** Filter bits */
#define CAN_F22DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F22DATA1_FD4_MSK								(0x01UL << CAN_F22DATA1_FD4_POS)		/** Filter bits */
#define CAN_F22DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F22DATA1_FD5_MSK								(0x01UL << CAN_F22DATA1_FD5_POS)		/** Filter bits */
#define CAN_F22DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F22DATA1_FD6_MSK								(0x01UL << CAN_F22DATA1_FD6_POS)		/** Filter bits */
#define CAN_F22DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F22DATA1_FD7_MSK								(0x01UL << CAN_F22DATA1_FD7_POS)		/** Filter bits */
#define CAN_F22DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F22DATA1_FD8_MSK								(0x01UL << CAN_F22DATA1_FD8_POS)		/** Filter bits */
#define CAN_F22DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F22DATA1_FD9_MSK								(0x01UL << CAN_F22DATA1_FD9_POS)		/** Filter bits */
#define CAN_F22DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F22DATA1_FD10_MSK								(0x01UL << CAN_F22DATA1_FD10_POS)		/** Filter bits */
#define CAN_F22DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F22DATA1_FD11_MSK								(0x01UL << CAN_F22DATA1_FD11_POS)		/** Filter bits */
#define CAN_F22DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F22DATA1_FD12_MSK								(0x01UL << CAN_F22DATA1_FD12_POS)		/** Filter bits */
#define CAN_F22DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F22DATA1_FD13_MSK								(0x01UL << CAN_F22DATA1_FD13_POS)		/** Filter bits */
#define CAN_F22DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F22DATA1_FD14_MSK								(0x01UL << CAN_F22DATA1_FD14_POS)		/** Filter bits */
#define CAN_F22DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F22DATA1_FD15_MSK								(0x01UL << CAN_F22DATA1_FD15_POS)		/** Filter bits */
#define CAN_F22DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F22DATA1_FD16_MSK								(0x01UL << CAN_F22DATA1_FD16_POS)		/** Filter bits */
#define CAN_F22DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F22DATA1_FD17_MSK								(0x01UL << CAN_F22DATA1_FD17_POS)		/** Filter bits */
#define CAN_F22DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F22DATA1_FD18_MSK								(0x01UL << CAN_F22DATA1_FD18_POS)		/** Filter bits */
#define CAN_F22DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F22DATA1_FD19_MSK								(0x01UL << CAN_F22DATA1_FD19_POS)		/** Filter bits */
#define CAN_F22DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F22DATA1_FD20_MSK								(0x01UL << CAN_F22DATA1_FD20_POS)		/** Filter bits */
#define CAN_F22DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F22DATA1_FD21_MSK								(0x01UL << CAN_F22DATA1_FD21_POS)		/** Filter bits */
#define CAN_F22DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F22DATA1_FD22_MSK								(0x01UL << CAN_F22DATA1_FD22_POS)		/** Filter bits */
#define CAN_F22DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F22DATA1_FD23_MSK								(0x01UL << CAN_F22DATA1_FD23_POS)		/** Filter bits */
#define CAN_F22DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F22DATA1_FD24_MSK								(0x01UL << CAN_F22DATA1_FD24_POS)		/** Filter bits */
#define CAN_F22DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F22DATA1_FD25_MSK								(0x01UL << CAN_F22DATA1_FD25_POS)		/** Filter bits */
#define CAN_F22DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F22DATA1_FD26_MSK								(0x01UL << CAN_F22DATA1_FD26_POS)		/** Filter bits */
#define CAN_F22DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F22DATA1_FD27_MSK								(0x01UL << CAN_F22DATA1_FD27_POS)		/** Filter bits */
#define CAN_F22DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F22DATA1_FD28_MSK								(0x01UL << CAN_F22DATA1_FD28_POS)		/** Filter bits */
#define CAN_F22DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F22DATA1_FD29_MSK								(0x01UL << CAN_F22DATA1_FD29_POS)		/** Filter bits */
#define CAN_F22DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F22DATA1_FD30_MSK								(0x01UL << CAN_F22DATA1_FD30_POS)		/** Filter bits */
#define CAN_F22DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F22DATA1_FD31_MSK								(0x01UL << CAN_F22DATA1_FD31_POS)		/** Filter bits */
#define CAN_F23DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F23DATA0_FD0_MSK								(0x01UL << CAN_F23DATA0_FD0_POS)		/** Filter bits */
#define CAN_F23DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F23DATA0_FD1_MSK								(0x01UL << CAN_F23DATA0_FD1_POS)		/** Filter bits */
#define CAN_F23DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F23DATA0_FD2_MSK								(0x01UL << CAN_F23DATA0_FD2_POS)		/** Filter bits */
#define CAN_F23DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F23DATA0_FD3_MSK								(0x01UL << CAN_F23DATA0_FD3_POS)		/** Filter bits */
#define CAN_F23DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F23DATA0_FD4_MSK								(0x01UL << CAN_F23DATA0_FD4_POS)		/** Filter bits */
#define CAN_F23DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F23DATA0_FD5_MSK								(0x01UL << CAN_F23DATA0_FD5_POS)		/** Filter bits */
#define CAN_F23DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F23DATA0_FD6_MSK								(0x01UL << CAN_F23DATA0_FD6_POS)		/** Filter bits */
#define CAN_F23DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F23DATA0_FD7_MSK								(0x01UL << CAN_F23DATA0_FD7_POS)		/** Filter bits */
#define CAN_F23DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F23DATA0_FD8_MSK								(0x01UL << CAN_F23DATA0_FD8_POS)		/** Filter bits */
#define CAN_F23DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F23DATA0_FD9_MSK								(0x01UL << CAN_F23DATA0_FD9_POS)		/** Filter bits */
#define CAN_F23DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F23DATA0_FD10_MSK								(0x01UL << CAN_F23DATA0_FD10_POS)		/** Filter bits */
#define CAN_F23DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F23DATA0_FD11_MSK								(0x01UL << CAN_F23DATA0_FD11_POS)		/** Filter bits */
#define CAN_F23DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F23DATA0_FD12_MSK								(0x01UL << CAN_F23DATA0_FD12_POS)		/** Filter bits */
#define CAN_F23DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F23DATA0_FD13_MSK								(0x01UL << CAN_F23DATA0_FD13_POS)		/** Filter bits */
#define CAN_F23DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F23DATA0_FD14_MSK								(0x01UL << CAN_F23DATA0_FD14_POS)		/** Filter bits */
#define CAN_F23DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F23DATA0_FD15_MSK								(0x01UL << CAN_F23DATA0_FD15_POS)		/** Filter bits */
#define CAN_F23DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F23DATA0_FD16_MSK								(0x01UL << CAN_F23DATA0_FD16_POS)		/** Filter bits */
#define CAN_F23DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F23DATA0_FD17_MSK								(0x01UL << CAN_F23DATA0_FD17_POS)		/** Filter bits */
#define CAN_F23DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F23DATA0_FD18_MSK								(0x01UL << CAN_F23DATA0_FD18_POS)		/** Filter bits */
#define CAN_F23DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F23DATA0_FD19_MSK								(0x01UL << CAN_F23DATA0_FD19_POS)		/** Filter bits */
#define CAN_F23DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F23DATA0_FD20_MSK								(0x01UL << CAN_F23DATA0_FD20_POS)		/** Filter bits */
#define CAN_F23DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F23DATA0_FD21_MSK								(0x01UL << CAN_F23DATA0_FD21_POS)		/** Filter bits */
#define CAN_F23DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F23DATA0_FD22_MSK								(0x01UL << CAN_F23DATA0_FD22_POS)		/** Filter bits */
#define CAN_F23DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F23DATA0_FD23_MSK								(0x01UL << CAN_F23DATA0_FD23_POS)		/** Filter bits */
#define CAN_F23DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F23DATA0_FD24_MSK								(0x01UL << CAN_F23DATA0_FD24_POS)		/** Filter bits */
#define CAN_F23DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F23DATA0_FD25_MSK								(0x01UL << CAN_F23DATA0_FD25_POS)		/** Filter bits */
#define CAN_F23DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F23DATA0_FD26_MSK								(0x01UL << CAN_F23DATA0_FD26_POS)		/** Filter bits */
#define CAN_F23DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F23DATA0_FD27_MSK								(0x01UL << CAN_F23DATA0_FD27_POS)		/** Filter bits */
#define CAN_F23DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F23DATA0_FD28_MSK								(0x01UL << CAN_F23DATA0_FD28_POS)		/** Filter bits */
#define CAN_F23DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F23DATA0_FD29_MSK								(0x01UL << CAN_F23DATA0_FD29_POS)		/** Filter bits */
#define CAN_F23DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F23DATA0_FD30_MSK								(0x01UL << CAN_F23DATA0_FD30_POS)		/** Filter bits */
#define CAN_F23DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F23DATA0_FD31_MSK								(0x01UL << CAN_F23DATA0_FD31_POS)		/** Filter bits */
#define CAN_F23DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F23DATA1_FD0_MSK								(0x01UL << CAN_F23DATA1_FD0_POS)		/** Filter bits */
#define CAN_F23DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F23DATA1_FD1_MSK								(0x01UL << CAN_F23DATA1_FD1_POS)		/** Filter bits */
#define CAN_F23DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F23DATA1_FD2_MSK								(0x01UL << CAN_F23DATA1_FD2_POS)		/** Filter bits */
#define CAN_F23DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F23DATA1_FD3_MSK								(0x01UL << CAN_F23DATA1_FD3_POS)		/** Filter bits */
#define CAN_F23DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F23DATA1_FD4_MSK								(0x01UL << CAN_F23DATA1_FD4_POS)		/** Filter bits */
#define CAN_F23DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F23DATA1_FD5_MSK								(0x01UL << CAN_F23DATA1_FD5_POS)		/** Filter bits */
#define CAN_F23DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F23DATA1_FD6_MSK								(0x01UL << CAN_F23DATA1_FD6_POS)		/** Filter bits */
#define CAN_F23DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F23DATA1_FD7_MSK								(0x01UL << CAN_F23DATA1_FD7_POS)		/** Filter bits */
#define CAN_F23DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F23DATA1_FD8_MSK								(0x01UL << CAN_F23DATA1_FD8_POS)		/** Filter bits */
#define CAN_F23DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F23DATA1_FD9_MSK								(0x01UL << CAN_F23DATA1_FD9_POS)		/** Filter bits */
#define CAN_F23DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F23DATA1_FD10_MSK								(0x01UL << CAN_F23DATA1_FD10_POS)		/** Filter bits */
#define CAN_F23DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F23DATA1_FD11_MSK								(0x01UL << CAN_F23DATA1_FD11_POS)		/** Filter bits */
#define CAN_F23DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F23DATA1_FD12_MSK								(0x01UL << CAN_F23DATA1_FD12_POS)		/** Filter bits */
#define CAN_F23DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F23DATA1_FD13_MSK								(0x01UL << CAN_F23DATA1_FD13_POS)		/** Filter bits */
#define CAN_F23DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F23DATA1_FD14_MSK								(0x01UL << CAN_F23DATA1_FD14_POS)		/** Filter bits */
#define CAN_F23DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F23DATA1_FD15_MSK								(0x01UL << CAN_F23DATA1_FD15_POS)		/** Filter bits */
#define CAN_F23DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F23DATA1_FD16_MSK								(0x01UL << CAN_F23DATA1_FD16_POS)		/** Filter bits */
#define CAN_F23DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F23DATA1_FD17_MSK								(0x01UL << CAN_F23DATA1_FD17_POS)		/** Filter bits */
#define CAN_F23DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F23DATA1_FD18_MSK								(0x01UL << CAN_F23DATA1_FD18_POS)		/** Filter bits */
#define CAN_F23DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F23DATA1_FD19_MSK								(0x01UL << CAN_F23DATA1_FD19_POS)		/** Filter bits */
#define CAN_F23DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F23DATA1_FD20_MSK								(0x01UL << CAN_F23DATA1_FD20_POS)		/** Filter bits */
#define CAN_F23DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F23DATA1_FD21_MSK								(0x01UL << CAN_F23DATA1_FD21_POS)		/** Filter bits */
#define CAN_F23DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F23DATA1_FD22_MSK								(0x01UL << CAN_F23DATA1_FD22_POS)		/** Filter bits */
#define CAN_F23DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F23DATA1_FD23_MSK								(0x01UL << CAN_F23DATA1_FD23_POS)		/** Filter bits */
#define CAN_F23DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F23DATA1_FD24_MSK								(0x01UL << CAN_F23DATA1_FD24_POS)		/** Filter bits */
#define CAN_F23DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F23DATA1_FD25_MSK								(0x01UL << CAN_F23DATA1_FD25_POS)		/** Filter bits */
#define CAN_F23DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F23DATA1_FD26_MSK								(0x01UL << CAN_F23DATA1_FD26_POS)		/** Filter bits */
#define CAN_F23DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F23DATA1_FD27_MSK								(0x01UL << CAN_F23DATA1_FD27_POS)		/** Filter bits */
#define CAN_F23DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F23DATA1_FD28_MSK								(0x01UL << CAN_F23DATA1_FD28_POS)		/** Filter bits */
#define CAN_F23DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F23DATA1_FD29_MSK								(0x01UL << CAN_F23DATA1_FD29_POS)		/** Filter bits */
#define CAN_F23DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F23DATA1_FD30_MSK								(0x01UL << CAN_F23DATA1_FD30_POS)		/** Filter bits */
#define CAN_F23DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F23DATA1_FD31_MSK								(0x01UL << CAN_F23DATA1_FD31_POS)		/** Filter bits */
#define CAN_F24DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F24DATA0_FD0_MSK								(0x01UL << CAN_F24DATA0_FD0_POS)		/** Filter bits */
#define CAN_F24DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F24DATA0_FD1_MSK								(0x01UL << CAN_F24DATA0_FD1_POS)		/** Filter bits */
#define CAN_F24DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F24DATA0_FD2_MSK								(0x01UL << CAN_F24DATA0_FD2_POS)		/** Filter bits */
#define CAN_F24DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F24DATA0_FD3_MSK								(0x01UL << CAN_F24DATA0_FD3_POS)		/** Filter bits */
#define CAN_F24DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F24DATA0_FD4_MSK								(0x01UL << CAN_F24DATA0_FD4_POS)		/** Filter bits */
#define CAN_F24DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F24DATA0_FD5_MSK								(0x01UL << CAN_F24DATA0_FD5_POS)		/** Filter bits */
#define CAN_F24DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F24DATA0_FD6_MSK								(0x01UL << CAN_F24DATA0_FD6_POS)		/** Filter bits */
#define CAN_F24DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F24DATA0_FD7_MSK								(0x01UL << CAN_F24DATA0_FD7_POS)		/** Filter bits */
#define CAN_F24DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F24DATA0_FD8_MSK								(0x01UL << CAN_F24DATA0_FD8_POS)		/** Filter bits */
#define CAN_F24DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F24DATA0_FD9_MSK								(0x01UL << CAN_F24DATA0_FD9_POS)		/** Filter bits */
#define CAN_F24DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F24DATA0_FD10_MSK								(0x01UL << CAN_F24DATA0_FD10_POS)		/** Filter bits */
#define CAN_F24DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F24DATA0_FD11_MSK								(0x01UL << CAN_F24DATA0_FD11_POS)		/** Filter bits */
#define CAN_F24DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F24DATA0_FD12_MSK								(0x01UL << CAN_F24DATA0_FD12_POS)		/** Filter bits */
#define CAN_F24DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F24DATA0_FD13_MSK								(0x01UL << CAN_F24DATA0_FD13_POS)		/** Filter bits */
#define CAN_F24DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F24DATA0_FD14_MSK								(0x01UL << CAN_F24DATA0_FD14_POS)		/** Filter bits */
#define CAN_F24DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F24DATA0_FD15_MSK								(0x01UL << CAN_F24DATA0_FD15_POS)		/** Filter bits */
#define CAN_F24DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F24DATA0_FD16_MSK								(0x01UL << CAN_F24DATA0_FD16_POS)		/** Filter bits */
#define CAN_F24DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F24DATA0_FD17_MSK								(0x01UL << CAN_F24DATA0_FD17_POS)		/** Filter bits */
#define CAN_F24DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F24DATA0_FD18_MSK								(0x01UL << CAN_F24DATA0_FD18_POS)		/** Filter bits */
#define CAN_F24DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F24DATA0_FD19_MSK								(0x01UL << CAN_F24DATA0_FD19_POS)		/** Filter bits */
#define CAN_F24DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F24DATA0_FD20_MSK								(0x01UL << CAN_F24DATA0_FD20_POS)		/** Filter bits */
#define CAN_F24DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F24DATA0_FD21_MSK								(0x01UL << CAN_F24DATA0_FD21_POS)		/** Filter bits */
#define CAN_F24DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F24DATA0_FD22_MSK								(0x01UL << CAN_F24DATA0_FD22_POS)		/** Filter bits */
#define CAN_F24DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F24DATA0_FD23_MSK								(0x01UL << CAN_F24DATA0_FD23_POS)		/** Filter bits */
#define CAN_F24DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F24DATA0_FD24_MSK								(0x01UL << CAN_F24DATA0_FD24_POS)		/** Filter bits */
#define CAN_F24DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F24DATA0_FD25_MSK								(0x01UL << CAN_F24DATA0_FD25_POS)		/** Filter bits */
#define CAN_F24DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F24DATA0_FD26_MSK								(0x01UL << CAN_F24DATA0_FD26_POS)		/** Filter bits */
#define CAN_F24DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F24DATA0_FD27_MSK								(0x01UL << CAN_F24DATA0_FD27_POS)		/** Filter bits */
#define CAN_F24DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F24DATA0_FD28_MSK								(0x01UL << CAN_F24DATA0_FD28_POS)		/** Filter bits */
#define CAN_F24DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F24DATA0_FD29_MSK								(0x01UL << CAN_F24DATA0_FD29_POS)		/** Filter bits */
#define CAN_F24DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F24DATA0_FD30_MSK								(0x01UL << CAN_F24DATA0_FD30_POS)		/** Filter bits */
#define CAN_F24DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F24DATA0_FD31_MSK								(0x01UL << CAN_F24DATA0_FD31_POS)		/** Filter bits */
#define CAN_F24DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F24DATA1_FD0_MSK								(0x01UL << CAN_F24DATA1_FD0_POS)		/** Filter bits */
#define CAN_F24DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F24DATA1_FD1_MSK								(0x01UL << CAN_F24DATA1_FD1_POS)		/** Filter bits */
#define CAN_F24DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F24DATA1_FD2_MSK								(0x01UL << CAN_F24DATA1_FD2_POS)		/** Filter bits */
#define CAN_F24DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F24DATA1_FD3_MSK								(0x01UL << CAN_F24DATA1_FD3_POS)		/** Filter bits */
#define CAN_F24DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F24DATA1_FD4_MSK								(0x01UL << CAN_F24DATA1_FD4_POS)		/** Filter bits */
#define CAN_F24DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F24DATA1_FD5_MSK								(0x01UL << CAN_F24DATA1_FD5_POS)		/** Filter bits */
#define CAN_F24DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F24DATA1_FD6_MSK								(0x01UL << CAN_F24DATA1_FD6_POS)		/** Filter bits */
#define CAN_F24DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F24DATA1_FD7_MSK								(0x01UL << CAN_F24DATA1_FD7_POS)		/** Filter bits */
#define CAN_F24DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F24DATA1_FD8_MSK								(0x01UL << CAN_F24DATA1_FD8_POS)		/** Filter bits */
#define CAN_F24DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F24DATA1_FD9_MSK								(0x01UL << CAN_F24DATA1_FD9_POS)		/** Filter bits */
#define CAN_F24DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F24DATA1_FD10_MSK								(0x01UL << CAN_F24DATA1_FD10_POS)		/** Filter bits */
#define CAN_F24DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F24DATA1_FD11_MSK								(0x01UL << CAN_F24DATA1_FD11_POS)		/** Filter bits */
#define CAN_F24DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F24DATA1_FD12_MSK								(0x01UL << CAN_F24DATA1_FD12_POS)		/** Filter bits */
#define CAN_F24DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F24DATA1_FD13_MSK								(0x01UL << CAN_F24DATA1_FD13_POS)		/** Filter bits */
#define CAN_F24DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F24DATA1_FD14_MSK								(0x01UL << CAN_F24DATA1_FD14_POS)		/** Filter bits */
#define CAN_F24DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F24DATA1_FD15_MSK								(0x01UL << CAN_F24DATA1_FD15_POS)		/** Filter bits */
#define CAN_F24DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F24DATA1_FD16_MSK								(0x01UL << CAN_F24DATA1_FD16_POS)		/** Filter bits */
#define CAN_F24DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F24DATA1_FD17_MSK								(0x01UL << CAN_F24DATA1_FD17_POS)		/** Filter bits */
#define CAN_F24DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F24DATA1_FD18_MSK								(0x01UL << CAN_F24DATA1_FD18_POS)		/** Filter bits */
#define CAN_F24DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F24DATA1_FD19_MSK								(0x01UL << CAN_F24DATA1_FD19_POS)		/** Filter bits */
#define CAN_F24DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F24DATA1_FD20_MSK								(0x01UL << CAN_F24DATA1_FD20_POS)		/** Filter bits */
#define CAN_F24DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F24DATA1_FD21_MSK								(0x01UL << CAN_F24DATA1_FD21_POS)		/** Filter bits */
#define CAN_F24DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F24DATA1_FD22_MSK								(0x01UL << CAN_F24DATA1_FD22_POS)		/** Filter bits */
#define CAN_F24DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F24DATA1_FD23_MSK								(0x01UL << CAN_F24DATA1_FD23_POS)		/** Filter bits */
#define CAN_F24DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F24DATA1_FD24_MSK								(0x01UL << CAN_F24DATA1_FD24_POS)		/** Filter bits */
#define CAN_F24DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F24DATA1_FD25_MSK								(0x01UL << CAN_F24DATA1_FD25_POS)		/** Filter bits */
#define CAN_F24DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F24DATA1_FD26_MSK								(0x01UL << CAN_F24DATA1_FD26_POS)		/** Filter bits */
#define CAN_F24DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F24DATA1_FD27_MSK								(0x01UL << CAN_F24DATA1_FD27_POS)		/** Filter bits */
#define CAN_F24DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F24DATA1_FD28_MSK								(0x01UL << CAN_F24DATA1_FD28_POS)		/** Filter bits */
#define CAN_F24DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F24DATA1_FD29_MSK								(0x01UL << CAN_F24DATA1_FD29_POS)		/** Filter bits */
#define CAN_F24DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F24DATA1_FD30_MSK								(0x01UL << CAN_F24DATA1_FD30_POS)		/** Filter bits */
#define CAN_F24DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F24DATA1_FD31_MSK								(0x01UL << CAN_F24DATA1_FD31_POS)		/** Filter bits */
#define CAN_F25DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F25DATA0_FD0_MSK								(0x01UL << CAN_F25DATA0_FD0_POS)		/** Filter bits */
#define CAN_F25DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F25DATA0_FD1_MSK								(0x01UL << CAN_F25DATA0_FD1_POS)		/** Filter bits */
#define CAN_F25DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F25DATA0_FD2_MSK								(0x01UL << CAN_F25DATA0_FD2_POS)		/** Filter bits */
#define CAN_F25DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F25DATA0_FD3_MSK								(0x01UL << CAN_F25DATA0_FD3_POS)		/** Filter bits */
#define CAN_F25DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F25DATA0_FD4_MSK								(0x01UL << CAN_F25DATA0_FD4_POS)		/** Filter bits */
#define CAN_F25DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F25DATA0_FD5_MSK								(0x01UL << CAN_F25DATA0_FD5_POS)		/** Filter bits */
#define CAN_F25DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F25DATA0_FD6_MSK								(0x01UL << CAN_F25DATA0_FD6_POS)		/** Filter bits */
#define CAN_F25DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F25DATA0_FD7_MSK								(0x01UL << CAN_F25DATA0_FD7_POS)		/** Filter bits */
#define CAN_F25DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F25DATA0_FD8_MSK								(0x01UL << CAN_F25DATA0_FD8_POS)		/** Filter bits */
#define CAN_F25DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F25DATA0_FD9_MSK								(0x01UL << CAN_F25DATA0_FD9_POS)		/** Filter bits */
#define CAN_F25DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F25DATA0_FD10_MSK								(0x01UL << CAN_F25DATA0_FD10_POS)		/** Filter bits */
#define CAN_F25DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F25DATA0_FD11_MSK								(0x01UL << CAN_F25DATA0_FD11_POS)		/** Filter bits */
#define CAN_F25DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F25DATA0_FD12_MSK								(0x01UL << CAN_F25DATA0_FD12_POS)		/** Filter bits */
#define CAN_F25DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F25DATA0_FD13_MSK								(0x01UL << CAN_F25DATA0_FD13_POS)		/** Filter bits */
#define CAN_F25DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F25DATA0_FD14_MSK								(0x01UL << CAN_F25DATA0_FD14_POS)		/** Filter bits */
#define CAN_F25DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F25DATA0_FD15_MSK								(0x01UL << CAN_F25DATA0_FD15_POS)		/** Filter bits */
#define CAN_F25DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F25DATA0_FD16_MSK								(0x01UL << CAN_F25DATA0_FD16_POS)		/** Filter bits */
#define CAN_F25DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F25DATA0_FD17_MSK								(0x01UL << CAN_F25DATA0_FD17_POS)		/** Filter bits */
#define CAN_F25DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F25DATA0_FD18_MSK								(0x01UL << CAN_F25DATA0_FD18_POS)		/** Filter bits */
#define CAN_F25DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F25DATA0_FD19_MSK								(0x01UL << CAN_F25DATA0_FD19_POS)		/** Filter bits */
#define CAN_F25DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F25DATA0_FD20_MSK								(0x01UL << CAN_F25DATA0_FD20_POS)		/** Filter bits */
#define CAN_F25DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F25DATA0_FD21_MSK								(0x01UL << CAN_F25DATA0_FD21_POS)		/** Filter bits */
#define CAN_F25DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F25DATA0_FD22_MSK								(0x01UL << CAN_F25DATA0_FD22_POS)		/** Filter bits */
#define CAN_F25DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F25DATA0_FD23_MSK								(0x01UL << CAN_F25DATA0_FD23_POS)		/** Filter bits */
#define CAN_F25DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F25DATA0_FD24_MSK								(0x01UL << CAN_F25DATA0_FD24_POS)		/** Filter bits */
#define CAN_F25DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F25DATA0_FD25_MSK								(0x01UL << CAN_F25DATA0_FD25_POS)		/** Filter bits */
#define CAN_F25DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F25DATA0_FD26_MSK								(0x01UL << CAN_F25DATA0_FD26_POS)		/** Filter bits */
#define CAN_F25DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F25DATA0_FD27_MSK								(0x01UL << CAN_F25DATA0_FD27_POS)		/** Filter bits */
#define CAN_F25DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F25DATA0_FD28_MSK								(0x01UL << CAN_F25DATA0_FD28_POS)		/** Filter bits */
#define CAN_F25DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F25DATA0_FD29_MSK								(0x01UL << CAN_F25DATA0_FD29_POS)		/** Filter bits */
#define CAN_F25DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F25DATA0_FD30_MSK								(0x01UL << CAN_F25DATA0_FD30_POS)		/** Filter bits */
#define CAN_F25DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F25DATA0_FD31_MSK								(0x01UL << CAN_F25DATA0_FD31_POS)		/** Filter bits */
#define CAN_F25DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F25DATA1_FD0_MSK								(0x01UL << CAN_F25DATA1_FD0_POS)		/** Filter bits */
#define CAN_F25DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F25DATA1_FD1_MSK								(0x01UL << CAN_F25DATA1_FD1_POS)		/** Filter bits */
#define CAN_F25DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F25DATA1_FD2_MSK								(0x01UL << CAN_F25DATA1_FD2_POS)		/** Filter bits */
#define CAN_F25DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F25DATA1_FD3_MSK								(0x01UL << CAN_F25DATA1_FD3_POS)		/** Filter bits */
#define CAN_F25DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F25DATA1_FD4_MSK								(0x01UL << CAN_F25DATA1_FD4_POS)		/** Filter bits */
#define CAN_F25DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F25DATA1_FD5_MSK								(0x01UL << CAN_F25DATA1_FD5_POS)		/** Filter bits */
#define CAN_F25DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F25DATA1_FD6_MSK								(0x01UL << CAN_F25DATA1_FD6_POS)		/** Filter bits */
#define CAN_F25DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F25DATA1_FD7_MSK								(0x01UL << CAN_F25DATA1_FD7_POS)		/** Filter bits */
#define CAN_F25DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F25DATA1_FD8_MSK								(0x01UL << CAN_F25DATA1_FD8_POS)		/** Filter bits */
#define CAN_F25DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F25DATA1_FD9_MSK								(0x01UL << CAN_F25DATA1_FD9_POS)		/** Filter bits */
#define CAN_F25DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F25DATA1_FD10_MSK								(0x01UL << CAN_F25DATA1_FD10_POS)		/** Filter bits */
#define CAN_F25DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F25DATA1_FD11_MSK								(0x01UL << CAN_F25DATA1_FD11_POS)		/** Filter bits */
#define CAN_F25DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F25DATA1_FD12_MSK								(0x01UL << CAN_F25DATA1_FD12_POS)		/** Filter bits */
#define CAN_F25DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F25DATA1_FD13_MSK								(0x01UL << CAN_F25DATA1_FD13_POS)		/** Filter bits */
#define CAN_F25DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F25DATA1_FD14_MSK								(0x01UL << CAN_F25DATA1_FD14_POS)		/** Filter bits */
#define CAN_F25DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F25DATA1_FD15_MSK								(0x01UL << CAN_F25DATA1_FD15_POS)		/** Filter bits */
#define CAN_F25DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F25DATA1_FD16_MSK								(0x01UL << CAN_F25DATA1_FD16_POS)		/** Filter bits */
#define CAN_F25DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F25DATA1_FD17_MSK								(0x01UL << CAN_F25DATA1_FD17_POS)		/** Filter bits */
#define CAN_F25DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F25DATA1_FD18_MSK								(0x01UL << CAN_F25DATA1_FD18_POS)		/** Filter bits */
#define CAN_F25DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F25DATA1_FD19_MSK								(0x01UL << CAN_F25DATA1_FD19_POS)		/** Filter bits */
#define CAN_F25DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F25DATA1_FD20_MSK								(0x01UL << CAN_F25DATA1_FD20_POS)		/** Filter bits */
#define CAN_F25DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F25DATA1_FD21_MSK								(0x01UL << CAN_F25DATA1_FD21_POS)		/** Filter bits */
#define CAN_F25DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F25DATA1_FD22_MSK								(0x01UL << CAN_F25DATA1_FD22_POS)		/** Filter bits */
#define CAN_F25DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F25DATA1_FD23_MSK								(0x01UL << CAN_F25DATA1_FD23_POS)		/** Filter bits */
#define CAN_F25DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F25DATA1_FD24_MSK								(0x01UL << CAN_F25DATA1_FD24_POS)		/** Filter bits */
#define CAN_F25DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F25DATA1_FD25_MSK								(0x01UL << CAN_F25DATA1_FD25_POS)		/** Filter bits */
#define CAN_F25DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F25DATA1_FD26_MSK								(0x01UL << CAN_F25DATA1_FD26_POS)		/** Filter bits */
#define CAN_F25DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F25DATA1_FD27_MSK								(0x01UL << CAN_F25DATA1_FD27_POS)		/** Filter bits */
#define CAN_F25DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F25DATA1_FD28_MSK								(0x01UL << CAN_F25DATA1_FD28_POS)		/** Filter bits */
#define CAN_F25DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F25DATA1_FD29_MSK								(0x01UL << CAN_F25DATA1_FD29_POS)		/** Filter bits */
#define CAN_F25DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F25DATA1_FD30_MSK								(0x01UL << CAN_F25DATA1_FD30_POS)		/** Filter bits */
#define CAN_F25DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F25DATA1_FD31_MSK								(0x01UL << CAN_F25DATA1_FD31_POS)		/** Filter bits */
#define CAN_F26DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F26DATA0_FD0_MSK								(0x01UL << CAN_F26DATA0_FD0_POS)		/** Filter bits */
#define CAN_F26DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F26DATA0_FD1_MSK								(0x01UL << CAN_F26DATA0_FD1_POS)		/** Filter bits */
#define CAN_F26DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F26DATA0_FD2_MSK								(0x01UL << CAN_F26DATA0_FD2_POS)		/** Filter bits */
#define CAN_F26DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F26DATA0_FD3_MSK								(0x01UL << CAN_F26DATA0_FD3_POS)		/** Filter bits */
#define CAN_F26DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F26DATA0_FD4_MSK								(0x01UL << CAN_F26DATA0_FD4_POS)		/** Filter bits */
#define CAN_F26DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F26DATA0_FD5_MSK								(0x01UL << CAN_F26DATA0_FD5_POS)		/** Filter bits */
#define CAN_F26DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F26DATA0_FD6_MSK								(0x01UL << CAN_F26DATA0_FD6_POS)		/** Filter bits */
#define CAN_F26DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F26DATA0_FD7_MSK								(0x01UL << CAN_F26DATA0_FD7_POS)		/** Filter bits */
#define CAN_F26DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F26DATA0_FD8_MSK								(0x01UL << CAN_F26DATA0_FD8_POS)		/** Filter bits */
#define CAN_F26DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F26DATA0_FD9_MSK								(0x01UL << CAN_F26DATA0_FD9_POS)		/** Filter bits */
#define CAN_F26DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F26DATA0_FD10_MSK								(0x01UL << CAN_F26DATA0_FD10_POS)		/** Filter bits */
#define CAN_F26DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F26DATA0_FD11_MSK								(0x01UL << CAN_F26DATA0_FD11_POS)		/** Filter bits */
#define CAN_F26DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F26DATA0_FD12_MSK								(0x01UL << CAN_F26DATA0_FD12_POS)		/** Filter bits */
#define CAN_F26DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F26DATA0_FD13_MSK								(0x01UL << CAN_F26DATA0_FD13_POS)		/** Filter bits */
#define CAN_F26DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F26DATA0_FD14_MSK								(0x01UL << CAN_F26DATA0_FD14_POS)		/** Filter bits */
#define CAN_F26DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F26DATA0_FD15_MSK								(0x01UL << CAN_F26DATA0_FD15_POS)		/** Filter bits */
#define CAN_F26DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F26DATA0_FD16_MSK								(0x01UL << CAN_F26DATA0_FD16_POS)		/** Filter bits */
#define CAN_F26DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F26DATA0_FD17_MSK								(0x01UL << CAN_F26DATA0_FD17_POS)		/** Filter bits */
#define CAN_F26DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F26DATA0_FD18_MSK								(0x01UL << CAN_F26DATA0_FD18_POS)		/** Filter bits */
#define CAN_F26DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F26DATA0_FD19_MSK								(0x01UL << CAN_F26DATA0_FD19_POS)		/** Filter bits */
#define CAN_F26DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F26DATA0_FD20_MSK								(0x01UL << CAN_F26DATA0_FD20_POS)		/** Filter bits */
#define CAN_F26DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F26DATA0_FD21_MSK								(0x01UL << CAN_F26DATA0_FD21_POS)		/** Filter bits */
#define CAN_F26DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F26DATA0_FD22_MSK								(0x01UL << CAN_F26DATA0_FD22_POS)		/** Filter bits */
#define CAN_F26DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F26DATA0_FD23_MSK								(0x01UL << CAN_F26DATA0_FD23_POS)		/** Filter bits */
#define CAN_F26DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F26DATA0_FD24_MSK								(0x01UL << CAN_F26DATA0_FD24_POS)		/** Filter bits */
#define CAN_F26DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F26DATA0_FD25_MSK								(0x01UL << CAN_F26DATA0_FD25_POS)		/** Filter bits */
#define CAN_F26DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F26DATA0_FD26_MSK								(0x01UL << CAN_F26DATA0_FD26_POS)		/** Filter bits */
#define CAN_F26DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F26DATA0_FD27_MSK								(0x01UL << CAN_F26DATA0_FD27_POS)		/** Filter bits */
#define CAN_F26DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F26DATA0_FD28_MSK								(0x01UL << CAN_F26DATA0_FD28_POS)		/** Filter bits */
#define CAN_F26DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F26DATA0_FD29_MSK								(0x01UL << CAN_F26DATA0_FD29_POS)		/** Filter bits */
#define CAN_F26DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F26DATA0_FD30_MSK								(0x01UL << CAN_F26DATA0_FD30_POS)		/** Filter bits */
#define CAN_F26DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F26DATA0_FD31_MSK								(0x01UL << CAN_F26DATA0_FD31_POS)		/** Filter bits */
#define CAN_F26DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F26DATA1_FD0_MSK								(0x01UL << CAN_F26DATA1_FD0_POS)		/** Filter bits */
#define CAN_F26DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F26DATA1_FD1_MSK								(0x01UL << CAN_F26DATA1_FD1_POS)		/** Filter bits */
#define CAN_F26DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F26DATA1_FD2_MSK								(0x01UL << CAN_F26DATA1_FD2_POS)		/** Filter bits */
#define CAN_F26DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F26DATA1_FD3_MSK								(0x01UL << CAN_F26DATA1_FD3_POS)		/** Filter bits */
#define CAN_F26DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F26DATA1_FD4_MSK								(0x01UL << CAN_F26DATA1_FD4_POS)		/** Filter bits */
#define CAN_F26DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F26DATA1_FD5_MSK								(0x01UL << CAN_F26DATA1_FD5_POS)		/** Filter bits */
#define CAN_F26DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F26DATA1_FD6_MSK								(0x01UL << CAN_F26DATA1_FD6_POS)		/** Filter bits */
#define CAN_F26DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F26DATA1_FD7_MSK								(0x01UL << CAN_F26DATA1_FD7_POS)		/** Filter bits */
#define CAN_F26DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F26DATA1_FD8_MSK								(0x01UL << CAN_F26DATA1_FD8_POS)		/** Filter bits */
#define CAN_F26DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F26DATA1_FD9_MSK								(0x01UL << CAN_F26DATA1_FD9_POS)		/** Filter bits */
#define CAN_F26DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F26DATA1_FD10_MSK								(0x01UL << CAN_F26DATA1_FD10_POS)		/** Filter bits */
#define CAN_F26DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F26DATA1_FD11_MSK								(0x01UL << CAN_F26DATA1_FD11_POS)		/** Filter bits */
#define CAN_F26DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F26DATA1_FD12_MSK								(0x01UL << CAN_F26DATA1_FD12_POS)		/** Filter bits */
#define CAN_F26DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F26DATA1_FD13_MSK								(0x01UL << CAN_F26DATA1_FD13_POS)		/** Filter bits */
#define CAN_F26DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F26DATA1_FD14_MSK								(0x01UL << CAN_F26DATA1_FD14_POS)		/** Filter bits */
#define CAN_F26DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F26DATA1_FD15_MSK								(0x01UL << CAN_F26DATA1_FD15_POS)		/** Filter bits */
#define CAN_F26DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F26DATA1_FD16_MSK								(0x01UL << CAN_F26DATA1_FD16_POS)		/** Filter bits */
#define CAN_F26DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F26DATA1_FD17_MSK								(0x01UL << CAN_F26DATA1_FD17_POS)		/** Filter bits */
#define CAN_F26DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F26DATA1_FD18_MSK								(0x01UL << CAN_F26DATA1_FD18_POS)		/** Filter bits */
#define CAN_F26DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F26DATA1_FD19_MSK								(0x01UL << CAN_F26DATA1_FD19_POS)		/** Filter bits */
#define CAN_F26DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F26DATA1_FD20_MSK								(0x01UL << CAN_F26DATA1_FD20_POS)		/** Filter bits */
#define CAN_F26DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F26DATA1_FD21_MSK								(0x01UL << CAN_F26DATA1_FD21_POS)		/** Filter bits */
#define CAN_F26DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F26DATA1_FD22_MSK								(0x01UL << CAN_F26DATA1_FD22_POS)		/** Filter bits */
#define CAN_F26DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F26DATA1_FD23_MSK								(0x01UL << CAN_F26DATA1_FD23_POS)		/** Filter bits */
#define CAN_F26DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F26DATA1_FD24_MSK								(0x01UL << CAN_F26DATA1_FD24_POS)		/** Filter bits */
#define CAN_F26DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F26DATA1_FD25_MSK								(0x01UL << CAN_F26DATA1_FD25_POS)		/** Filter bits */
#define CAN_F26DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F26DATA1_FD26_MSK								(0x01UL << CAN_F26DATA1_FD26_POS)		/** Filter bits */
#define CAN_F26DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F26DATA1_FD27_MSK								(0x01UL << CAN_F26DATA1_FD27_POS)		/** Filter bits */
#define CAN_F26DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F26DATA1_FD28_MSK								(0x01UL << CAN_F26DATA1_FD28_POS)		/** Filter bits */
#define CAN_F26DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F26DATA1_FD29_MSK								(0x01UL << CAN_F26DATA1_FD29_POS)		/** Filter bits */
#define CAN_F26DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F26DATA1_FD30_MSK								(0x01UL << CAN_F26DATA1_FD30_POS)		/** Filter bits */
#define CAN_F26DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F26DATA1_FD31_MSK								(0x01UL << CAN_F26DATA1_FD31_POS)		/** Filter bits */
#define CAN_F27DATA0_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F27DATA0_FD0_MSK								(0x01UL << CAN_F27DATA0_FD0_POS)		/** Filter bits */
#define CAN_F27DATA0_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F27DATA0_FD1_MSK								(0x01UL << CAN_F27DATA0_FD1_POS)		/** Filter bits */
#define CAN_F27DATA0_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F27DATA0_FD2_MSK								(0x01UL << CAN_F27DATA0_FD2_POS)		/** Filter bits */
#define CAN_F27DATA0_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F27DATA0_FD3_MSK								(0x01UL << CAN_F27DATA0_FD3_POS)		/** Filter bits */
#define CAN_F27DATA0_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F27DATA0_FD4_MSK								(0x01UL << CAN_F27DATA0_FD4_POS)		/** Filter bits */
#define CAN_F27DATA0_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F27DATA0_FD5_MSK								(0x01UL << CAN_F27DATA0_FD5_POS)		/** Filter bits */
#define CAN_F27DATA0_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F27DATA0_FD6_MSK								(0x01UL << CAN_F27DATA0_FD6_POS)		/** Filter bits */
#define CAN_F27DATA0_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F27DATA0_FD7_MSK								(0x01UL << CAN_F27DATA0_FD7_POS)		/** Filter bits */
#define CAN_F27DATA0_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F27DATA0_FD8_MSK								(0x01UL << CAN_F27DATA0_FD8_POS)		/** Filter bits */
#define CAN_F27DATA0_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F27DATA0_FD9_MSK								(0x01UL << CAN_F27DATA0_FD9_POS)		/** Filter bits */
#define CAN_F27DATA0_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F27DATA0_FD10_MSK								(0x01UL << CAN_F27DATA0_FD10_POS)		/** Filter bits */
#define CAN_F27DATA0_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F27DATA0_FD11_MSK								(0x01UL << CAN_F27DATA0_FD11_POS)		/** Filter bits */
#define CAN_F27DATA0_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F27DATA0_FD12_MSK								(0x01UL << CAN_F27DATA0_FD12_POS)		/** Filter bits */
#define CAN_F27DATA0_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F27DATA0_FD13_MSK								(0x01UL << CAN_F27DATA0_FD13_POS)		/** Filter bits */
#define CAN_F27DATA0_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F27DATA0_FD14_MSK								(0x01UL << CAN_F27DATA0_FD14_POS)		/** Filter bits */
#define CAN_F27DATA0_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F27DATA0_FD15_MSK								(0x01UL << CAN_F27DATA0_FD15_POS)		/** Filter bits */
#define CAN_F27DATA0_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F27DATA0_FD16_MSK								(0x01UL << CAN_F27DATA0_FD16_POS)		/** Filter bits */
#define CAN_F27DATA0_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F27DATA0_FD17_MSK								(0x01UL << CAN_F27DATA0_FD17_POS)		/** Filter bits */
#define CAN_F27DATA0_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F27DATA0_FD18_MSK								(0x01UL << CAN_F27DATA0_FD18_POS)		/** Filter bits */
#define CAN_F27DATA0_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F27DATA0_FD19_MSK								(0x01UL << CAN_F27DATA0_FD19_POS)		/** Filter bits */
#define CAN_F27DATA0_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F27DATA0_FD20_MSK								(0x01UL << CAN_F27DATA0_FD20_POS)		/** Filter bits */
#define CAN_F27DATA0_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F27DATA0_FD21_MSK								(0x01UL << CAN_F27DATA0_FD21_POS)		/** Filter bits */
#define CAN_F27DATA0_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F27DATA0_FD22_MSK								(0x01UL << CAN_F27DATA0_FD22_POS)		/** Filter bits */
#define CAN_F27DATA0_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F27DATA0_FD23_MSK								(0x01UL << CAN_F27DATA0_FD23_POS)		/** Filter bits */
#define CAN_F27DATA0_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F27DATA0_FD24_MSK								(0x01UL << CAN_F27DATA0_FD24_POS)		/** Filter bits */
#define CAN_F27DATA0_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F27DATA0_FD25_MSK								(0x01UL << CAN_F27DATA0_FD25_POS)		/** Filter bits */
#define CAN_F27DATA0_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F27DATA0_FD26_MSK								(0x01UL << CAN_F27DATA0_FD26_POS)		/** Filter bits */
#define CAN_F27DATA0_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F27DATA0_FD27_MSK								(0x01UL << CAN_F27DATA0_FD27_POS)		/** Filter bits */
#define CAN_F27DATA0_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F27DATA0_FD28_MSK								(0x01UL << CAN_F27DATA0_FD28_POS)		/** Filter bits */
#define CAN_F27DATA0_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F27DATA0_FD29_MSK								(0x01UL << CAN_F27DATA0_FD29_POS)		/** Filter bits */
#define CAN_F27DATA0_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F27DATA0_FD30_MSK								(0x01UL << CAN_F27DATA0_FD30_POS)		/** Filter bits */
#define CAN_F27DATA0_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F27DATA0_FD31_MSK								(0x01UL << CAN_F27DATA0_FD31_POS)		/** Filter bits */
#define CAN_F27DATA1_FD0_POS								0x00UL		/** Filter bits */
#define CAN_F27DATA1_FD0_MSK								(0x01UL << CAN_F27DATA1_FD0_POS)		/** Filter bits */
#define CAN_F27DATA1_FD1_POS								0x01UL		/** Filter bits */
#define CAN_F27DATA1_FD1_MSK								(0x01UL << CAN_F27DATA1_FD1_POS)		/** Filter bits */
#define CAN_F27DATA1_FD2_POS								0x02UL		/** Filter bits */
#define CAN_F27DATA1_FD2_MSK								(0x01UL << CAN_F27DATA1_FD2_POS)		/** Filter bits */
#define CAN_F27DATA1_FD3_POS								0x03UL		/** Filter bits */
#define CAN_F27DATA1_FD3_MSK								(0x01UL << CAN_F27DATA1_FD3_POS)		/** Filter bits */
#define CAN_F27DATA1_FD4_POS								0x04UL		/** Filter bits */
#define CAN_F27DATA1_FD4_MSK								(0x01UL << CAN_F27DATA1_FD4_POS)		/** Filter bits */
#define CAN_F27DATA1_FD5_POS								0x05UL		/** Filter bits */
#define CAN_F27DATA1_FD5_MSK								(0x01UL << CAN_F27DATA1_FD5_POS)		/** Filter bits */
#define CAN_F27DATA1_FD6_POS								0x06UL		/** Filter bits */
#define CAN_F27DATA1_FD6_MSK								(0x01UL << CAN_F27DATA1_FD6_POS)		/** Filter bits */
#define CAN_F27DATA1_FD7_POS								0x07UL		/** Filter bits */
#define CAN_F27DATA1_FD7_MSK								(0x01UL << CAN_F27DATA1_FD7_POS)		/** Filter bits */
#define CAN_F27DATA1_FD8_POS								0x08UL		/** Filter bits */
#define CAN_F27DATA1_FD8_MSK								(0x01UL << CAN_F27DATA1_FD8_POS)		/** Filter bits */
#define CAN_F27DATA1_FD9_POS								0x09UL		/** Filter bits */
#define CAN_F27DATA1_FD9_MSK								(0x01UL << CAN_F27DATA1_FD9_POS)		/** Filter bits */
#define CAN_F27DATA1_FD10_POS								0x0AUL		/** Filter bits */
#define CAN_F27DATA1_FD10_MSK								(0x01UL << CAN_F27DATA1_FD10_POS)		/** Filter bits */
#define CAN_F27DATA1_FD11_POS								0x0BUL		/** Filter bits */
#define CAN_F27DATA1_FD11_MSK								(0x01UL << CAN_F27DATA1_FD11_POS)		/** Filter bits */
#define CAN_F27DATA1_FD12_POS								0x0CUL		/** Filter bits */
#define CAN_F27DATA1_FD12_MSK								(0x01UL << CAN_F27DATA1_FD12_POS)		/** Filter bits */
#define CAN_F27DATA1_FD13_POS								0x0DUL		/** Filter bits */
#define CAN_F27DATA1_FD13_MSK								(0x01UL << CAN_F27DATA1_FD13_POS)		/** Filter bits */
#define CAN_F27DATA1_FD14_POS								0x0EUL		/** Filter bits */
#define CAN_F27DATA1_FD14_MSK								(0x01UL << CAN_F27DATA1_FD14_POS)		/** Filter bits */
#define CAN_F27DATA1_FD15_POS								0x0FUL		/** Filter bits */
#define CAN_F27DATA1_FD15_MSK								(0x01UL << CAN_F27DATA1_FD15_POS)		/** Filter bits */
#define CAN_F27DATA1_FD16_POS								0x10UL		/** Filter bits */
#define CAN_F27DATA1_FD16_MSK								(0x01UL << CAN_F27DATA1_FD16_POS)		/** Filter bits */
#define CAN_F27DATA1_FD17_POS								0x11UL		/** Filter bits */
#define CAN_F27DATA1_FD17_MSK								(0x01UL << CAN_F27DATA1_FD17_POS)		/** Filter bits */
#define CAN_F27DATA1_FD18_POS								0x12UL		/** Filter bits */
#define CAN_F27DATA1_FD18_MSK								(0x01UL << CAN_F27DATA1_FD18_POS)		/** Filter bits */
#define CAN_F27DATA1_FD19_POS								0x13UL		/** Filter bits */
#define CAN_F27DATA1_FD19_MSK								(0x01UL << CAN_F27DATA1_FD19_POS)		/** Filter bits */
#define CAN_F27DATA1_FD20_POS								0x14UL		/** Filter bits */
#define CAN_F27DATA1_FD20_MSK								(0x01UL << CAN_F27DATA1_FD20_POS)		/** Filter bits */
#define CAN_F27DATA1_FD21_POS								0x15UL		/** Filter bits */
#define CAN_F27DATA1_FD21_MSK								(0x01UL << CAN_F27DATA1_FD21_POS)		/** Filter bits */
#define CAN_F27DATA1_FD22_POS								0x16UL		/** Filter bits */
#define CAN_F27DATA1_FD22_MSK								(0x01UL << CAN_F27DATA1_FD22_POS)		/** Filter bits */
#define CAN_F27DATA1_FD23_POS								0x17UL		/** Filter bits */
#define CAN_F27DATA1_FD23_MSK								(0x01UL << CAN_F27DATA1_FD23_POS)		/** Filter bits */
#define CAN_F27DATA1_FD24_POS								0x18UL		/** Filter bits */
#define CAN_F27DATA1_FD24_MSK								(0x01UL << CAN_F27DATA1_FD24_POS)		/** Filter bits */
#define CAN_F27DATA1_FD25_POS								0x19UL		/** Filter bits */
#define CAN_F27DATA1_FD25_MSK								(0x01UL << CAN_F27DATA1_FD25_POS)		/** Filter bits */
#define CAN_F27DATA1_FD26_POS								0x1AUL		/** Filter bits */
#define CAN_F27DATA1_FD26_MSK								(0x01UL << CAN_F27DATA1_FD26_POS)		/** Filter bits */
#define CAN_F27DATA1_FD27_POS								0x1BUL		/** Filter bits */
#define CAN_F27DATA1_FD27_MSK								(0x01UL << CAN_F27DATA1_FD27_POS)		/** Filter bits */
#define CAN_F27DATA1_FD28_POS								0x1CUL		/** Filter bits */
#define CAN_F27DATA1_FD28_MSK								(0x01UL << CAN_F27DATA1_FD28_POS)		/** Filter bits */
#define CAN_F27DATA1_FD29_POS								0x1DUL		/** Filter bits */
#define CAN_F27DATA1_FD29_MSK								(0x01UL << CAN_F27DATA1_FD29_POS)		/** Filter bits */
#define CAN_F27DATA1_FD30_POS								0x1EUL		/** Filter bits */
#define CAN_F27DATA1_FD30_MSK								(0x01UL << CAN_F27DATA1_FD30_POS)		/** Filter bits */
#define CAN_F27DATA1_FD31_POS								0x1FUL		/** Filter bits */
#define CAN_F27DATA1_FD31_MSK								(0x01UL << CAN_F27DATA1_FD31_POS)		/** Filter bits */
#define CRC_DATA_DATA_POS								    0x00UL		/** CRC calculation result bits */
#define CRC_DATA_DATA_MSK								    (0xFFFFFFFFUL << CRC_DATA_DATA_POS)		/** CRC calculation result bits */
#define CRC_FDATA_FDATA_POS								  0x00UL		/** Free Data Register bits */
#define CRC_FDATA_FDATA_MSK								  (0xFFUL << CRC_FDATA_FDATA_POS)		/** Free Data Register bits */
#define CRC_CTL_RST_POS								0x00UL		/** reset bit */
#define CRC_CTL_RST_MSK								(0x01UL << CRC_CTL_RST_POS)		/** reset bit */
#define DAC_CTL_DEN0_POS								0x00UL		/** DAC0 enable */
#define DAC_CTL_DEN0_MSK								(0x01UL << DAC_CTL_DEN0_POS)		/** DAC0 enable */
#define DAC_CTL_DBOFF0_POS								0x01UL		/** DAC0 output buffer turn off */
#define DAC_CTL_DBOFF0_MSK								(0x01UL << DAC_CTL_DBOFF0_POS)		/** DAC0 output buffer turn off */
#define DAC_CTL_DTEN0_POS								0x02UL		/** DAC0 trigger enable */
#define DAC_CTL_DTEN0_MSK								(0x01UL << DAC_CTL_DTEN0_POS)		/** DAC0 trigger enable */
#define DAC_CTL_DTSEL0_POS								0x03UL		/** DAC0 trigger selection */
#define DAC_CTL_DTSEL0_MSK								(0x07UL << DAC_CTL_DTSEL0_POS)		/** DAC0 trigger selection */
#define DAC_CTL_DWM0_POS								0x06UL		/** DAC0 noise wave mode */
#define DAC_CTL_DWM0_MSK								(0x03UL << DAC_CTL_DWM0_POS)		/** DAC0 noise wave mode */
#define DAC_CTL_DWBW0_POS								0x08UL		/** DAC0 noise wave bit width */
#define DAC_CTL_DWBW0_MSK								(0x0FUL << DAC_CTL_DWBW0_POS)		/** DAC0 noise wave bit width */
#define DAC_CTL_DDMAEN0_POS								0x0CUL		/** DAC0 DMA enable */
#define DAC_CTL_DDMAEN0_MSK								(0x01UL << DAC_CTL_DDMAEN0_POS)		/** DAC0 DMA enable */
#define DAC_CTL_DEN1_POS								0x10UL		/** DAC1 enable */
#define DAC_CTL_DEN1_MSK								(0x01UL << DAC_CTL_DEN1_POS)		/** DAC1 enable */
#define DAC_CTL_DBOFF1_POS								0x11UL		/** DAC1 output buffer turn off */
#define DAC_CTL_DBOFF1_MSK								(0x01UL << DAC_CTL_DBOFF1_POS)		/** DAC1 output buffer turn off */
#define DAC_CTL_DTEN1_POS								0x12UL		/** DAC1 trigger enable */
#define DAC_CTL_DTEN1_MSK								(0x01UL << DAC_CTL_DTEN1_POS)		/** DAC1 trigger enable */
#define DAC_CTL_DTSEL1_POS								0x13UL		/** DAC1 trigger selection */
#define DAC_CTL_DTSEL1_MSK								(0x07UL << DAC_CTL_DTSEL1_POS)		/** DAC1 trigger selection */
#define DAC_CTL_DWM1_POS								0x16UL		/** DAC1 noise wave mode */
#define DAC_CTL_DWM1_MSK								(0x03UL << DAC_CTL_DWM1_POS)		/** DAC1 noise wave mode */
#define DAC_CTL_DWBW1_POS								0x18UL		/** DAC1 noise wave bit width */
#define DAC_CTL_DWBW1_MSK								(0x0FUL << DAC_CTL_DWBW1_POS)		/** DAC1 noise wave bit width */
#define DAC_CTL_DDMAEN1_POS								0x1CUL		/** DAC1 DMA enable */
#define DAC_CTL_DDMAEN1_MSK								(0x01UL << DAC_CTL_DDMAEN1_POS)		/** DAC1 DMA enable */
#define DAC_SWT_SWTR0_POS								0x00UL		/** DAC0 software trigger */
#define DAC_SWT_SWTR0_MSK								(0x01UL << DAC_SWT_SWTR0_POS)		/** DAC0 software trigger */
#define DAC_SWT_SWTR1_POS								0x01UL		/** DAC1 software trigger */
#define DAC_SWT_SWTR1_MSK								(0x01UL << DAC_SWT_SWTR1_POS)		/** DAC1 software trigger */
#define DAC_DAC0_R12DH_DAC0_DH_POS								0x00UL		/** DAC0 12-bit right-aligned data */
#define DAC_DAC0_R12DH_DAC0_DH_MSK								(0xFFFUL << DAC_DAC0_R12DH_DAC0_DH_POS)		/** DAC0 12-bit right-aligned data */
#define DAC_DAC0_L12DH_DAC0_DH_POS								0x04UL		/** DAC0 12-bit left-aligned data */
#define DAC_DAC0_L12DH_DAC0_DH_MSK								(0xFFFUL << DAC_DAC0_L12DH_DAC0_DH_POS)		/** DAC0 12-bit left-aligned data */
#define DAC_DAC0_R8DH_DAC0_DH_POS								0x00UL		/** DAC0 8-bit right-aligned data */
#define DAC_DAC0_R8DH_DAC0_DH_MSK								(0xFFUL << DAC_DAC0_R8DH_DAC0_DH_POS)		/** DAC0 8-bit right-aligned data */
#define DAC_DAC1_R12DH_DAC1_DH_POS								0x00UL		/** DAC1 12-bit right-aligned data */
#define DAC_DAC1_R12DH_DAC1_DH_MSK								(0xFFFUL << DAC_DAC1_R12DH_DAC1_DH_POS)		/** DAC1 12-bit right-aligned data */
#define DAC_DAC1_L12DH_DAC1_DH_POS								0x04UL		/** DAC1 12-bit left-aligned data */
#define DAC_DAC1_L12DH_DAC1_DH_MSK								(0xFFFUL << DAC_DAC1_L12DH_DAC1_DH_POS)		/** DAC1 12-bit left-aligned data */
#define DAC_DAC1_R8DH_DAC1_DH_POS								0x00UL		/** DAC1 8-bit right-aligned data */
#define DAC_DAC1_R8DH_DAC1_DH_MSK								(0xFFUL << DAC_DAC1_R8DH_DAC1_DH_POS)		/** DAC1 8-bit right-aligned data */
#define DAC_DACC_R12DH_DAC0_DH_POS								0x00UL		/** DAC0 12-bit right-aligned data */
#define DAC_DACC_R12DH_DAC0_DH_MSK								(0xFFFUL << DAC_DACC_R12DH_DAC0_DH_POS)		/** DAC0 12-bit right-aligned data */
#define DAC_DACC_R12DH_DAC1_DH_POS								0x10UL		/** DAC1 12-bit right-aligned data */
#define DAC_DACC_R12DH_DAC1_DH_MSK								(0xFFFUL << DAC_DACC_R12DH_DAC1_DH_POS)		/** DAC1 12-bit right-aligned data */
#define DAC_DACC_L12DH_DAC0_DH_POS								0x04UL		/** DAC0 12-bit left-aligned data */
#define DAC_DACC_L12DH_DAC0_DH_MSK								(0xFFFUL << DAC_DACC_L12DH_DAC0_DH_POS)		/** DAC0 12-bit left-aligned data */
#define DAC_DACC_L12DH_DAC1_DH_POS								0x14UL		/** DAC1 12-bit left-aligned data */
#define DAC_DACC_L12DH_DAC1_DH_MSK								(0xFFFUL << DAC_DACC_L12DH_DAC1_DH_POS)		/** DAC1 12-bit left-aligned data */
#define DAC_DACC_R8DH_DAC0_DH_POS								0x00UL		/** DAC0 8-bit right-aligned data */
#define DAC_DACC_R8DH_DAC0_DH_MSK								(0xFFUL << DAC_DACC_R8DH_DAC0_DH_POS)		/** DAC0 8-bit right-aligned data */
#define DAC_DACC_R8DH_DAC1_DH_POS								0x08UL		/** DAC1 8-bit right-aligned data */
#define DAC_DACC_R8DH_DAC1_DH_MSK								(0xFFUL << DAC_DACC_R8DH_DAC1_DH_POS)		/** DAC1 8-bit right-aligned data */
#define DAC_DAC0_DO_DAC0_DO_POS								0x00UL		/** DAC0 data output */
#define DAC_DAC0_DO_DAC0_DO_MSK								(0xFFFUL << DAC_DAC0_DO_DAC0_DO_POS)		/** DAC0 data output */
#define DAC_DAC1_DO_DAC1_DO_POS								0x00UL		/** DAC1 data output */
#define DAC_DAC1_DO_DAC1_DO_MSK								(0xFFFUL << DAC_DAC1_DO_DAC1_DO_POS)		/** DAC1 data output */
#define DBG_ID_ID_CODE_POS								0x00UL		/** DBG ID code register */
#define DBG_ID_ID_CODE_MSK								(0xFFFFFFFFUL << DBG_ID_ID_CODE_POS)		/** DBG ID code register */
#define DBG_CTL_SLP_HOLD_POS								0x00UL		/** Sleep mode hold register */
#define DBG_CTL_SLP_HOLD_MSK								(0x01UL << DBG_CTL_SLP_HOLD_POS)		/** Sleep mode hold register */
#define DBG_CTL_DSLP_HOLD_POS								0x01UL		/** Deep-sleep mode hold register */
#define DBG_CTL_DSLP_HOLD_MSK								(0x01UL << DBG_CTL_DSLP_HOLD_POS)		/** Deep-sleep mode hold register */
#define DBG_CTL_STB_HOLD_POS								0x02UL		/** Standby mode hold register */
#define DBG_CTL_STB_HOLD_MSK								(0x01UL << DBG_CTL_STB_HOLD_POS)		/** Standby mode hold register */
#define DBG_CTL_FWDGT_HOLD_POS								0x08UL		/** FWDGT hold bit */
#define DBG_CTL_FWDGT_HOLD_MSK								(0x01UL << DBG_CTL_FWDGT_HOLD_POS)		/** FWDGT hold bit */
#define DBG_CTL_WWDGT_HOLD_POS								0x09UL		/** WWDGT hold bit */
#define DBG_CTL_WWDGT_HOLD_MSK								(0x01UL << DBG_CTL_WWDGT_HOLD_POS)		/** WWDGT hold bit */
#define DBG_CTL_TIMER0_HOLD_POS								0x0AUL		/** TIMER 0 hold bit */
#define DBG_CTL_TIMER0_HOLD_MSK								(0x01UL << DBG_CTL_TIMER0_HOLD_POS)		/** TIMER 0 hold bit */
#define DBG_CTL_TIMER1_HOLD_POS								0x0BUL		/** TIMER 1 hold bit */
#define DBG_CTL_TIMER1_HOLD_MSK								(0x01UL << DBG_CTL_TIMER1_HOLD_POS)		/** TIMER 1 hold bit */
#define DBG_CTL_TIMER2_HOLD_POS								0x0CUL		/** TIMER 2 hold bit */
#define DBG_CTL_TIMER2_HOLD_MSK								(0x01UL << DBG_CTL_TIMER2_HOLD_POS)		/** TIMER 2 hold bit */
#define DBG_CTL_TIMER3_HOLD_POS								0x0DUL		/** TIMER 23 hold bit */
#define DBG_CTL_TIMER3_HOLD_MSK								(0x01UL << DBG_CTL_TIMER3_HOLD_POS)		/** TIMER 23 hold bit */
#define DBG_CTL_CAN0_HOLD_POS								0x0EUL		/** CAN0 hold bit */
#define DBG_CTL_CAN0_HOLD_MSK								(0x01UL << DBG_CTL_CAN0_HOLD_POS)		/** CAN0 hold bit */
#define DBG_CTL_I2C0_HOLD_POS								0x0FUL		/** I2C0 hold bit */
#define DBG_CTL_I2C0_HOLD_MSK								(0x01UL << DBG_CTL_I2C0_HOLD_POS)		/** I2C0 hold bit */
#define DBG_CTL_I2C1_HOLD_POS								0x10UL		/** I2C1 hold bit */
#define DBG_CTL_I2C1_HOLD_MSK								(0x01UL << DBG_CTL_I2C1_HOLD_POS)		/** I2C1 hold bit */
#define DBG_CTL_TIMER4_HOLD_POS								0x12UL		/** TIMER4_HOLD */
#define DBG_CTL_TIMER4_HOLD_MSK								(0x01UL << DBG_CTL_TIMER4_HOLD_POS)		/** TIMER4_HOLD */
#define DBG_CTL_TIMER5_HOLD_POS								0x13UL		/** TIMER 5 hold bit */
#define DBG_CTL_TIMER5_HOLD_MSK								(0x01UL << DBG_CTL_TIMER5_HOLD_POS)		/** TIMER 5 hold bit */
#define DBG_CTL_TIMER6_HOLD_POS								0x14UL		/** TIMER 6 hold bit */
#define DBG_CTL_TIMER6_HOLD_MSK								(0x01UL << DBG_CTL_TIMER6_HOLD_POS)		/** TIMER 6 hold bit */
#define DBG_CTL_CAN1_HOLD_POS								0x15UL		/** CAN1 hold bit */
#define DBG_CTL_CAN1_HOLD_MSK								(0x01UL << DBG_CTL_CAN1_HOLD_POS)		/** CAN1 hold bit */
#define DMA_INTF_GIF0_POS								0x00UL		/** Global interrupt flag of channel 0 */
#define DMA_INTF_GIF0_MSK								(0x01UL << DMA_INTF_GIF0_POS)		/** Global interrupt flag of channel 0 */
#define DMA_INTF_FTFIF0_POS								0x01UL		/** Full Transfer finish flag of channe 0 */
#define DMA_INTF_FTFIF0_MSK								(0x01UL << DMA_INTF_FTFIF0_POS)		/** Full Transfer finish flag of channe 0 */
#define DMA_INTF_HTFIF0_POS								0x02UL		/** Half transfer finish flag of channel 0 */
#define DMA_INTF_HTFIF0_MSK								(0x01UL << DMA_INTF_HTFIF0_POS)		/** Half transfer finish flag of channel 0 */
#define DMA_INTF_ERRIF0_POS								0x03UL		/** Error flag of channel 0 */
#define DMA_INTF_ERRIF0_MSK								(0x01UL << DMA_INTF_ERRIF0_POS)		/** Error flag of channel 0 */
#define DMA_INTF_GIF1_POS								0x04UL		/** Global interrupt flag of channel 1 */
#define DMA_INTF_GIF1_MSK								(0x01UL << DMA_INTF_GIF1_POS)		/** Global interrupt flag of channel 1 */
#define DMA_INTF_FTFIF1_POS								0x05UL		/** Full Transfer finish flag of channe 1 */
#define DMA_INTF_FTFIF1_MSK								(0x01UL << DMA_INTF_FTFIF1_POS)		/** Full Transfer finish flag of channe 1 */
#define DMA_INTF_HTFIF1_POS								0x06UL		/** Half transfer finish flag of channel 1 */
#define DMA_INTF_HTFIF1_MSK								(0x01UL << DMA_INTF_HTFIF1_POS)		/** Half transfer finish flag of channel 1 */
#define DMA_INTF_ERRIF1_POS								0x07UL		/** Error flag of channel 1 */
#define DMA_INTF_ERRIF1_MSK								(0x01UL << DMA_INTF_ERRIF1_POS)		/** Error flag of channel 1 */
#define DMA_INTF_GIF2_POS								0x08UL		/** Global interrupt flag of channel 2 */
#define DMA_INTF_GIF2_MSK								(0x01UL << DMA_INTF_GIF2_POS)		/** Global interrupt flag of channel 2 */
#define DMA_INTF_FTFIF2_POS								0x09UL		/** Full Transfer finish flag of channe 2 */
#define DMA_INTF_FTFIF2_MSK								(0x01UL << DMA_INTF_FTFIF2_POS)		/** Full Transfer finish flag of channe 2 */
#define DMA_INTF_HTFIF2_POS								0x0AUL		/** Half transfer finish flag of channel 2 */
#define DMA_INTF_HTFIF2_MSK								(0x01UL << DMA_INTF_HTFIF2_POS)		/** Half transfer finish flag of channel 2 */
#define DMA_INTF_ERRIF2_POS								0x0BUL		/** Error flag of channel 2 */
#define DMA_INTF_ERRIF2_MSK								(0x01UL << DMA_INTF_ERRIF2_POS)		/** Error flag of channel 2 */
#define DMA_INTF_GIF3_POS								0x0CUL		/** Global interrupt flag of channel 3 */
#define DMA_INTF_GIF3_MSK								(0x01UL << DMA_INTF_GIF3_POS)		/** Global interrupt flag of channel 3 */
#define DMA_INTF_FTFIF3_POS								0x0DUL		/** Full Transfer finish flag of channe 3 */
#define DMA_INTF_FTFIF3_MSK								(0x01UL << DMA_INTF_FTFIF3_POS)		/** Full Transfer finish flag of channe 3 */
#define DMA_INTF_HTFIF3_POS								0x0EUL		/** Half transfer finish flag of channel 3 */
#define DMA_INTF_HTFIF3_MSK								(0x01UL << DMA_INTF_HTFIF3_POS)		/** Half transfer finish flag of channel 3 */
#define DMA_INTF_ERRIF3_POS								0x0FUL		/** Error flag of channel 3 */
#define DMA_INTF_ERRIF3_MSK								(0x01UL << DMA_INTF_ERRIF3_POS)		/** Error flag of channel 3 */
#define DMA_INTF_GIF4_POS								0x10UL		/** Global interrupt flag of channel 4 */
#define DMA_INTF_GIF4_MSK								(0x01UL << DMA_INTF_GIF4_POS)		/** Global interrupt flag of channel 4 */
#define DMA_INTF_FTFIF4_POS								0x11UL		/** Full Transfer finish flag of channe 4 */
#define DMA_INTF_FTFIF4_MSK								(0x01UL << DMA_INTF_FTFIF4_POS)		/** Full Transfer finish flag of channe 4 */
#define DMA_INTF_HTFIF4_POS								0x12UL		/** Half transfer finish flag of channel 4 */
#define DMA_INTF_HTFIF4_MSK								(0x01UL << DMA_INTF_HTFIF4_POS)		/** Half transfer finish flag of channel 4 */
#define DMA_INTF_ERRIF4_POS								0x13UL		/** Error flag of channel 4 */
#define DMA_INTF_ERRIF4_MSK								(0x01UL << DMA_INTF_ERRIF4_POS)		/** Error flag of channel 4 */
#define DMA_INTF_GIF5_POS								0x14UL		/** Global interrupt flag of channel 5 */
#define DMA_INTF_GIF5_MSK								(0x01UL << DMA_INTF_GIF5_POS)		/** Global interrupt flag of channel 5 */
#define DMA_INTF_FTFIF5_POS								0x15UL		/** Full Transfer finish flag of channe 5 */
#define DMA_INTF_FTFIF5_MSK								(0x01UL << DMA_INTF_FTFIF5_POS)		/** Full Transfer finish flag of channe 5 */
#define DMA_INTF_HTFIF5_POS								0x16UL		/** Half transfer finish flag of channel 5 */
#define DMA_INTF_HTFIF5_MSK								(0x01UL << DMA_INTF_HTFIF5_POS)		/** Half transfer finish flag of channel 5 */
#define DMA_INTF_ERRIF5_POS								0x17UL		/** Error flag of channel 5 */
#define DMA_INTF_ERRIF5_MSK								(0x01UL << DMA_INTF_ERRIF5_POS)		/** Error flag of channel 5 */
#define DMA_INTF_GIF6_POS								0x18UL		/** Global interrupt flag of channel 6 */
#define DMA_INTF_GIF6_MSK								(0x01UL << DMA_INTF_GIF6_POS)		/** Global interrupt flag of channel 6 */
#define DMA_INTF_FTFIF6_POS								0x19UL		/** Full Transfer finish flag of channe 6 */
#define DMA_INTF_FTFIF6_MSK								(0x01UL << DMA_INTF_FTFIF6_POS)		/** Full Transfer finish flag of channe 6 */
#define DMA_INTF_HTFIF6_POS								0x1AUL		/** Half transfer finish flag of channel 6 */
#define DMA_INTF_HTFIF6_MSK								(0x01UL << DMA_INTF_HTFIF6_POS)		/** Half transfer finish flag of channel 6 */
#define DMA_INTF_ERRIF6_POS								0x1BUL		/** Error flag of channel 6 */
#define DMA_INTF_ERRIF6_MSK								(0x01UL << DMA_INTF_ERRIF6_POS)		/** Error flag of channel 6 */
#define DMA_INTC_GIFC0_POS								0x00UL		/** Clear global interrupt flag of channel 0 */
#define DMA_INTC_GIFC0_MSK								(0x01UL << DMA_INTC_GIFC0_POS)		/** Clear global interrupt flag of channel 0 */
#define DMA_INTC_FTFIFC0_POS								0x01UL		/** Clear bit for full transfer finish flag of channel 0 */
#define DMA_INTC_FTFIFC0_MSK								(0x01UL << DMA_INTC_FTFIFC0_POS)		/** Clear bit for full transfer finish flag of channel 0 */
#define DMA_INTC_HTFIFC0_POS								0x02UL		/** Clear bit for half transfer finish flag of channel 0 */
#define DMA_INTC_HTFIFC0_MSK								(0x01UL << DMA_INTC_HTFIFC0_POS)		/** Clear bit for half transfer finish flag of channel 0 */
#define DMA_INTC_ERRIFC0_POS								0x03UL		/** Clear bit for error flag of channel 0 */
#define DMA_INTC_ERRIFC0_MSK								(0x01UL << DMA_INTC_ERRIFC0_POS)		/** Clear bit for error flag of channel 0 */
#define DMA_INTC_GIFC1_POS								0x04UL		/** Clear global interrupt flag of channel 1 */
#define DMA_INTC_GIFC1_MSK								(0x01UL << DMA_INTC_GIFC1_POS)		/** Clear global interrupt flag of channel 1 */
#define DMA_INTC_FTFIFC1_POS								0x05UL		/** Clear bit for full transfer finish flag of channel 1 */
#define DMA_INTC_FTFIFC1_MSK								(0x01UL << DMA_INTC_FTFIFC1_POS)		/** Clear bit for full transfer finish flag of channel 1 */
#define DMA_INTC_HTFIFC1_POS								0x06UL		/** Clear bit for half transfer finish flag of channel 1 */
#define DMA_INTC_HTFIFC1_MSK								(0x01UL << DMA_INTC_HTFIFC1_POS)		/** Clear bit for half transfer finish flag of channel 1 */
#define DMA_INTC_ERRIFC1_POS								0x07UL		/** Clear bit for error flag of channel 1 */
#define DMA_INTC_ERRIFC1_MSK								(0x01UL << DMA_INTC_ERRIFC1_POS)		/** Clear bit for error flag of channel 1 */
#define DMA_INTC_GIFC2_POS								0x08UL		/** Clear global interrupt flag of channel 2 */
#define DMA_INTC_GIFC2_MSK								(0x01UL << DMA_INTC_GIFC2_POS)		/** Clear global interrupt flag of channel 2 */
#define DMA_INTC_FTFIFC2_POS								0x09UL		/** Clear bit for full transfer finish flag of channel 2 */
#define DMA_INTC_FTFIFC2_MSK								(0x01UL << DMA_INTC_FTFIFC2_POS)		/** Clear bit for full transfer finish flag of channel 2 */
#define DMA_INTC_HTFIFC2_POS								0x0AUL		/** Clear bit for half transfer finish flag of channel 2 */
#define DMA_INTC_HTFIFC2_MSK								(0x01UL << DMA_INTC_HTFIFC2_POS)		/** Clear bit for half transfer finish flag of channel 2 */
#define DMA_INTC_ERRIFC2_POS								0x0BUL		/** Clear bit for error flag of channel 2 */
#define DMA_INTC_ERRIFC2_MSK								(0x01UL << DMA_INTC_ERRIFC2_POS)		/** Clear bit for error flag of channel 2 */
#define DMA_INTC_GIFC3_POS								0x0CUL		/** Clear global interrupt flag of channel 3 */
#define DMA_INTC_GIFC3_MSK								(0x01UL << DMA_INTC_GIFC3_POS)		/** Clear global interrupt flag of channel 3 */
#define DMA_INTC_FTFIFC3_POS								0x0DUL		/** Clear bit for full transfer finish flag of channel 3 */
#define DMA_INTC_FTFIFC3_MSK								(0x01UL << DMA_INTC_FTFIFC3_POS)		/** Clear bit for full transfer finish flag of channel 3 */
#define DMA_INTC_HTFIFC3_POS								0x0EUL		/** Clear bit for half transfer finish flag of channel 3 */
#define DMA_INTC_HTFIFC3_MSK								(0x01UL << DMA_INTC_HTFIFC3_POS)		/** Clear bit for half transfer finish flag of channel 3 */
#define DMA_INTC_ERRIFC3_POS								0x0FUL		/** Clear bit for error flag of channel 3 */
#define DMA_INTC_ERRIFC3_MSK								(0x01UL << DMA_INTC_ERRIFC3_POS)		/** Clear bit for error flag of channel 3 */
#define DMA_INTC_GIFC4_POS								0x10UL		/** Clear global interrupt flag of channel 4 */
#define DMA_INTC_GIFC4_MSK								(0x01UL << DMA_INTC_GIFC4_POS)		/** Clear global interrupt flag of channel 4 */
#define DMA_INTC_FTFIFC4_POS								0x11UL		/** Clear bit for full transfer finish flag of channel 4 */
#define DMA_INTC_FTFIFC4_MSK								(0x01UL << DMA_INTC_FTFIFC4_POS)		/** Clear bit for full transfer finish flag of channel 4 */
#define DMA_INTC_HTFIFC4_POS								0x12UL		/** Clear bit for half transfer finish flag of channel 4 */
#define DMA_INTC_HTFIFC4_MSK								(0x01UL << DMA_INTC_HTFIFC4_POS)		/** Clear bit for half transfer finish flag of channel 4 */
#define DMA_INTC_ERRIFC4_POS								0x13UL		/** Clear bit for error flag of channel 4 */
#define DMA_INTC_ERRIFC4_MSK								(0x01UL << DMA_INTC_ERRIFC4_POS)		/** Clear bit for error flag of channel 4 */
#define DMA_INTC_GIFC5_POS								0x14UL		/** Clear global interrupt flag of channel 5 */
#define DMA_INTC_GIFC5_MSK								(0x01UL << DMA_INTC_GIFC5_POS)		/** Clear global interrupt flag of channel 5 */
#define DMA_INTC_FTFIFC5_POS								0x15UL		/** Clear bit for full transfer finish flag of channel 5 */
#define DMA_INTC_FTFIFC5_MSK								(0x01UL << DMA_INTC_FTFIFC5_POS)		/** Clear bit for full transfer finish flag of channel 5 */
#define DMA_INTC_HTFIFC5_POS								0x16UL		/** Clear bit for half transfer finish flag of channel 5 */
#define DMA_INTC_HTFIFC5_MSK								(0x01UL << DMA_INTC_HTFIFC5_POS)		/** Clear bit for half transfer finish flag of channel 5 */
#define DMA_INTC_ERRIFC5_POS								0x17UL		/** Clear bit for error flag of channel 5 */
#define DMA_INTC_ERRIFC5_MSK								(0x01UL << DMA_INTC_ERRIFC5_POS)		/** Clear bit for error flag of channel 5 */
#define DMA_INTC_GIFC6_POS								0x18UL		/** Clear global interrupt flag of channel 6 */
#define DMA_INTC_GIFC6_MSK								(0x01UL << DMA_INTC_GIFC6_POS)		/** Clear global interrupt flag of channel 6 */
#define DMA_INTC_FTFIFC6_POS								0x19UL		/** Clear bit for full transfer finish flag of channel 6 */
#define DMA_INTC_FTFIFC6_MSK								(0x01UL << DMA_INTC_FTFIFC6_POS)		/** Clear bit for full transfer finish flag of channel 6 */
#define DMA_INTC_HTFIFC6_POS								0x1AUL		/** Clear bit for half transfer finish flag of channel 6 */
#define DMA_INTC_HTFIFC6_MSK								(0x01UL << DMA_INTC_HTFIFC6_POS)		/** Clear bit for half transfer finish flag of channel 6 */
#define DMA_INTC_ERRIFC6_POS								0x1BUL		/** Clear bit for error flag of channel 6 */
#define DMA_INTC_ERRIFC6_MSK								(0x01UL << DMA_INTC_ERRIFC6_POS)		/** Clear bit for error flag of channel 6 */
#define DMA_CH0CTL_CHEN_POS								0x00UL		/** Channel enable */
#define DMA_CH0CTL_CHEN_MSK								(0x01UL << DMA_CH0CTL_CHEN_POS)		/** Channel enable */
#define DMA_CH0CTL_FTFIE_POS								0x01UL		/** Enable bit for channel full transfer finish interrupt */
#define DMA_CH0CTL_FTFIE_MSK								(0x01UL << DMA_CH0CTL_FTFIE_POS)		/** Enable bit for channel full transfer finish interrupt */
#define DMA_CH0CTL_HTFIE_POS								0x02UL		/** Enable bit for channel half transfer finish interrupt */
#define DMA_CH0CTL_HTFIE_MSK								(0x01UL << DMA_CH0CTL_HTFIE_POS)		/** Enable bit for channel half transfer finish interrupt */
#define DMA_CH0CTL_ERRIE_POS								0x03UL		/** Enable bit for channel error interrupt */
#define DMA_CH0CTL_ERRIE_MSK								(0x01UL << DMA_CH0CTL_ERRIE_POS)		/** Enable bit for channel error interrupt */
#define DMA_CH0CTL_DIR_POS								0x04UL		/** Transfer direction */
#define DMA_CH0CTL_DIR_MSK								(0x01UL << DMA_CH0CTL_DIR_POS)		/** Transfer direction */
#define DMA_CH0CTL_CMEN_POS								0x05UL		/** Circular mode enable */
#define DMA_CH0CTL_CMEN_MSK								(0x01UL << DMA_CH0CTL_CMEN_POS)		/** Circular mode enable */
#define DMA_CH0CTL_PNAGA_POS								0x06UL		/** Next address generation algorithm of peripheral */
#define DMA_CH0CTL_PNAGA_MSK								(0x01UL << DMA_CH0CTL_PNAGA_POS)		/** Next address generation algorithm of peripheral */
#define DMA_CH0CTL_MNAGA_POS								0x07UL		/** Next address generation algorithm of memory */
#define DMA_CH0CTL_MNAGA_MSK								(0x01UL << DMA_CH0CTL_MNAGA_POS)		/** Next address generation algorithm of memory */
#define DMA_CH0CTL_PWIDTH_POS								0x08UL		/** Transfer data size of peripheral */
#define DMA_CH0CTL_PWIDTH_MSK								(0x03UL << DMA_CH0CTL_PWIDTH_POS)		/** Transfer data size of peripheral */
#define DMA_CH0CTL_MWIDTH_POS								0x0AUL		/** Transfer data size of memory */
#define DMA_CH0CTL_MWIDTH_MSK								(0x03UL << DMA_CH0CTL_MWIDTH_POS)		/** Transfer data size of memory */
#define DMA_CH0CTL_PRIO_POS								0x0CUL		/** Priority level */
#define DMA_CH0CTL_PRIO_MSK								(0x03UL << DMA_CH0CTL_PRIO_POS)		/** Priority level */
#define DMA_CH0CTL_M2M_POS								0x0EUL		/** Memory to Memory Mode */
#define DMA_CH0CTL_M2M_MSK								(0x01UL << DMA_CH0CTL_M2M_POS)		/** Memory to Memory Mode */
#define DMA_CH0CNT_CNT_POS								0x00UL		/** Transfer counter */
#define DMA_CH0CNT_CNT_MSK								(0xFFFFUL << DMA_CH0CNT_CNT_POS)		/** Transfer counter */
#define DMA_CH0PADDR_PADDR_POS								0x00UL		/** Peripheral base address */
#define DMA_CH0PADDR_PADDR_MSK								(0xFFFFFFFFUL << DMA_CH0PADDR_PADDR_POS)		/** Peripheral base address */
#define DMA_CH0MADDR_MADDR_POS								0x00UL		/** Memory base address */
#define DMA_CH0MADDR_MADDR_MSK								(0xFFFFFFFFUL << DMA_CH0MADDR_MADDR_POS)		/** Memory base address */
#define DMA_CH1CTL_CHEN_POS								0x00UL		/** Channel enable */
#define DMA_CH1CTL_CHEN_MSK								(0x01UL << DMA_CH1CTL_CHEN_POS)		/** Channel enable */
#define DMA_CH1CTL_FTFIE_POS								0x01UL		/** Enable bit for channel full transfer finish interrupt */
#define DMA_CH1CTL_FTFIE_MSK								(0x01UL << DMA_CH1CTL_FTFIE_POS)		/** Enable bit for channel full transfer finish interrupt */
#define DMA_CH1CTL_HTFIE_POS								0x02UL		/** Enable bit for channel half transfer finish interrupt */
#define DMA_CH1CTL_HTFIE_MSK								(0x01UL << DMA_CH1CTL_HTFIE_POS)		/** Enable bit for channel half transfer finish interrupt */
#define DMA_CH1CTL_ERRIE_POS								0x03UL		/** Enable bit for channel error interrupt */
#define DMA_CH1CTL_ERRIE_MSK								(0x01UL << DMA_CH1CTL_ERRIE_POS)		/** Enable bit for channel error interrupt */
#define DMA_CH1CTL_DIR_POS								0x04UL		/** Transfer direction */
#define DMA_CH1CTL_DIR_MSK								(0x01UL << DMA_CH1CTL_DIR_POS)		/** Transfer direction */
#define DMA_CH1CTL_CMEN_POS								0x05UL		/** Circular mode enable */
#define DMA_CH1CTL_CMEN_MSK								(0x01UL << DMA_CH1CTL_CMEN_POS)		/** Circular mode enable */
#define DMA_CH1CTL_PNAGA_POS								0x06UL		/** Next address generation algorithm of peripheral */
#define DMA_CH1CTL_PNAGA_MSK								(0x01UL << DMA_CH1CTL_PNAGA_POS)		/** Next address generation algorithm of peripheral */
#define DMA_CH1CTL_MNAGA_POS								0x07UL		/** Next address generation algorithm of memory */
#define DMA_CH1CTL_MNAGA_MSK								(0x01UL << DMA_CH1CTL_MNAGA_POS)		/** Next address generation algorithm of memory */
#define DMA_CH1CTL_PWIDTH_POS								0x08UL		/** Transfer data size of peripheral */
#define DMA_CH1CTL_PWIDTH_MSK								(0x03UL << DMA_CH1CTL_PWIDTH_POS)		/** Transfer data size of peripheral */
#define DMA_CH1CTL_MWIDTH_POS								0x0AUL		/** Transfer data size of memory */
#define DMA_CH1CTL_MWIDTH_MSK								(0x03UL << DMA_CH1CTL_MWIDTH_POS)		/** Transfer data size of memory */
#define DMA_CH1CTL_PRIO_POS								0x0CUL		/** Priority level */
#define DMA_CH1CTL_PRIO_MSK								(0x03UL << DMA_CH1CTL_PRIO_POS)		/** Priority level */
#define DMA_CH1CTL_M2M_POS								0x0EUL		/** Memory to Memory Mode */
#define DMA_CH1CTL_M2M_MSK								(0x01UL << DMA_CH1CTL_M2M_POS)		/** Memory to Memory Mode */
#define DMA_CH1CNT_CNT_POS								0x00UL		/** Transfer counter */
#define DMA_CH1CNT_CNT_MSK								(0xFFFFUL << DMA_CH1CNT_CNT_POS)		/** Transfer counter */
#define DMA_CH1PADDR_PADDR_POS								0x00UL		/** Peripheral base address */
#define DMA_CH1PADDR_PADDR_MSK								(0xFFFFFFFFUL << DMA_CH1PADDR_PADDR_POS)		/** Peripheral base address */
#define DMA_CH1MADDR_MADDR_POS								0x00UL		/** Memory base address */
#define DMA_CH1MADDR_MADDR_MSK								(0xFFFFFFFFUL << DMA_CH1MADDR_MADDR_POS)		/** Memory base address */
#define DMA_CH2CTL_CHEN_POS								0x00UL		/** Channel enable */
#define DMA_CH2CTL_CHEN_MSK								(0x01UL << DMA_CH2CTL_CHEN_POS)		/** Channel enable */
#define DMA_CH2CTL_FTFIE_POS								0x01UL		/** Enable bit for channel full transfer finish interrupt */
#define DMA_CH2CTL_FTFIE_MSK								(0x01UL << DMA_CH2CTL_FTFIE_POS)		/** Enable bit for channel full transfer finish interrupt */
#define DMA_CH2CTL_HTFIE_POS								0x02UL		/** Enable bit for channel half transfer finish interrupt */
#define DMA_CH2CTL_HTFIE_MSK								(0x01UL << DMA_CH2CTL_HTFIE_POS)		/** Enable bit for channel half transfer finish interrupt */
#define DMA_CH2CTL_ERRIE_POS								0x03UL		/** Enable bit for channel error interrupt */
#define DMA_CH2CTL_ERRIE_MSK								(0x01UL << DMA_CH2CTL_ERRIE_POS)		/** Enable bit for channel error interrupt */
#define DMA_CH2CTL_DIR_POS								0x04UL		/** Transfer direction */
#define DMA_CH2CTL_DIR_MSK								(0x01UL << DMA_CH2CTL_DIR_POS)		/** Transfer direction */
#define DMA_CH2CTL_CMEN_POS								0x05UL		/** Circular mode enable */
#define DMA_CH2CTL_CMEN_MSK								(0x01UL << DMA_CH2CTL_CMEN_POS)		/** Circular mode enable */
#define DMA_CH2CTL_PNAGA_POS								0x06UL		/** Next address generation algorithm of peripheral */
#define DMA_CH2CTL_PNAGA_MSK								(0x01UL << DMA_CH2CTL_PNAGA_POS)		/** Next address generation algorithm of peripheral */
#define DMA_CH2CTL_MNAGA_POS								0x07UL		/** Next address generation algorithm of memory */
#define DMA_CH2CTL_MNAGA_MSK								(0x01UL << DMA_CH2CTL_MNAGA_POS)		/** Next address generation algorithm of memory */
#define DMA_CH2CTL_PWIDTH_POS								0x08UL		/** Transfer data size of peripheral */
#define DMA_CH2CTL_PWIDTH_MSK								(0x03UL << DMA_CH2CTL_PWIDTH_POS)		/** Transfer data size of peripheral */
#define DMA_CH2CTL_MWIDTH_POS								0x0AUL		/** Transfer data size of memory */
#define DMA_CH2CTL_MWIDTH_MSK								(0x03UL << DMA_CH2CTL_MWIDTH_POS)		/** Transfer data size of memory */
#define DMA_CH2CTL_PRIO_POS								0x0CUL		/** Priority level */
#define DMA_CH2CTL_PRIO_MSK								(0x03UL << DMA_CH2CTL_PRIO_POS)		/** Priority level */
#define DMA_CH2CTL_M2M_POS								0x0EUL		/** Memory to Memory Mode */
#define DMA_CH2CTL_M2M_MSK								(0x01UL << DMA_CH2CTL_M2M_POS)		/** Memory to Memory Mode */
#define DMA_CH2CNT_CNT_POS								0x00UL		/** Transfer counter */
#define DMA_CH2CNT_CNT_MSK								(0xFFFFUL << DMA_CH2CNT_CNT_POS)		/** Transfer counter */
#define DMA_CH2PADDR_PADDR_POS								0x00UL		/** Peripheral base address */
#define DMA_CH2PADDR_PADDR_MSK								(0xFFFFFFFFUL << DMA_CH2PADDR_PADDR_POS)		/** Peripheral base address */
#define DMA_CH2MADDR_MADDR_POS								0x00UL		/** Memory base address */
#define DMA_CH2MADDR_MADDR_MSK								(0xFFFFFFFFUL << DMA_CH2MADDR_MADDR_POS)		/** Memory base address */
#define DMA_CH3CTL_CHEN_POS								0x00UL		/** Channel enable */
#define DMA_CH3CTL_CHEN_MSK								(0x01UL << DMA_CH3CTL_CHEN_POS)		/** Channel enable */
#define DMA_CH3CTL_FTFIE_POS								0x01UL		/** Enable bit for channel full transfer finish interrupt */
#define DMA_CH3CTL_FTFIE_MSK								(0x01UL << DMA_CH3CTL_FTFIE_POS)		/** Enable bit for channel full transfer finish interrupt */
#define DMA_CH3CTL_HTFIE_POS								0x02UL		/** Enable bit for channel half transfer finish interrupt */
#define DMA_CH3CTL_HTFIE_MSK								(0x01UL << DMA_CH3CTL_HTFIE_POS)		/** Enable bit for channel half transfer finish interrupt */
#define DMA_CH3CTL_ERRIE_POS								0x03UL		/** Enable bit for channel error interrupt */
#define DMA_CH3CTL_ERRIE_MSK								(0x01UL << DMA_CH3CTL_ERRIE_POS)		/** Enable bit for channel error interrupt */
#define DMA_CH3CTL_DIR_POS								0x04UL		/** Transfer direction */
#define DMA_CH3CTL_DIR_MSK								(0x01UL << DMA_CH3CTL_DIR_POS)		/** Transfer direction */
#define DMA_CH3CTL_CMEN_POS								0x05UL		/** Circular mode enable */
#define DMA_CH3CTL_CMEN_MSK								(0x01UL << DMA_CH3CTL_CMEN_POS)		/** Circular mode enable */
#define DMA_CH3CTL_PNAGA_POS								0x06UL		/** Next address generation algorithm of peripheral */
#define DMA_CH3CTL_PNAGA_MSK								(0x01UL << DMA_CH3CTL_PNAGA_POS)		/** Next address generation algorithm of peripheral */
#define DMA_CH3CTL_MNAGA_POS								0x07UL		/** Next address generation algorithm of memory */
#define DMA_CH3CTL_MNAGA_MSK								(0x01UL << DMA_CH3CTL_MNAGA_POS)		/** Next address generation algorithm of memory */
#define DMA_CH3CTL_PWIDTH_POS								0x08UL		/** Transfer data size of peripheral */
#define DMA_CH3CTL_PWIDTH_MSK								(0x03UL << DMA_CH3CTL_PWIDTH_POS)		/** Transfer data size of peripheral */
#define DMA_CH3CTL_MWIDTH_POS								0x0AUL		/** Transfer data size of memory */
#define DMA_CH3CTL_MWIDTH_MSK								(0x03UL << DMA_CH3CTL_MWIDTH_POS)		/** Transfer data size of memory */
#define DMA_CH3CTL_PRIO_POS								0x0CUL		/** Priority level */
#define DMA_CH3CTL_PRIO_MSK								(0x03UL << DMA_CH3CTL_PRIO_POS)		/** Priority level */
#define DMA_CH3CTL_M2M_POS								0x0EUL		/** Memory to Memory Mode */
#define DMA_CH3CTL_M2M_MSK								(0x01UL << DMA_CH3CTL_M2M_POS)		/** Memory to Memory Mode */
#define DMA_CH3CNT_CNT_POS								0x00UL		/** Transfer counter */
#define DMA_CH3CNT_CNT_MSK								(0xFFFFUL << DMA_CH3CNT_CNT_POS)		/** Transfer counter */
#define DMA_CH3PADDR_PADDR_POS								0x00UL		/** Peripheral base address */
#define DMA_CH3PADDR_PADDR_MSK								(0xFFFFFFFFUL << DMA_CH3PADDR_PADDR_POS)		/** Peripheral base address */
#define DMA_CH3MADDR_MADDR_POS								0x00UL		/** Memory base address */
#define DMA_CH3MADDR_MADDR_MSK								(0xFFFFFFFFUL << DMA_CH3MADDR_MADDR_POS)		/** Memory base address */
#define DMA_CH4CTL_CHEN_POS								0x00UL		/** Channel enable */
#define DMA_CH4CTL_CHEN_MSK								(0x01UL << DMA_CH4CTL_CHEN_POS)		/** Channel enable */
#define DMA_CH4CTL_FTFIE_POS								0x01UL		/** Enable bit for channel full transfer finish interrupt */
#define DMA_CH4CTL_FTFIE_MSK								(0x01UL << DMA_CH4CTL_FTFIE_POS)		/** Enable bit for channel full transfer finish interrupt */
#define DMA_CH4CTL_HTFIE_POS								0x02UL		/** Enable bit for channel half transfer finish interrupt */
#define DMA_CH4CTL_HTFIE_MSK								(0x01UL << DMA_CH4CTL_HTFIE_POS)		/** Enable bit for channel half transfer finish interrupt */
#define DMA_CH4CTL_ERRIE_POS								0x03UL		/** Enable bit for channel error interrupt */
#define DMA_CH4CTL_ERRIE_MSK								(0x01UL << DMA_CH4CTL_ERRIE_POS)		/** Enable bit for channel error interrupt */
#define DMA_CH4CTL_DIR_POS								0x04UL		/** Transfer direction */
#define DMA_CH4CTL_DIR_MSK								(0x01UL << DMA_CH4CTL_DIR_POS)		/** Transfer direction */
#define DMA_CH4CTL_CMEN_POS								0x05UL		/** Circular mode enable */
#define DMA_CH4CTL_CMEN_MSK								(0x01UL << DMA_CH4CTL_CMEN_POS)		/** Circular mode enable */
#define DMA_CH4CTL_PNAGA_POS								0x06UL		/** Next address generation algorithm of peripheral */
#define DMA_CH4CTL_PNAGA_MSK								(0x01UL << DMA_CH4CTL_PNAGA_POS)		/** Next address generation algorithm of peripheral */
#define DMA_CH4CTL_MNAGA_POS								0x07UL		/** Next address generation algorithm of memory */
#define DMA_CH4CTL_MNAGA_MSK								(0x01UL << DMA_CH4CTL_MNAGA_POS)		/** Next address generation algorithm of memory */
#define DMA_CH4CTL_PWIDTH_POS								0x08UL		/** Transfer data size of peripheral */
#define DMA_CH4CTL_PWIDTH_MSK								(0x03UL << DMA_CH4CTL_PWIDTH_POS)		/** Transfer data size of peripheral */
#define DMA_CH4CTL_MWIDTH_POS								0x0AUL		/** Transfer data size of memory */
#define DMA_CH4CTL_MWIDTH_MSK								(0x03UL << DMA_CH4CTL_MWIDTH_POS)		/** Transfer data size of memory */
#define DMA_CH4CTL_PRIO_POS								0x0CUL		/** Priority level */
#define DMA_CH4CTL_PRIO_MSK								(0x03UL << DMA_CH4CTL_PRIO_POS)		/** Priority level */
#define DMA_CH4CTL_M2M_POS								0x0EUL		/** Memory to Memory Mode */
#define DMA_CH4CTL_M2M_MSK								(0x01UL << DMA_CH4CTL_M2M_POS)		/** Memory to Memory Mode */
#define DMA_CH4CNT_CNT_POS								0x00UL		/** Transfer counter */
#define DMA_CH4CNT_CNT_MSK								(0xFFFFUL << DMA_CH4CNT_CNT_POS)		/** Transfer counter */
#define DMA_CH4PADDR_PADDR_POS								0x00UL		/** Peripheral base address */
#define DMA_CH4PADDR_PADDR_MSK								(0xFFFFFFFFUL << DMA_CH4PADDR_PADDR_POS)		/** Peripheral base address */
#define DMA_CH4MADDR_MADDR_POS								0x00UL		/** Memory base address */
#define DMA_CH4MADDR_MADDR_MSK								(0xFFFFFFFFUL << DMA_CH4MADDR_MADDR_POS)		/** Memory base address */
#define DMA_CH5CTL_CHEN_POS								0x00UL		/** Channel enable */
#define DMA_CH5CTL_CHEN_MSK								(0x01UL << DMA_CH5CTL_CHEN_POS)		/** Channel enable */
#define DMA_CH5CTL_FTFIE_POS								0x01UL		/** Enable bit for channel full transfer finish interrupt */
#define DMA_CH5CTL_FTFIE_MSK								(0x01UL << DMA_CH5CTL_FTFIE_POS)		/** Enable bit for channel full transfer finish interrupt */
#define DMA_CH5CTL_HTFIE_POS								0x02UL		/** Enable bit for channel half transfer finish interrupt */
#define DMA_CH5CTL_HTFIE_MSK								(0x01UL << DMA_CH5CTL_HTFIE_POS)		/** Enable bit for channel half transfer finish interrupt */
#define DMA_CH5CTL_ERRIE_POS								0x03UL		/** Enable bit for channel error interrupt */
#define DMA_CH5CTL_ERRIE_MSK								(0x01UL << DMA_CH5CTL_ERRIE_POS)		/** Enable bit for channel error interrupt */
#define DMA_CH5CTL_DIR_POS								0x04UL		/** Transfer direction */
#define DMA_CH5CTL_DIR_MSK								(0x01UL << DMA_CH5CTL_DIR_POS)		/** Transfer direction */
#define DMA_CH5CTL_CMEN_POS								0x05UL		/** Circular mode enable */
#define DMA_CH5CTL_CMEN_MSK								(0x01UL << DMA_CH5CTL_CMEN_POS)		/** Circular mode enable */
#define DMA_CH5CTL_PNAGA_POS								0x06UL		/** Next address generation algorithm of peripheral */
#define DMA_CH5CTL_PNAGA_MSK								(0x01UL << DMA_CH5CTL_PNAGA_POS)		/** Next address generation algorithm of peripheral */
#define DMA_CH5CTL_MNAGA_POS								0x07UL		/** Next address generation algorithm of memory */
#define DMA_CH5CTL_MNAGA_MSK								(0x01UL << DMA_CH5CTL_MNAGA_POS)		/** Next address generation algorithm of memory */
#define DMA_CH5CTL_PWIDTH_POS								0x08UL		/** Transfer data size of peripheral */
#define DMA_CH5CTL_PWIDTH_MSK								(0x03UL << DMA_CH5CTL_PWIDTH_POS)		/** Transfer data size of peripheral */
#define DMA_CH5CTL_MWIDTH_POS								0x0AUL		/** Transfer data size of memory */
#define DMA_CH5CTL_MWIDTH_MSK								(0x03UL << DMA_CH5CTL_MWIDTH_POS)		/** Transfer data size of memory */
#define DMA_CH5CTL_PRIO_POS								0x0CUL		/** Priority level */
#define DMA_CH5CTL_PRIO_MSK								(0x03UL << DMA_CH5CTL_PRIO_POS)		/** Priority level */
#define DMA_CH5CTL_M2M_POS								0x0EUL		/** Memory to Memory Mode */
#define DMA_CH5CTL_M2M_MSK								(0x01UL << DMA_CH5CTL_M2M_POS)		/** Memory to Memory Mode */
#define DMA_CH5CNT_CNT_POS								0x00UL		/** Transfer counter */
#define DMA_CH5CNT_CNT_MSK								(0xFFFFUL << DMA_CH5CNT_CNT_POS)		/** Transfer counter */
#define DMA_CH5PADDR_PADDR_POS								0x00UL		/** Peripheral base address */
#define DMA_CH5PADDR_PADDR_MSK								(0xFFFFFFFFUL << DMA_CH5PADDR_PADDR_POS)		/** Peripheral base address */
#define DMA_CH5MADDR_MADDR_POS								0x00UL		/** Memory base address */
#define DMA_CH5MADDR_MADDR_MSK								(0xFFFFFFFFUL << DMA_CH5MADDR_MADDR_POS)		/** Memory base address */
#define DMA_CH6CTL_CHEN_POS								0x00UL		/** Channel enable */
#define DMA_CH6CTL_CHEN_MSK								(0x01UL << DMA_CH6CTL_CHEN_POS)		/** Channel enable */
#define DMA_CH6CTL_FTFIE_POS								0x01UL		/** Enable bit for channel full transfer finish interrupt */
#define DMA_CH6CTL_FTFIE_MSK								(0x01UL << DMA_CH6CTL_FTFIE_POS)		/** Enable bit for channel full transfer finish interrupt */
#define DMA_CH6CTL_HTFIE_POS								0x02UL		/** Enable bit for channel half transfer finish interrupt */
#define DMA_CH6CTL_HTFIE_MSK								(0x01UL << DMA_CH6CTL_HTFIE_POS)		/** Enable bit for channel half transfer finish interrupt */
#define DMA_CH6CTL_ERRIE_POS								0x03UL		/** Enable bit for channel error interrupt */
#define DMA_CH6CTL_ERRIE_MSK								(0x01UL << DMA_CH6CTL_ERRIE_POS)		/** Enable bit for channel error interrupt */
#define DMA_CH6CTL_DIR_POS								0x04UL		/** Transfer direction */
#define DMA_CH6CTL_DIR_MSK								(0x01UL << DMA_CH6CTL_DIR_POS)		/** Transfer direction */
#define DMA_CH6CTL_CMEN_POS								0x05UL		/** Circular mode enable */
#define DMA_CH6CTL_CMEN_MSK								(0x01UL << DMA_CH6CTL_CMEN_POS)		/** Circular mode enable */
#define DMA_CH6CTL_PNAGA_POS								0x06UL		/** Next address generation algorithm of peripheral */
#define DMA_CH6CTL_PNAGA_MSK								(0x01UL << DMA_CH6CTL_PNAGA_POS)		/** Next address generation algorithm of peripheral */
#define DMA_CH6CTL_MNAGA_POS								0x07UL		/** Next address generation algorithm of memory */
#define DMA_CH6CTL_MNAGA_MSK								(0x01UL << DMA_CH6CTL_MNAGA_POS)		/** Next address generation algorithm of memory */
#define DMA_CH6CTL_PWIDTH_POS								0x08UL		/** Transfer data size of peripheral */
#define DMA_CH6CTL_PWIDTH_MSK								(0x03UL << DMA_CH6CTL_PWIDTH_POS)		/** Transfer data size of peripheral */
#define DMA_CH6CTL_MWIDTH_POS								0x0AUL		/** Transfer data size of memory */
#define DMA_CH6CTL_MWIDTH_MSK								(0x03UL << DMA_CH6CTL_MWIDTH_POS)		/** Transfer data size of memory */
#define DMA_CH6CTL_PRIO_POS								0x0CUL		/** Priority level */
#define DMA_CH6CTL_PRIO_MSK								(0x03UL << DMA_CH6CTL_PRIO_POS)		/** Priority level */
#define DMA_CH6CTL_M2M_POS								0x0EUL		/** Memory to Memory Mode */
#define DMA_CH6CTL_M2M_MSK								(0x01UL << DMA_CH6CTL_M2M_POS)		/** Memory to Memory Mode */
#define DMA_CH6CNT_CNT_POS								0x00UL		/** Transfer counter */
#define DMA_CH6CNT_CNT_MSK								(0xFFFFUL << DMA_CH6CNT_CNT_POS)		/** Transfer counter */
#define DMA_CH6PADDR_PADDR_POS								0x00UL		/** Peripheral base address */
#define DMA_CH6PADDR_PADDR_MSK								(0xFFFFFFFFUL << DMA_CH6PADDR_PADDR_POS)		/** Peripheral base address */
#define DMA_CH6MADDR_MADDR_POS								0x00UL		/** Memory base address */
#define DMA_CH6MADDR_MADDR_MSK								(0xFFFFFFFFUL << DMA_CH6MADDR_MADDR_POS)		/** Memory base address */
#define EXMC_SNCTL0_ASYNCWAIT_POS								0x0FUL		/** Asynchronous wait */
#define EXMC_SNCTL0_ASYNCWAIT_MSK								(0x01UL << EXMC_SNCTL0_ASYNCWAIT_POS)		/** Asynchronous wait */
#define EXMC_SNCTL0_NRWTEN_POS								0x0DUL		/** NWAIT signal enable */
#define EXMC_SNCTL0_NRWTEN_MSK								(0x01UL << EXMC_SNCTL0_NRWTEN_POS)		/** NWAIT signal enable */
#define EXMC_SNCTL0_WREN_POS								0x0CUL		/** Write enable */
#define EXMC_SNCTL0_WREN_MSK								(0x01UL << EXMC_SNCTL0_WREN_POS)		/** Write enable */
#define EXMC_SNCTL0_NRWTPOL_POS								0x09UL		/** NWAIT signal polarity */
#define EXMC_SNCTL0_NRWTPOL_MSK								(0x01UL << EXMC_SNCTL0_NRWTPOL_POS)		/** NWAIT signal polarity */
#define EXMC_SNCTL0_NREN_POS								0x06UL		/** NOR Flash access enable */
#define EXMC_SNCTL0_NREN_MSK								(0x01UL << EXMC_SNCTL0_NREN_POS)		/** NOR Flash access enable */
#define EXMC_SNCTL0_NRW_POS								0x04UL		/** NOR bank memory data bus width */
#define EXMC_SNCTL0_NRW_MSK								(0x03UL << EXMC_SNCTL0_NRW_POS)		/** NOR bank memory data bus width */
#define EXMC_SNCTL0_NRTP_POS								0x02UL		/** NOR bank memory type */
#define EXMC_SNCTL0_NRTP_MSK								(0x03UL << EXMC_SNCTL0_NRTP_POS)		/** NOR bank memory type */
#define EXMC_SNCTL0_NRMUX_POS								0x01UL		/** NOR bank memory address/data multiplexing */
#define EXMC_SNCTL0_NRMUX_MSK								(0x01UL << EXMC_SNCTL0_NRMUX_POS)		/** NOR bank memory address/data multiplexing */
#define EXMC_SNCTL0_NRBKEN_POS								0x00UL		/** NOR bank enable */
#define EXMC_SNCTL0_NRBKEN_MSK								(0x01UL << EXMC_SNCTL0_NRBKEN_POS)		/** NOR bank enable */
#define EXMC_SNTCFG0_BUSLAT_POS								0x10UL		/** Bus latency */
#define EXMC_SNTCFG0_BUSLAT_MSK								(0x0FUL << EXMC_SNTCFG0_BUSLAT_POS)		/** Bus latency */
#define EXMC_SNTCFG0_DSET_POS								0x08UL		/** Data setup time */
#define EXMC_SNTCFG0_DSET_MSK								(0xFFUL << EXMC_SNTCFG0_DSET_POS)		/** Data setup time */
#define EXMC_SNTCFG0_AHLD_POS								0x04UL		/** Address hold time */
#define EXMC_SNTCFG0_AHLD_MSK								(0x0FUL << EXMC_SNTCFG0_AHLD_POS)		/** Address hold time */
#define EXMC_SNTCFG0_ASET_POS								0x00UL		/** Address setup time */
#define EXMC_SNTCFG0_ASET_MSK								(0x0FUL << EXMC_SNTCFG0_ASET_POS)		/** Address setup time */
#define EXMC_SNCTL1_ASYNCWAIT_POS								0x0FUL		/** Asynchronous wait */
#define EXMC_SNCTL1_ASYNCWAIT_MSK								(0x01UL << EXMC_SNCTL1_ASYNCWAIT_POS)		/** Asynchronous wait */
#define EXMC_SNCTL1_NRWTEN_POS								0x0DUL		/** NWAIT signal enable */
#define EXMC_SNCTL1_NRWTEN_MSK								(0x01UL << EXMC_SNCTL1_NRWTEN_POS)		/** NWAIT signal enable */
#define EXMC_SNCTL1_WREN_POS								0x0CUL		/** Write enable */
#define EXMC_SNCTL1_WREN_MSK								(0x01UL << EXMC_SNCTL1_WREN_POS)		/** Write enable */
#define EXMC_SNCTL1_NRWTPOL_POS								0x09UL		/** NWAIT signal polarity */
#define EXMC_SNCTL1_NRWTPOL_MSK								(0x01UL << EXMC_SNCTL1_NRWTPOL_POS)		/** NWAIT signal polarity */
#define EXMC_SNCTL1_NREN_POS								0x06UL		/** NOR Flash access enable */
#define EXMC_SNCTL1_NREN_MSK								(0x01UL << EXMC_SNCTL1_NREN_POS)		/** NOR Flash access enable */
#define EXMC_SNCTL1_NRW_POS								0x04UL		/** NOR bank memory data bus width */
#define EXMC_SNCTL1_NRW_MSK								(0x03UL << EXMC_SNCTL1_NRW_POS)		/** NOR bank memory data bus width */
#define EXMC_SNCTL1_NRTP_POS								0x02UL		/** NOR bank memory type */
#define EXMC_SNCTL1_NRTP_MSK								(0x03UL << EXMC_SNCTL1_NRTP_POS)		/** NOR bank memory type */
#define EXMC_SNCTL1_NRMUX_POS								0x01UL		/** NOR bank memory address/data multiplexing */
#define EXMC_SNCTL1_NRMUX_MSK								(0x01UL << EXMC_SNCTL1_NRMUX_POS)		/** NOR bank memory address/data multiplexing */
#define EXMC_SNCTL1_NRBKEN_POS								0x00UL		/** NOR bank enable */
#define EXMC_SNCTL1_NRBKEN_MSK								(0x01UL << EXMC_SNCTL1_NRBKEN_POS)		/** NOR bank enable */
#define EXTI_INTEN_INTEN0_POS								0x00UL		/** Enable Interrupt on line 0 */
#define EXTI_INTEN_INTEN0_MSK								(0x01UL << EXTI_INTEN_INTEN0_POS)		/** Enable Interrupt on line 0 */
#define EXTI_INTEN_INTEN1_POS								0x01UL		/** Enable Interrupt on line 1 */
#define EXTI_INTEN_INTEN1_MSK								(0x01UL << EXTI_INTEN_INTEN1_POS)		/** Enable Interrupt on line 1 */
#define EXTI_INTEN_INTEN2_POS								0x02UL		/** Enable Interrupt on line 2 */
#define EXTI_INTEN_INTEN2_MSK								(0x01UL << EXTI_INTEN_INTEN2_POS)		/** Enable Interrupt on line 2 */
#define EXTI_INTEN_INTEN3_POS								0x03UL		/** Enable Interrupt on line 3 */
#define EXTI_INTEN_INTEN3_MSK								(0x01UL << EXTI_INTEN_INTEN3_POS)		/** Enable Interrupt on line 3 */
#define EXTI_INTEN_INTEN4_POS								0x04UL		/** Enable Interrupt on line 4 */
#define EXTI_INTEN_INTEN4_MSK								(0x01UL << EXTI_INTEN_INTEN4_POS)		/** Enable Interrupt on line 4 */
#define EXTI_INTEN_INTEN5_POS								0x05UL		/** Enable Interrupt on line 5 */
#define EXTI_INTEN_INTEN5_MSK								(0x01UL << EXTI_INTEN_INTEN5_POS)		/** Enable Interrupt on line 5 */
#define EXTI_INTEN_INTEN6_POS								0x06UL		/** Enable Interrupt on line 6 */
#define EXTI_INTEN_INTEN6_MSK								(0x01UL << EXTI_INTEN_INTEN6_POS)		/** Enable Interrupt on line 6 */
#define EXTI_INTEN_INTEN7_POS								0x07UL		/** Enable Interrupt on line 7 */
#define EXTI_INTEN_INTEN7_MSK								(0x01UL << EXTI_INTEN_INTEN7_POS)		/** Enable Interrupt on line 7 */
#define EXTI_INTEN_INTEN8_POS								0x08UL		/** Enable Interrupt on line 8 */
#define EXTI_INTEN_INTEN8_MSK								(0x01UL << EXTI_INTEN_INTEN8_POS)		/** Enable Interrupt on line 8 */
#define EXTI_INTEN_INTEN9_POS								0x09UL		/** Enable Interrupt on line 9 */
#define EXTI_INTEN_INTEN9_MSK								(0x01UL << EXTI_INTEN_INTEN9_POS)		/** Enable Interrupt on line 9 */
#define EXTI_INTEN_INTEN10_POS								0x0AUL		/** Enable Interrupt on line 10 */
#define EXTI_INTEN_INTEN10_MSK								(0x01UL << EXTI_INTEN_INTEN10_POS)		/** Enable Interrupt on line 10 */
#define EXTI_INTEN_INTEN11_POS								0x0BUL		/** Enable Interrupt on line 11 */
#define EXTI_INTEN_INTEN11_MSK								(0x01UL << EXTI_INTEN_INTEN11_POS)		/** Enable Interrupt on line 11 */
#define EXTI_INTEN_INTEN12_POS								0x0CUL		/** Enable Interrupt on line 12 */
#define EXTI_INTEN_INTEN12_MSK								(0x01UL << EXTI_INTEN_INTEN12_POS)		/** Enable Interrupt on line 12 */
#define EXTI_INTEN_INTEN13_POS								0x0DUL		/** Enable Interrupt on line 13 */
#define EXTI_INTEN_INTEN13_MSK								(0x01UL << EXTI_INTEN_INTEN13_POS)		/** Enable Interrupt on line 13 */
#define EXTI_INTEN_INTEN14_POS								0x0EUL		/** Enable Interrupt on line 14 */
#define EXTI_INTEN_INTEN14_MSK								(0x01UL << EXTI_INTEN_INTEN14_POS)		/** Enable Interrupt on line 14 */
#define EXTI_INTEN_INTEN15_POS								0x0FUL		/** Enable Interrupt on line 15 */
#define EXTI_INTEN_INTEN15_MSK								(0x01UL << EXTI_INTEN_INTEN15_POS)		/** Enable Interrupt on line 15 */
#define EXTI_INTEN_INTEN16_POS								0x10UL		/** Enable Interrupt on line 16 */
#define EXTI_INTEN_INTEN16_MSK								(0x01UL << EXTI_INTEN_INTEN16_POS)		/** Enable Interrupt on line 16 */
#define EXTI_INTEN_INTEN17_POS								0x11UL		/** Enable Interrupt on line 17 */
#define EXTI_INTEN_INTEN17_MSK								(0x01UL << EXTI_INTEN_INTEN17_POS)		/** Enable Interrupt on line 17 */
#define EXTI_INTEN_INTEN18_POS								0x12UL		/** Enable Interrupt on line 18 */
#define EXTI_INTEN_INTEN18_MSK								(0x01UL << EXTI_INTEN_INTEN18_POS)		/** Enable Interrupt on line 18 */
#define EXTI_EVEN_EVEN0_POS								0x00UL		/** Enable Event on line 0 */
#define EXTI_EVEN_EVEN0_MSK								(0x01UL << EXTI_EVEN_EVEN0_POS)		/** Enable Event on line 0 */
#define EXTI_EVEN_EVEN1_POS								0x01UL		/** Enable Event on line 1 */
#define EXTI_EVEN_EVEN1_MSK								(0x01UL << EXTI_EVEN_EVEN1_POS)		/** Enable Event on line 1 */
#define EXTI_EVEN_EVEN2_POS								0x02UL		/** Enable Event on line 2 */
#define EXTI_EVEN_EVEN2_MSK								(0x01UL << EXTI_EVEN_EVEN2_POS)		/** Enable Event on line 2 */
#define EXTI_EVEN_EVEN3_POS								0x03UL		/** Enable Event on line 3 */
#define EXTI_EVEN_EVEN3_MSK								(0x01UL << EXTI_EVEN_EVEN3_POS)		/** Enable Event on line 3 */
#define EXTI_EVEN_EVEN4_POS								0x04UL		/** Enable Event on line 4 */
#define EXTI_EVEN_EVEN4_MSK								(0x01UL << EXTI_EVEN_EVEN4_POS)		/** Enable Event on line 4 */
#define EXTI_EVEN_EVEN5_POS								0x05UL		/** Enable Event on line 5 */
#define EXTI_EVEN_EVEN5_MSK								(0x01UL << EXTI_EVEN_EVEN5_POS)		/** Enable Event on line 5 */
#define EXTI_EVEN_EVEN6_POS								0x06UL		/** Enable Event on line 6 */
#define EXTI_EVEN_EVEN6_MSK								(0x01UL << EXTI_EVEN_EVEN6_POS)		/** Enable Event on line 6 */
#define EXTI_EVEN_EVEN7_POS								0x07UL		/** Enable Event on line 7 */
#define EXTI_EVEN_EVEN7_MSK								(0x01UL << EXTI_EVEN_EVEN7_POS)		/** Enable Event on line 7 */
#define EXTI_EVEN_EVEN8_POS								0x08UL		/** Enable Event on line 8 */
#define EXTI_EVEN_EVEN8_MSK								(0x01UL << EXTI_EVEN_EVEN8_POS)		/** Enable Event on line 8 */
#define EXTI_EVEN_EVEN9_POS								0x09UL		/** Enable Event on line 9 */
#define EXTI_EVEN_EVEN9_MSK								(0x01UL << EXTI_EVEN_EVEN9_POS)		/** Enable Event on line 9 */
#define EXTI_EVEN_EVEN10_POS								0x0AUL		/** Enable Event on line 10 */
#define EXTI_EVEN_EVEN10_MSK								(0x01UL << EXTI_EVEN_EVEN10_POS)		/** Enable Event on line 10 */
#define EXTI_EVEN_EVEN11_POS								0x0BUL		/** Enable Event on line 11 */
#define EXTI_EVEN_EVEN11_MSK								(0x01UL << EXTI_EVEN_EVEN11_POS)		/** Enable Event on line 11 */
#define EXTI_EVEN_EVEN12_POS								0x0CUL		/** Enable Event on line 12 */
#define EXTI_EVEN_EVEN12_MSK								(0x01UL << EXTI_EVEN_EVEN12_POS)		/** Enable Event on line 12 */
#define EXTI_EVEN_EVEN13_POS								0x0DUL		/** Enable Event on line 13 */
#define EXTI_EVEN_EVEN13_MSK								(0x01UL << EXTI_EVEN_EVEN13_POS)		/** Enable Event on line 13 */
#define EXTI_EVEN_EVEN14_POS								0x0EUL		/** Enable Event on line 14 */
#define EXTI_EVEN_EVEN14_MSK								(0x01UL << EXTI_EVEN_EVEN14_POS)		/** Enable Event on line 14 */
#define EXTI_EVEN_EVEN15_POS								0x0FUL		/** Enable Event on line 15 */
#define EXTI_EVEN_EVEN15_MSK								(0x01UL << EXTI_EVEN_EVEN15_POS)		/** Enable Event on line 15 */
#define EXTI_EVEN_EVEN16_POS								0x10UL		/** Enable Event on line 16 */
#define EXTI_EVEN_EVEN16_MSK								(0x01UL << EXTI_EVEN_EVEN16_POS)		/** Enable Event on line 16 */
#define EXTI_EVEN_EVEN17_POS								0x11UL		/** Enable Event on line 17 */
#define EXTI_EVEN_EVEN17_MSK								(0x01UL << EXTI_EVEN_EVEN17_POS)		/** Enable Event on line 17 */
#define EXTI_EVEN_EVEN18_POS								0x12UL		/** Enable Event on line 18 */
#define EXTI_EVEN_EVEN18_MSK								(0x01UL << EXTI_EVEN_EVEN18_POS)		/** Enable Event on line 18 */
#define EXTI_RTEN_RTEN0_POS								0x00UL		/** Rising edge trigger enable of line 0 */
#define EXTI_RTEN_RTEN0_MSK								(0x01UL << EXTI_RTEN_RTEN0_POS)		/** Rising edge trigger enable of line 0 */
#define EXTI_RTEN_RTEN1_POS								0x01UL		/** Rising edge trigger enable of line 1 */
#define EXTI_RTEN_RTEN1_MSK								(0x01UL << EXTI_RTEN_RTEN1_POS)		/** Rising edge trigger enable of line 1 */
#define EXTI_RTEN_RTEN2_POS								0x02UL		/** Rising edge trigger enable of line 2 */
#define EXTI_RTEN_RTEN2_MSK								(0x01UL << EXTI_RTEN_RTEN2_POS)		/** Rising edge trigger enable of line 2 */
#define EXTI_RTEN_RTEN3_POS								0x03UL		/** Rising edge trigger enable of line 3 */
#define EXTI_RTEN_RTEN3_MSK								(0x01UL << EXTI_RTEN_RTEN3_POS)		/** Rising edge trigger enable of line 3 */
#define EXTI_RTEN_RTEN4_POS								0x04UL		/** Rising edge trigger enable of line 4 */
#define EXTI_RTEN_RTEN4_MSK								(0x01UL << EXTI_RTEN_RTEN4_POS)		/** Rising edge trigger enable of line 4 */
#define EXTI_RTEN_RTEN5_POS								0x05UL		/** Rising edge trigger enable of line 5 */
#define EXTI_RTEN_RTEN5_MSK								(0x01UL << EXTI_RTEN_RTEN5_POS)		/** Rising edge trigger enable of line 5 */
#define EXTI_RTEN_RTEN6_POS								0x06UL		/** Rising edge trigger enable of line 6 */
#define EXTI_RTEN_RTEN6_MSK								(0x01UL << EXTI_RTEN_RTEN6_POS)		/** Rising edge trigger enable of line 6 */
#define EXTI_RTEN_RTEN7_POS								0x07UL		/** Rising edge trigger enable of line 7 */
#define EXTI_RTEN_RTEN7_MSK								(0x01UL << EXTI_RTEN_RTEN7_POS)		/** Rising edge trigger enable of line 7 */
#define EXTI_RTEN_RTEN8_POS								0x08UL		/** Rising edge trigger enable of line 8 */
#define EXTI_RTEN_RTEN8_MSK								(0x01UL << EXTI_RTEN_RTEN8_POS)		/** Rising edge trigger enable of line 8 */
#define EXTI_RTEN_RTEN9_POS								0x09UL		/** Rising edge trigger enable of line 9 */
#define EXTI_RTEN_RTEN9_MSK								(0x01UL << EXTI_RTEN_RTEN9_POS)		/** Rising edge trigger enable of line 9 */
#define EXTI_RTEN_RTEN10_POS								0x0AUL		/** Rising edge trigger enable of line 10 */
#define EXTI_RTEN_RTEN10_MSK								(0x01UL << EXTI_RTEN_RTEN10_POS)		/** Rising edge trigger enable of line 10 */
#define EXTI_RTEN_RTEN11_POS								0x0BUL		/** Rising edge trigger enable of line 11 */
#define EXTI_RTEN_RTEN11_MSK								(0x01UL << EXTI_RTEN_RTEN11_POS)		/** Rising edge trigger enable of line 11 */
#define EXTI_RTEN_RTEN12_POS								0x0CUL		/** Rising edge trigger enable of line 12 */
#define EXTI_RTEN_RTEN12_MSK								(0x01UL << EXTI_RTEN_RTEN12_POS)		/** Rising edge trigger enable of line 12 */
#define EXTI_RTEN_RTEN13_POS								0x0DUL		/** Rising edge trigger enable of line 13 */
#define EXTI_RTEN_RTEN13_MSK								(0x01UL << EXTI_RTEN_RTEN13_POS)		/** Rising edge trigger enable of line 13 */
#define EXTI_RTEN_RTEN14_POS								0x0EUL		/** Rising edge trigger enable of line 14 */
#define EXTI_RTEN_RTEN14_MSK								(0x01UL << EXTI_RTEN_RTEN14_POS)		/** Rising edge trigger enable of line 14 */
#define EXTI_RTEN_RTEN15_POS								0x0FUL		/** Rising edge trigger enable of line 15 */
#define EXTI_RTEN_RTEN15_MSK								(0x01UL << EXTI_RTEN_RTEN15_POS)		/** Rising edge trigger enable of line 15 */
#define EXTI_RTEN_RTEN16_POS								0x10UL		/** Rising edge trigger enable of line 16 */
#define EXTI_RTEN_RTEN16_MSK								(0x01UL << EXTI_RTEN_RTEN16_POS)		/** Rising edge trigger enable of line 16 */
#define EXTI_RTEN_RTEN17_POS								0x11UL		/** Rising edge trigger enable of line 17 */
#define EXTI_RTEN_RTEN17_MSK								(0x01UL << EXTI_RTEN_RTEN17_POS)		/** Rising edge trigger enable of line 17 */
#define EXTI_RTEN_RTEN18_POS								0x12UL		/** Rising edge trigger enable of line 18 */
#define EXTI_RTEN_RTEN18_MSK								(0x01UL << EXTI_RTEN_RTEN18_POS)		/** Rising edge trigger enable of line 18 */
#define EXTI_FTEN_FTEN0_POS								0x00UL		/** Falling edge trigger enable of line 0 */
#define EXTI_FTEN_FTEN0_MSK								(0x01UL << EXTI_FTEN_FTEN0_POS)		/** Falling edge trigger enable of line 0 */
#define EXTI_FTEN_FTEN1_POS								0x01UL		/** Falling edge trigger enable of line 1 */
#define EXTI_FTEN_FTEN1_MSK								(0x01UL << EXTI_FTEN_FTEN1_POS)		/** Falling edge trigger enable of line 1 */
#define EXTI_FTEN_FTEN2_POS								0x02UL		/** Falling edge trigger enable of line 2 */
#define EXTI_FTEN_FTEN2_MSK								(0x01UL << EXTI_FTEN_FTEN2_POS)		/** Falling edge trigger enable of line 2 */
#define EXTI_FTEN_FTEN3_POS								0x03UL		/** Falling edge trigger enable of line 3 */
#define EXTI_FTEN_FTEN3_MSK								(0x01UL << EXTI_FTEN_FTEN3_POS)		/** Falling edge trigger enable of line 3 */
#define EXTI_FTEN_FTEN4_POS								0x04UL		/** Falling edge trigger enable of line 4 */
#define EXTI_FTEN_FTEN4_MSK								(0x01UL << EXTI_FTEN_FTEN4_POS)		/** Falling edge trigger enable of line 4 */
#define EXTI_FTEN_FTEN5_POS								0x05UL		/** Falling edge trigger enable of line 5 */
#define EXTI_FTEN_FTEN5_MSK								(0x01UL << EXTI_FTEN_FTEN5_POS)		/** Falling edge trigger enable of line 5 */
#define EXTI_FTEN_FTEN6_POS								0x06UL		/** Falling edge trigger enable of line 6 */
#define EXTI_FTEN_FTEN6_MSK								(0x01UL << EXTI_FTEN_FTEN6_POS)		/** Falling edge trigger enable of line 6 */
#define EXTI_FTEN_FTEN7_POS								0x07UL		/** Falling edge trigger enable of line 7 */
#define EXTI_FTEN_FTEN7_MSK								(0x01UL << EXTI_FTEN_FTEN7_POS)		/** Falling edge trigger enable of line 7 */
#define EXTI_FTEN_FTEN8_POS								0x08UL		/** Falling edge trigger enable of line 8 */
#define EXTI_FTEN_FTEN8_MSK								(0x01UL << EXTI_FTEN_FTEN8_POS)		/** Falling edge trigger enable of line 8 */
#define EXTI_FTEN_FTEN9_POS								0x09UL		/** Falling edge trigger enable of line 9 */
#define EXTI_FTEN_FTEN9_MSK								(0x01UL << EXTI_FTEN_FTEN9_POS)		/** Falling edge trigger enable of line 9 */
#define EXTI_FTEN_FTEN10_POS								0x0AUL		/** Falling edge trigger enable of line 10 */
#define EXTI_FTEN_FTEN10_MSK								(0x01UL << EXTI_FTEN_FTEN10_POS)		/** Falling edge trigger enable of line 10 */
#define EXTI_FTEN_FTEN11_POS								0x0BUL		/** Falling edge trigger enable of line 11 */
#define EXTI_FTEN_FTEN11_MSK								(0x01UL << EXTI_FTEN_FTEN11_POS)		/** Falling edge trigger enable of line 11 */
#define EXTI_FTEN_FTEN12_POS								0x0CUL		/** Falling edge trigger enable of line 12 */
#define EXTI_FTEN_FTEN12_MSK								(0x01UL << EXTI_FTEN_FTEN12_POS)		/** Falling edge trigger enable of line 12 */
#define EXTI_FTEN_FTEN13_POS								0x0DUL		/** Falling edge trigger enable of line 13 */
#define EXTI_FTEN_FTEN13_MSK								(0x01UL << EXTI_FTEN_FTEN13_POS)		/** Falling edge trigger enable of line 13 */
#define EXTI_FTEN_FTEN14_POS								0x0EUL		/** Falling edge trigger enable of line 14 */
#define EXTI_FTEN_FTEN14_MSK								(0x01UL << EXTI_FTEN_FTEN14_POS)		/** Falling edge trigger enable of line 14 */
#define EXTI_FTEN_FTEN15_POS								0x0FUL		/** Falling edge trigger enable of line 15 */
#define EXTI_FTEN_FTEN15_MSK								(0x01UL << EXTI_FTEN_FTEN15_POS)		/** Falling edge trigger enable of line 15 */
#define EXTI_FTEN_FTEN16_POS								0x10UL		/** Falling edge trigger enable of line 16 */
#define EXTI_FTEN_FTEN16_MSK								(0x01UL << EXTI_FTEN_FTEN16_POS)		/** Falling edge trigger enable of line 16 */
#define EXTI_FTEN_FTEN17_POS								0x11UL		/** Falling edge trigger enable of line 17 */
#define EXTI_FTEN_FTEN17_MSK								(0x01UL << EXTI_FTEN_FTEN17_POS)		/** Falling edge trigger enable of line 17 */
#define EXTI_FTEN_FTEN18_POS								0x12UL		/** Falling edge trigger enable of line 18 */
#define EXTI_FTEN_FTEN18_MSK								(0x01UL << EXTI_FTEN_FTEN18_POS)		/** Falling edge trigger enable of line 18 */
#define EXTI_SWIEV_SWIEV0_POS								0x00UL		/** Interrupt/Event software trigger on line 0 */
#define EXTI_SWIEV_SWIEV0_MSK								(0x01UL << EXTI_SWIEV_SWIEV0_POS)		/** Interrupt/Event software trigger on line 0 */
#define EXTI_SWIEV_SWIEV1_POS								0x01UL		/** Interrupt/Event software trigger on line 1 */
#define EXTI_SWIEV_SWIEV1_MSK								(0x01UL << EXTI_SWIEV_SWIEV1_POS)		/** Interrupt/Event software trigger on line 1 */
#define EXTI_SWIEV_SWIEV2_POS								0x02UL		/** Interrupt/Event software trigger on line 2 */
#define EXTI_SWIEV_SWIEV2_MSK								(0x01UL << EXTI_SWIEV_SWIEV2_POS)		/** Interrupt/Event software trigger on line 2 */
#define EXTI_SWIEV_SWIEV3_POS								0x03UL		/** Interrupt/Event software trigger on line 3 */
#define EXTI_SWIEV_SWIEV3_MSK								(0x01UL << EXTI_SWIEV_SWIEV3_POS)		/** Interrupt/Event software trigger on line 3 */
#define EXTI_SWIEV_SWIEV4_POS								0x04UL		/** Interrupt/Event software trigger on line 4 */
#define EXTI_SWIEV_SWIEV4_MSK								(0x01UL << EXTI_SWIEV_SWIEV4_POS)		/** Interrupt/Event software trigger on line 4 */
#define EXTI_SWIEV_SWIEV5_POS								0x05UL		/** Interrupt/Event software trigger on line 5 */
#define EXTI_SWIEV_SWIEV5_MSK								(0x01UL << EXTI_SWIEV_SWIEV5_POS)		/** Interrupt/Event software trigger on line 5 */
#define EXTI_SWIEV_SWIEV6_POS								0x06UL		/** Interrupt/Event software trigger on line 6 */
#define EXTI_SWIEV_SWIEV6_MSK								(0x01UL << EXTI_SWIEV_SWIEV6_POS)		/** Interrupt/Event software trigger on line 6 */
#define EXTI_SWIEV_SWIEV7_POS								0x07UL		/** Interrupt/Event software trigger on line 7 */
#define EXTI_SWIEV_SWIEV7_MSK								(0x01UL << EXTI_SWIEV_SWIEV7_POS)		/** Interrupt/Event software trigger on line 7 */
#define EXTI_SWIEV_SWIEV8_POS								0x08UL		/** Interrupt/Event software trigger on line 8 */
#define EXTI_SWIEV_SWIEV8_MSK								(0x01UL << EXTI_SWIEV_SWIEV8_POS)		/** Interrupt/Event software trigger on line 8 */
#define EXTI_SWIEV_SWIEV9_POS								0x09UL		/** Interrupt/Event software trigger on line 9 */
#define EXTI_SWIEV_SWIEV9_MSK								(0x01UL << EXTI_SWIEV_SWIEV9_POS)		/** Interrupt/Event software trigger on line 9 */
#define EXTI_SWIEV_SWIEV10_POS								0x0AUL		/** Interrupt/Event software trigger on line 10 */
#define EXTI_SWIEV_SWIEV10_MSK								(0x01UL << EXTI_SWIEV_SWIEV10_POS)		/** Interrupt/Event software trigger on line 10 */
#define EXTI_SWIEV_SWIEV11_POS								0x0BUL		/** Interrupt/Event software trigger on line 11 */
#define EXTI_SWIEV_SWIEV11_MSK								(0x01UL << EXTI_SWIEV_SWIEV11_POS)		/** Interrupt/Event software trigger on line 11 */
#define EXTI_SWIEV_SWIEV12_POS								0x0CUL		/** Interrupt/Event software trigger on line 12 */
#define EXTI_SWIEV_SWIEV12_MSK								(0x01UL << EXTI_SWIEV_SWIEV12_POS)		/** Interrupt/Event software trigger on line 12 */
#define EXTI_SWIEV_SWIEV13_POS								0x0DUL		/** Interrupt/Event software trigger on line 13 */
#define EXTI_SWIEV_SWIEV13_MSK								(0x01UL << EXTI_SWIEV_SWIEV13_POS)		/** Interrupt/Event software trigger on line 13 */
#define EXTI_SWIEV_SWIEV14_POS								0x0EUL		/** Interrupt/Event software trigger on line 14 */
#define EXTI_SWIEV_SWIEV14_MSK								(0x01UL << EXTI_SWIEV_SWIEV14_POS)		/** Interrupt/Event software trigger on line 14 */
#define EXTI_SWIEV_SWIEV15_POS								0x0FUL		/** Interrupt/Event software trigger on line 15 */
#define EXTI_SWIEV_SWIEV15_MSK								(0x01UL << EXTI_SWIEV_SWIEV15_POS)		/** Interrupt/Event software trigger on line 15 */
#define EXTI_SWIEV_SWIEV16_POS								0x10UL		/** Interrupt/Event software trigger on line 16 */
#define EXTI_SWIEV_SWIEV16_MSK								(0x01UL << EXTI_SWIEV_SWIEV16_POS)		/** Interrupt/Event software trigger on line 16 */
#define EXTI_SWIEV_SWIEV17_POS								0x11UL		/** Interrupt/Event software trigger on line 17 */
#define EXTI_SWIEV_SWIEV17_MSK								(0x01UL << EXTI_SWIEV_SWIEV17_POS)		/** Interrupt/Event software trigger on line 17 */
#define EXTI_SWIEV_SWIEV18_POS								0x12UL		/** Interrupt/Event software trigger on line 18 */
#define EXTI_SWIEV_SWIEV18_MSK								(0x01UL << EXTI_SWIEV_SWIEV18_POS)		/** Interrupt/Event software trigger on line 18 */
#define EXTI_PD_PD0_POS								0x00UL		/** Interrupt pending status of line 0 */
#define EXTI_PD_PD0_MSK								(0x01UL << EXTI_PD_PD0_POS)		/** Interrupt pending status of line 0 */
#define EXTI_PD_PD1_POS								0x01UL		/** Interrupt pending status of line 1 */
#define EXTI_PD_PD1_MSK								(0x01UL << EXTI_PD_PD1_POS)		/** Interrupt pending status of line 1 */
#define EXTI_PD_PD2_POS								0x02UL		/** Interrupt pending status of line 2 */
#define EXTI_PD_PD2_MSK								(0x01UL << EXTI_PD_PD2_POS)		/** Interrupt pending status of line 2 */
#define EXTI_PD_PD3_POS								0x03UL		/** Interrupt pending status of line 3 */
#define EXTI_PD_PD3_MSK								(0x01UL << EXTI_PD_PD3_POS)		/** Interrupt pending status of line 3 */
#define EXTI_PD_PD4_POS								0x04UL		/** Interrupt pending status of line 4 */
#define EXTI_PD_PD4_MSK								(0x01UL << EXTI_PD_PD4_POS)		/** Interrupt pending status of line 4 */
#define EXTI_PD_PD5_POS								0x05UL		/** Interrupt pending status of line 5 */
#define EXTI_PD_PD5_MSK								(0x01UL << EXTI_PD_PD5_POS)		/** Interrupt pending status of line 5 */
#define EXTI_PD_PD6_POS								0x06UL		/** Interrupt pending status of line 6 */
#define EXTI_PD_PD6_MSK								(0x01UL << EXTI_PD_PD6_POS)		/** Interrupt pending status of line 6 */
#define EXTI_PD_PD7_POS								0x07UL		/** Interrupt pending status of line 7 */
#define EXTI_PD_PD7_MSK								(0x01UL << EXTI_PD_PD7_POS)		/** Interrupt pending status of line 7 */
#define EXTI_PD_PD8_POS								0x08UL		/** Interrupt pending status of line 8 */
#define EXTI_PD_PD8_MSK								(0x01UL << EXTI_PD_PD8_POS)		/** Interrupt pending status of line 8 */
#define EXTI_PD_PD9_POS								0x09UL		/** Interrupt pending status of line 9 */
#define EXTI_PD_PD9_MSK								(0x01UL << EXTI_PD_PD9_POS)		/** Interrupt pending status of line 9 */
#define EXTI_PD_PD10_POS								0x0AUL		/** Interrupt pending status of line 10 */
#define EXTI_PD_PD10_MSK								(0x01UL << EXTI_PD_PD10_POS)		/** Interrupt pending status of line 10 */
#define EXTI_PD_PD11_POS								0x0BUL		/** Interrupt pending status of line 11 */
#define EXTI_PD_PD11_MSK								(0x01UL << EXTI_PD_PD11_POS)		/** Interrupt pending status of line 11 */
#define EXTI_PD_PD12_POS								0x0CUL		/** Interrupt pending status of line 12 */
#define EXTI_PD_PD12_MSK								(0x01UL << EXTI_PD_PD12_POS)		/** Interrupt pending status of line 12 */
#define EXTI_PD_PD13_POS								0x0DUL		/** Interrupt pending status of line 13 */
#define EXTI_PD_PD13_MSK								(0x01UL << EXTI_PD_PD13_POS)		/** Interrupt pending status of line 13 */
#define EXTI_PD_PD14_POS								0x0EUL		/** Interrupt pending status of line 14 */
#define EXTI_PD_PD14_MSK								(0x01UL << EXTI_PD_PD14_POS)		/** Interrupt pending status of line 14 */
#define EXTI_PD_PD15_POS								0x0FUL		/** Interrupt pending status of line 15 */
#define EXTI_PD_PD15_MSK								(0x01UL << EXTI_PD_PD15_POS)		/** Interrupt pending status of line 15 */
#define EXTI_PD_PD16_POS								0x10UL		/** Interrupt pending status of line 16 */
#define EXTI_PD_PD16_MSK								(0x01UL << EXTI_PD_PD16_POS)		/** Interrupt pending status of line 16 */
#define EXTI_PD_PD17_POS								0x11UL		/** Interrupt pending status of line 17 */
#define EXTI_PD_PD17_MSK								(0x01UL << EXTI_PD_PD17_POS)		/** Interrupt pending status of line 17 */
#define EXTI_PD_PD18_POS								0x12UL		/** Interrupt pending status of line 18 */
#define EXTI_PD_PD18_MSK								(0x01UL << EXTI_PD_PD18_POS)		/** Interrupt pending status of line 18 */
#define FMC_WS_WSCNT_POS								0x00UL		/** wait state counter register */
#define FMC_WS_WSCNT_MSK								(0x07UL << FMC_WS_WSCNT_POS)		/** wait state counter register */
#define FMC_KEY0_KEY_POS								0x00UL		/** FMC_CTL0 unlock key */
#define FMC_KEY0_KEY_MSK								(0xFFFFFFFFUL << FMC_KEY0_KEY_POS)		/** FMC_CTL0 unlock key */
#define FMC_OBKEY_OBKEY_POS								0x00UL		/** FMC_ CTL0 option byte operation unlock register */
#define FMC_OBKEY_OBKEY_MSK								(0xFFFFFFFFUL << FMC_OBKEY_OBKEY_POS)		/** FMC_ CTL0 option byte operation unlock register */
#define FMC_STAT0_ENDF_POS								0x05UL		/** End of operation flag bit */
#define FMC_STAT0_ENDF_MSK								(0x01UL << FMC_STAT0_ENDF_POS)		/** End of operation flag bit */
#define FMC_STAT0_WPERR_POS								0x04UL		/** Erase/Program protection error flag bit */
#define FMC_STAT0_WPERR_MSK								(0x01UL << FMC_STAT0_WPERR_POS)		/** Erase/Program protection error flag bit */
#define FMC_STAT0_PGERR_POS								0x02UL		/** Program error flag bit */
#define FMC_STAT0_PGERR_MSK								(0x01UL << FMC_STAT0_PGERR_POS)		/** Program error flag bit */
#define FMC_STAT0_BUSY_POS								0x00UL		/** The flash is busy bit */
#define FMC_STAT0_BUSY_MSK								(0x01UL << FMC_STAT0_BUSY_POS)		/** The flash is busy bit */
#define FMC_CTL0_ENDIE_POS								0x0CUL		/** End of operation interrupt enable bit */
#define FMC_CTL0_ENDIE_MSK								(0x01UL << FMC_CTL0_ENDIE_POS)		/** End of operation interrupt enable bit */
#define FMC_CTL0_ERRIE_POS								0x0AUL		/** Error interrupt enable bit */
#define FMC_CTL0_ERRIE_MSK								(0x01UL << FMC_CTL0_ERRIE_POS)		/** Error interrupt enable bit */
#define FMC_CTL0_OBWEN_POS								0x09UL		/** Option byte erase/program enable bit */
#define FMC_CTL0_OBWEN_MSK								(0x01UL << FMC_CTL0_OBWEN_POS)		/** Option byte erase/program enable bit */
#define FMC_CTL0_LK_POS								0x07UL		/** FMC_CTL0 lock bit */
#define FMC_CTL0_LK_MSK								(0x01UL << FMC_CTL0_LK_POS)		/** FMC_CTL0 lock bit */
#define FMC_CTL0_START_POS								0x06UL		/** Send erase command to FMC bit */
#define FMC_CTL0_START_MSK								(0x01UL << FMC_CTL0_START_POS)		/** Send erase command to FMC bit */
#define FMC_CTL0_OBER_POS								0x05UL		/** Option bytes erase command bit */
#define FMC_CTL0_OBER_MSK								(0x01UL << FMC_CTL0_OBER_POS)		/** Option bytes erase command bit */
#define FMC_CTL0_OBPG_POS								0x04UL		/** Option bytes program command bit */
#define FMC_CTL0_OBPG_MSK								(0x01UL << FMC_CTL0_OBPG_POS)		/** Option bytes program command bit */
#define FMC_CTL0_MER_POS								0x02UL		/** Main flash mass erase for bank0 command bit */
#define FMC_CTL0_MER_MSK								(0x01UL << FMC_CTL0_MER_POS)		/** Main flash mass erase for bank0 command bit */
#define FMC_CTL0_PER_POS								0x01UL		/** Main flash page erase for bank0 command bit */
#define FMC_CTL0_PER_MSK								(0x01UL << FMC_CTL0_PER_POS)		/** Main flash page erase for bank0 command bit */
#define FMC_CTL0_PG_POS								0x00UL		/** Main flash program for bank0 command bit */
#define FMC_CTL0_PG_MSK								(0x01UL << FMC_CTL0_PG_POS)		/** Main flash program for bank0 command bit */
#define FMC_ADDR0_ADDR_POS								0x00UL		/** Flash erase/program command address bits */
#define FMC_ADDR0_ADDR_MSK								(0xFFFFFFFFUL << FMC_ADDR0_ADDR_POS)		/** Flash erase/program command address bits */
#define FMC_OBSTAT_OBERR_POS								0x00UL		/** Option bytes read error bit */
#define FMC_OBSTAT_OBERR_MSK								(0x01UL << FMC_OBSTAT_OBERR_POS)		/** Option bytes read error bit */
#define FMC_OBSTAT_SPC_POS								0x01UL		/** Option bytes security protection code */
#define FMC_OBSTAT_SPC_MSK								(0x01UL << FMC_OBSTAT_SPC_POS)		/** Option bytes security protection code */
#define FMC_OBSTAT_USER_POS								0x02UL		/** Store USER of option bytes block after system reset */
#define FMC_OBSTAT_USER_MSK								(0xFFUL << FMC_OBSTAT_USER_POS)		/** Store USER of option bytes block after system reset */
#define FMC_OBSTAT_DATA_POS								0x0AUL		/** Store DATA[15:0] of option bytes block after system reset */
#define FMC_OBSTAT_DATA_MSK								(0xFFFFUL << FMC_OBSTAT_DATA_POS)		/** Store DATA[15:0] of option bytes block after system reset */
#define FMC_WP_WP_POS								0x00UL		/** Store WP[31:0] of option bytes block after system reset */
#define FMC_WP_WP_MSK								(0xFFFFFFFFUL << FMC_WP_WP_POS)		/** Store WP[31:0] of option bytes block after system reset */
#define FMC_PID_PID_POS								0x00UL		/** Product reserved ID code register */
#define FMC_PID_PID_MSK								(0xFFFFFFFFUL << FMC_PID_PID_POS)		/** Product reserved ID code register */
#define FWDGT_CTL_CMD_POS								0x00UL		/** Key value */
#define FWDGT_CTL_CMD_MSK								(0xFFFFUL << FWDGT_CTL_CMD_POS)		/** Key value */
#define FWDGT_PSC_PSC_POS								0x00UL		/** Free watchdog timer prescaler selection */
#define FWDGT_PSC_PSC_MSK								(0x07UL << FWDGT_PSC_PSC_POS)		/** Free watchdog timer prescaler selection */
#define FWDGT_RLD_RLD_POS								0x00UL		/** Free watchdog timer counter reload value */
#define FWDGT_RLD_RLD_MSK								(0xFFFUL << FWDGT_RLD_RLD_POS)		/** Free watchdog timer counter reload value */
#define FWDGT_STAT_PUD_POS								0x00UL		/** Free watchdog timer prescaler value update */
#define FWDGT_STAT_PUD_MSK								(0x01UL << FWDGT_STAT_PUD_POS)		/** Free watchdog timer prescaler value update */
#define FWDGT_STAT_RUD_POS								0x01UL		/** Free watchdog timer counter reload value update */
#define FWDGT_STAT_RUD_MSK								(0x01UL << FWDGT_STAT_RUD_POS)		/** Free watchdog timer counter reload value update */
#define GPIO_CTL0_CTL7_POS								0x1EUL		/** Port x configuration bits (x = 7) */
#define GPIO_CTL0_CTL7_MSK								(0x03UL << GPIO_CTL0_CTL7_POS)		/** Port x configuration bits (x = 7) */
#define GPIO_CTL0_MD7_POS								0x1CUL		/** Port x mode bits (x = 7) */
#define GPIO_CTL0_MD7_MSK								(0x03UL << GPIO_CTL0_MD7_POS)		/** Port x mode bits (x = 7) */
#define GPIO_CTL0_CTL6_POS								0x1AUL		/** Port x configuration bits (x = 6) */
#define GPIO_CTL0_CTL6_MSK								(0x03UL << GPIO_CTL0_CTL6_POS)		/** Port x configuration bits (x = 6) */
#define GPIO_CTL0_MD6_POS								0x18UL		/** Port x mode bits (x = 6) */
#define GPIO_CTL0_MD6_MSK								(0x03UL << GPIO_CTL0_MD6_POS)		/** Port x mode bits (x = 6) */
#define GPIO_CTL0_CTL5_POS								0x16UL		/** Port x configuration bits (x = 5) */
#define GPIO_CTL0_CTL5_MSK								(0x03UL << GPIO_CTL0_CTL5_POS)		/** Port x configuration bits (x = 5) */
#define GPIO_CTL0_MD5_POS								0x14UL		/** Port x mode bits (x = 5) */
#define GPIO_CTL0_MD5_MSK								(0x03UL << GPIO_CTL0_MD5_POS)		/** Port x mode bits (x = 5) */
#define GPIO_CTL0_CTL4_POS								0x12UL		/** Port x configuration bits (x = 4) */
#define GPIO_CTL0_CTL4_MSK								(0x03UL << GPIO_CTL0_CTL4_POS)		/** Port x configuration bits (x = 4) */
#define GPIO_CTL0_MD4_POS								0x10UL		/** Port x mode bits (x = 4) */
#define GPIO_CTL0_MD4_MSK								(0x03UL << GPIO_CTL0_MD4_POS)		/** Port x mode bits (x = 4) */
#define GPIO_CTL0_CTL3_POS								0x0EUL		/** Port x configuration bits (x = 3) */
#define GPIO_CTL0_CTL3_MSK								(0x03UL << GPIO_CTL0_CTL3_POS)		/** Port x configuration bits (x = 3) */
#define GPIO_CTL0_MD3_POS								0x0CUL		/** Port x mode bits (x = 3 ) */
#define GPIO_CTL0_MD3_MSK								(0x03UL << GPIO_CTL0_MD3_POS)		/** Port x mode bits (x = 3 ) */
#define GPIO_CTL0_CTL2_POS								0x0AUL		/** Port x configuration bits (x = 2) */
#define GPIO_CTL0_CTL2_MSK								(0x03UL << GPIO_CTL0_CTL2_POS)		/** Port x configuration bits (x = 2) */
#define GPIO_CTL0_MD2_POS								0x08UL		/** Port x mode bits (x = 2 ) */
#define GPIO_CTL0_MD2_MSK								(0x03UL << GPIO_CTL0_MD2_POS)		/** Port x mode bits (x = 2 ) */
#define GPIO_CTL0_CTL1_POS								0x06UL		/** Port x configuration bits (x = 1) */
#define GPIO_CTL0_CTL1_MSK								(0x03UL << GPIO_CTL0_CTL1_POS)		/** Port x configuration bits (x = 1) */
#define GPIO_CTL0_MD1_POS								0x04UL		/** Port x mode bits (x = 1) */
#define GPIO_CTL0_MD1_MSK								(0x03UL << GPIO_CTL0_MD1_POS)		/** Port x mode bits (x = 1) */
#define GPIO_CTL0_CTL0_POS								0x02UL		/** Port x configuration bits (x = 0) */
#define GPIO_CTL0_CTL0_MSK								(0x03UL << GPIO_CTL0_CTL0_POS)		/** Port x configuration bits (x = 0) */
#define GPIO_CTL0_MD0_POS								0x00UL		/** Port x mode bits (x = 0) */
#define GPIO_CTL0_MD0_MSK								(0x03UL << GPIO_CTL0_MD0_POS)		/** Port x mode bits (x = 0) */
#define GPIO_CTL1_CTL15_POS								0x1EUL		/** Port x configuration bits (x = 15) */
#define GPIO_CTL1_CTL15_MSK								(0x03UL << GPIO_CTL1_CTL15_POS)		/** Port x configuration bits (x = 15) */
#define GPIO_CTL1_MD15_POS								0x1CUL		/** Port x mode bits (x = 15) */
#define GPIO_CTL1_MD15_MSK								(0x03UL << GPIO_CTL1_MD15_POS)		/** Port x mode bits (x = 15) */
#define GPIO_CTL1_CTL14_POS								0x1AUL		/** Port x configuration bits (x = 14) */
#define GPIO_CTL1_CTL14_MSK								(0x03UL << GPIO_CTL1_CTL14_POS)		/** Port x configuration bits (x = 14) */
#define GPIO_CTL1_MD14_POS								0x18UL		/** Port x mode bits (x = 14) */
#define GPIO_CTL1_MD14_MSK								(0x03UL << GPIO_CTL1_MD14_POS)		/** Port x mode bits (x = 14) */
#define GPIO_CTL1_CTL13_POS								0x16UL		/** Port x configuration bits (x = 13) */
#define GPIO_CTL1_CTL13_MSK								(0x03UL << GPIO_CTL1_CTL13_POS)		/** Port x configuration bits (x = 13) */
#define GPIO_CTL1_MD13_POS								0x14UL		/** Port x mode bits (x = 13) */
#define GPIO_CTL1_MD13_MSK								(0x03UL << GPIO_CTL1_MD13_POS)		/** Port x mode bits (x = 13) */
#define GPIO_CTL1_CTL12_POS								0x12UL		/** Port x configuration bits (x = 12) */
#define GPIO_CTL1_CTL12_MSK								(0x03UL << GPIO_CTL1_CTL12_POS)		/** Port x configuration bits (x = 12) */
#define GPIO_CTL1_MD12_POS								0x10UL		/** Port x mode bits (x = 12) */
#define GPIO_CTL1_MD12_MSK								(0x03UL << GPIO_CTL1_MD12_POS)		/** Port x mode bits (x = 12) */
#define GPIO_CTL1_CTL11_POS								0x0EUL		/** Port x configuration bits (x = 11) */
#define GPIO_CTL1_CTL11_MSK								(0x03UL << GPIO_CTL1_CTL11_POS)		/** Port x configuration bits (x = 11) */
#define GPIO_CTL1_MD11_POS								0x0CUL		/** Port x mode bits (x = 11 ) */
#define GPIO_CTL1_MD11_MSK								(0x03UL << GPIO_CTL1_MD11_POS)		/** Port x mode bits (x = 11 ) */
#define GPIO_CTL1_CTL10_POS								0x0AUL		/** Port x configuration bits (x = 10) */
#define GPIO_CTL1_CTL10_MSK								(0x03UL << GPIO_CTL1_CTL10_POS)		/** Port x configuration bits (x = 10) */
#define GPIO_CTL1_MD10_POS								0x08UL		/** Port x mode bits (x = 10 ) */
#define GPIO_CTL1_MD10_MSK								(0x03UL << GPIO_CTL1_MD10_POS)		/** Port x mode bits (x = 10 ) */
#define GPIO_CTL1_CTL9_POS								0x06UL		/** Port x configuration bits (x = 9) */
#define GPIO_CTL1_CTL9_MSK								(0x03UL << GPIO_CTL1_CTL9_POS)		/** Port x configuration bits (x = 9) */
#define GPIO_CTL1_MD9_POS								0x04UL		/** Port x mode bits (x = 9) */
#define GPIO_CTL1_MD9_MSK								(0x03UL << GPIO_CTL1_MD9_POS)		/** Port x mode bits (x = 9) */
#define GPIO_CTL1_CTL8_POS								0x02UL		/** Port x configuration bits (x = 8) */
#define GPIO_CTL1_CTL8_MSK								(0x03UL << GPIO_CTL1_CTL8_POS)		/** Port x configuration bits (x = 8) */
#define GPIO_CTL1_MD8_POS								0x00UL		/** Port x mode bits (x = 8) */
#define GPIO_CTL1_MD8_MSK								(0x03UL << GPIO_CTL1_MD8_POS)		/** Port x mode bits (x = 8) */
#define GPIO_ISTAT_ISTAT15_POS								0x0FUL		/** Port input status */
#define GPIO_ISTAT_ISTAT15_MSK								(0x01UL << GPIO_ISTAT_ISTAT15_POS)		/** Port input status */
#define GPIO_ISTAT_ISTAT14_POS								0x0EUL		/** Port input status */
#define GPIO_ISTAT_ISTAT14_MSK								(0x01UL << GPIO_ISTAT_ISTAT14_POS)		/** Port input status */
#define GPIO_ISTAT_ISTAT13_POS								0x0DUL		/** Port input status */
#define GPIO_ISTAT_ISTAT13_MSK								(0x01UL << GPIO_ISTAT_ISTAT13_POS)		/** Port input status */
#define GPIO_ISTAT_ISTAT12_POS								0x0CUL		/** Port input status */
#define GPIO_ISTAT_ISTAT12_MSK								(0x01UL << GPIO_ISTAT_ISTAT12_POS)		/** Port input status */
#define GPIO_ISTAT_ISTAT11_POS								0x0BUL		/** Port input status */
#define GPIO_ISTAT_ISTAT11_MSK								(0x01UL << GPIO_ISTAT_ISTAT11_POS)		/** Port input status */
#define GPIO_ISTAT_ISTAT10_POS								0x0AUL		/** Port input status */
#define GPIO_ISTAT_ISTAT10_MSK								(0x01UL << GPIO_ISTAT_ISTAT10_POS)		/** Port input status */
#define GPIO_ISTAT_ISTAT9_POS								0x09UL		/** Port input status */
#define GPIO_ISTAT_ISTAT9_MSK								(0x01UL << GPIO_ISTAT_ISTAT9_POS)		/** Port input status */
#define GPIO_ISTAT_ISTAT8_POS								0x08UL		/** Port input status */
#define GPIO_ISTAT_ISTAT8_MSK								(0x01UL << GPIO_ISTAT_ISTAT8_POS)		/** Port input status */
#define GPIO_ISTAT_ISTAT7_POS								0x07UL		/** Port input status */
#define GPIO_ISTAT_ISTAT7_MSK								(0x01UL << GPIO_ISTAT_ISTAT7_POS)		/** Port input status */
#define GPIO_ISTAT_ISTAT6_POS								0x06UL		/** Port input status */
#define GPIO_ISTAT_ISTAT6_MSK								(0x01UL << GPIO_ISTAT_ISTAT6_POS)		/** Port input status */
#define GPIO_ISTAT_ISTAT5_POS								0x05UL		/** Port input status */
#define GPIO_ISTAT_ISTAT5_MSK								(0x01UL << GPIO_ISTAT_ISTAT5_POS)		/** Port input status */
#define GPIO_ISTAT_ISTAT4_POS								0x04UL		/** Port input status */
#define GPIO_ISTAT_ISTAT4_MSK								(0x01UL << GPIO_ISTAT_ISTAT4_POS)		/** Port input status */
#define GPIO_ISTAT_ISTAT3_POS								0x03UL		/** Port input status */
#define GPIO_ISTAT_ISTAT3_MSK								(0x01UL << GPIO_ISTAT_ISTAT3_POS)		/** Port input status */
#define GPIO_ISTAT_ISTAT2_POS								0x02UL		/** Port input status */
#define GPIO_ISTAT_ISTAT2_MSK								(0x01UL << GPIO_ISTAT_ISTAT2_POS)		/** Port input status */
#define GPIO_ISTAT_ISTAT1_POS								0x01UL		/** Port input status */
#define GPIO_ISTAT_ISTAT1_MSK								(0x01UL << GPIO_ISTAT_ISTAT1_POS)		/** Port input status */
#define GPIO_ISTAT_ISTAT0_POS								0x00UL		/** Port input status */
#define GPIO_ISTAT_ISTAT0_MSK								(0x01UL << GPIO_ISTAT_ISTAT0_POS)		/** Port input status */
#define GPIO_OCTL_OCTL15_POS								0x0FUL		/** Port output control */
#define GPIO_OCTL_OCTL15_MSK								(0x01UL << GPIO_OCTL_OCTL15_POS)		/** Port output control */
#define GPIO_OCTL_OCTL14_POS								0x0EUL		/** Port output control */
#define GPIO_OCTL_OCTL14_MSK								(0x01UL << GPIO_OCTL_OCTL14_POS)		/** Port output control */
#define GPIO_OCTL_OCTL13_POS								0x0DUL		/** Port output control */
#define GPIO_OCTL_OCTL13_MSK								(0x01UL << GPIO_OCTL_OCTL13_POS)		/** Port output control */
#define GPIO_OCTL_OCTL12_POS								0x0CUL		/** Port output control */
#define GPIO_OCTL_OCTL12_MSK								(0x01UL << GPIO_OCTL_OCTL12_POS)		/** Port output control */
#define GPIO_OCTL_OCTL11_POS								0x0BUL		/** Port output control */
#define GPIO_OCTL_OCTL11_MSK								(0x01UL << GPIO_OCTL_OCTL11_POS)		/** Port output control */
#define GPIO_OCTL_OCTL10_POS								0x0AUL		/** Port output control */
#define GPIO_OCTL_OCTL10_MSK								(0x01UL << GPIO_OCTL_OCTL10_POS)		/** Port output control */
#define GPIO_OCTL_OCTL9_POS								0x09UL		/** Port output control */
#define GPIO_OCTL_OCTL9_MSK								(0x01UL << GPIO_OCTL_OCTL9_POS)		/** Port output control */
#define GPIO_OCTL_OCTL8_POS								0x08UL		/** Port output control */
#define GPIO_OCTL_OCTL8_MSK								(0x01UL << GPIO_OCTL_OCTL8_POS)		/** Port output control */
#define GPIO_OCTL_OCTL7_POS								0x07UL		/** Port output control */
#define GPIO_OCTL_OCTL7_MSK								(0x01UL << GPIO_OCTL_OCTL7_POS)		/** Port output control */
#define GPIO_OCTL_OCTL6_POS								0x06UL		/** Port output control */
#define GPIO_OCTL_OCTL6_MSK								(0x01UL << GPIO_OCTL_OCTL6_POS)		/** Port output control */
#define GPIO_OCTL_OCTL5_POS								0x05UL		/** Port output control */
#define GPIO_OCTL_OCTL5_MSK								(0x01UL << GPIO_OCTL_OCTL5_POS)		/** Port output control */
#define GPIO_OCTL_OCTL4_POS								0x04UL		/** Port output control */
#define GPIO_OCTL_OCTL4_MSK								(0x01UL << GPIO_OCTL_OCTL4_POS)		/** Port output control */
#define GPIO_OCTL_OCTL3_POS								0x03UL		/** Port output control */
#define GPIO_OCTL_OCTL3_MSK								(0x01UL << GPIO_OCTL_OCTL3_POS)		/** Port output control */
#define GPIO_OCTL_OCTL2_POS								0x02UL		/** Port output control */
#define GPIO_OCTL_OCTL2_MSK								(0x01UL << GPIO_OCTL_OCTL2_POS)		/** Port output control */
#define GPIO_OCTL_OCTL1_POS								0x01UL		/** Port output control */
#define GPIO_OCTL_OCTL1_MSK								(0x01UL << GPIO_OCTL_OCTL1_POS)		/** Port output control */
#define GPIO_OCTL_OCTL0_POS								0x00UL		/** Port output control */
#define GPIO_OCTL_OCTL0_MSK								(0x01UL << GPIO_OCTL_OCTL0_POS)		/** Port output control */
#define GPIO_BOP_CR15_POS								0x1FUL		/** Port 15 Clear bit */
#define GPIO_BOP_CR15_MSK								(0x01UL << GPIO_BOP_CR15_POS)		/** Port 15 Clear bit */
#define GPIO_BOP_CR14_POS								0x1EUL		/** Port 14 Clear bit */
#define GPIO_BOP_CR14_MSK								(0x01UL << GPIO_BOP_CR14_POS)		/** Port 14 Clear bit */
#define GPIO_BOP_CR13_POS								0x1DUL		/** Port 13 Clear bit */
#define GPIO_BOP_CR13_MSK								(0x01UL << GPIO_BOP_CR13_POS)		/** Port 13 Clear bit */
#define GPIO_BOP_CR12_POS								0x1CUL		/** Port 12 Clear bit */
#define GPIO_BOP_CR12_MSK								(0x01UL << GPIO_BOP_CR12_POS)		/** Port 12 Clear bit */
#define GPIO_BOP_CR11_POS								0x1BUL		/** Port 11 Clear bit */
#define GPIO_BOP_CR11_MSK								(0x01UL << GPIO_BOP_CR11_POS)		/** Port 11 Clear bit */
#define GPIO_BOP_CR10_POS								0x1AUL		/** Port 10 Clear bit */
#define GPIO_BOP_CR10_MSK								(0x01UL << GPIO_BOP_CR10_POS)		/** Port 10 Clear bit */
#define GPIO_BOP_CR9_POS								0x19UL		/** Port 9 Clear bit */
#define GPIO_BOP_CR9_MSK								(0x01UL << GPIO_BOP_CR9_POS)		/** Port 9 Clear bit */
#define GPIO_BOP_CR8_POS								0x18UL		/** Port 8 Clear bit */
#define GPIO_BOP_CR8_MSK								(0x01UL << GPIO_BOP_CR8_POS)		/** Port 8 Clear bit */
#define GPIO_BOP_CR7_POS								0x17UL		/** Port 7 Clear bit */
#define GPIO_BOP_CR7_MSK								(0x01UL << GPIO_BOP_CR7_POS)		/** Port 7 Clear bit */
#define GPIO_BOP_CR6_POS								0x16UL		/** Port 6 Clear bit */
#define GPIO_BOP_CR6_MSK								(0x01UL << GPIO_BOP_CR6_POS)		/** Port 6 Clear bit */
#define GPIO_BOP_CR5_POS								0x15UL		/** Port 5 Clear bit */
#define GPIO_BOP_CR5_MSK								(0x01UL << GPIO_BOP_CR5_POS)		/** Port 5 Clear bit */
#define GPIO_BOP_CR4_POS								0x14UL		/** Port 4 Clear bit */
#define GPIO_BOP_CR4_MSK								(0x01UL << GPIO_BOP_CR4_POS)		/** Port 4 Clear bit */
#define GPIO_BOP_CR3_POS								0x13UL		/** Port 3 Clear bit */
#define GPIO_BOP_CR3_MSK								(0x01UL << GPIO_BOP_CR3_POS)		/** Port 3 Clear bit */
#define GPIO_BOP_CR2_POS								0x12UL		/** Port 2 Clear bit */
#define GPIO_BOP_CR2_MSK								(0x01UL << GPIO_BOP_CR2_POS)		/** Port 2 Clear bit */
#define GPIO_BOP_CR1_POS								0x11UL		/** Port 1 Clear bit */
#define GPIO_BOP_CR1_MSK								(0x01UL << GPIO_BOP_CR1_POS)		/** Port 1 Clear bit */
#define GPIO_BOP_CR0_POS								0x10UL		/** Port 0 Clear bit */
#define GPIO_BOP_CR0_MSK								(0x01UL << GPIO_BOP_CR0_POS)		/** Port 0 Clear bit */
#define GPIO_BOP_BOP15_POS								0x0FUL		/** Port 15 Set bit */
#define GPIO_BOP_BOP15_MSK								(0x01UL << GPIO_BOP_BOP15_POS)		/** Port 15 Set bit */
#define GPIO_BOP_BOP14_POS								0x0EUL		/** Port 14 Set bit */
#define GPIO_BOP_BOP14_MSK								(0x01UL << GPIO_BOP_BOP14_POS)		/** Port 14 Set bit */
#define GPIO_BOP_BOP13_POS								0x0DUL		/** Port 13 Set bit */
#define GPIO_BOP_BOP13_MSK								(0x01UL << GPIO_BOP_BOP13_POS)		/** Port 13 Set bit */
#define GPIO_BOP_BOP12_POS								0x0CUL		/** Port 12 Set bit */
#define GPIO_BOP_BOP12_MSK								(0x01UL << GPIO_BOP_BOP12_POS)		/** Port 12 Set bit */
#define GPIO_BOP_BOP11_POS								0x0BUL		/** Port 11 Set bit */
#define GPIO_BOP_BOP11_MSK								(0x01UL << GPIO_BOP_BOP11_POS)		/** Port 11 Set bit */
#define GPIO_BOP_BOP10_POS								0x0AUL		/** Port 10 Set bit */
#define GPIO_BOP_BOP10_MSK								(0x01UL << GPIO_BOP_BOP10_POS)		/** Port 10 Set bit */
#define GPIO_BOP_BOP9_POS								0x09UL		/** Port 9 Set bit */
#define GPIO_BOP_BOP9_MSK								(0x01UL << GPIO_BOP_BOP9_POS)		/** Port 9 Set bit */
#define GPIO_BOP_BOP8_POS								0x08UL		/** Port 8 Set bit */
#define GPIO_BOP_BOP8_MSK								(0x01UL << GPIO_BOP_BOP8_POS)		/** Port 8 Set bit */
#define GPIO_BOP_BOP7_POS								0x07UL		/** Port 7 Set bit */
#define GPIO_BOP_BOP7_MSK								(0x01UL << GPIO_BOP_BOP7_POS)		/** Port 7 Set bit */
#define GPIO_BOP_BOP6_POS								0x06UL		/** Port 6 Set bit */
#define GPIO_BOP_BOP6_MSK								(0x01UL << GPIO_BOP_BOP6_POS)		/** Port 6 Set bit */
#define GPIO_BOP_BOP5_POS								0x05UL		/** Port 5 Set bit */
#define GPIO_BOP_BOP5_MSK								(0x01UL << GPIO_BOP_BOP5_POS)		/** Port 5 Set bit */
#define GPIO_BOP_BOP4_POS								0x04UL		/** Port 4 Set bit */
#define GPIO_BOP_BOP4_MSK								(0x01UL << GPIO_BOP_BOP4_POS)		/** Port 4 Set bit */
#define GPIO_BOP_BOP3_POS								0x03UL		/** Port 3 Set bit */
#define GPIO_BOP_BOP3_MSK								(0x01UL << GPIO_BOP_BOP3_POS)		/** Port 3 Set bit */
#define GPIO_BOP_BOP2_POS								0x02UL		/** Port 2 Set bit */
#define GPIO_BOP_BOP2_MSK								(0x01UL << GPIO_BOP_BOP2_POS)		/** Port 2 Set bit */
#define GPIO_BOP_BOP1_POS								0x01UL		/** Port 1 Set bit */
#define GPIO_BOP_BOP1_MSK								(0x01UL << GPIO_BOP_BOP1_POS)		/** Port 1 Set bit */
#define GPIO_BOP_BOP0_POS								0x00UL		/** Port 0 Set bit */
#define GPIO_BOP_BOP0_MSK								(0x01UL << GPIO_BOP_BOP0_POS)		/** Port 0 Set bit */
#define GPIO_BC_CR15_POS								0x0FUL		/** Port 15 Clear bit */
#define GPIO_BC_CR15_MSK								(0x01UL << GPIO_BC_CR15_POS)		/** Port 15 Clear bit */
#define GPIO_BC_CR14_POS								0x0EUL		/** Port 14 Clear bit */
#define GPIO_BC_CR14_MSK								(0x01UL << GPIO_BC_CR14_POS)		/** Port 14 Clear bit */
#define GPIO_BC_CR13_POS								0x0DUL		/** Port 13 Clear bit */
#define GPIO_BC_CR13_MSK								(0x01UL << GPIO_BC_CR13_POS)		/** Port 13 Clear bit */
#define GPIO_BC_CR12_POS								0x0CUL		/** Port 12 Clear bit */
#define GPIO_BC_CR12_MSK								(0x01UL << GPIO_BC_CR12_POS)		/** Port 12 Clear bit */
#define GPIO_BC_CR11_POS								0x0BUL		/** Port 11 Clear bit */
#define GPIO_BC_CR11_MSK								(0x01UL << GPIO_BC_CR11_POS)		/** Port 11 Clear bit */
#define GPIO_BC_CR10_POS								0x0AUL		/** Port 10 Clear bit */
#define GPIO_BC_CR10_MSK								(0x01UL << GPIO_BC_CR10_POS)		/** Port 10 Clear bit */
#define GPIO_BC_CR9_POS								0x09UL		/** Port 9 Clear bit */
#define GPIO_BC_CR9_MSK								(0x01UL << GPIO_BC_CR9_POS)		/** Port 9 Clear bit */
#define GPIO_BC_CR8_POS								0x08UL		/** Port 8 Clear bit */
#define GPIO_BC_CR8_MSK								(0x01UL << GPIO_BC_CR8_POS)		/** Port 8 Clear bit */
#define GPIO_BC_CR7_POS								0x07UL		/** Port 7 Clear bit */
#define GPIO_BC_CR7_MSK								(0x01UL << GPIO_BC_CR7_POS)		/** Port 7 Clear bit */
#define GPIO_BC_CR6_POS								0x06UL		/** Port 6 Clear bit */
#define GPIO_BC_CR6_MSK								(0x01UL << GPIO_BC_CR6_POS)		/** Port 6 Clear bit */
#define GPIO_BC_CR5_POS								0x05UL		/** Port 5 Clear bit */
#define GPIO_BC_CR5_MSK								(0x01UL << GPIO_BC_CR5_POS)		/** Port 5 Clear bit */
#define GPIO_BC_CR4_POS								0x04UL		/** Port 4 Clear bit */
#define GPIO_BC_CR4_MSK								(0x01UL << GPIO_BC_CR4_POS)		/** Port 4 Clear bit */
#define GPIO_BC_CR3_POS								0x03UL		/** Port 3 Clear bit */
#define GPIO_BC_CR3_MSK								(0x01UL << GPIO_BC_CR3_POS)		/** Port 3 Clear bit */
#define GPIO_BC_CR2_POS								0x02UL		/** Port 2 Clear bit */
#define GPIO_BC_CR2_MSK								(0x01UL << GPIO_BC_CR2_POS)		/** Port 2 Clear bit */
#define GPIO_BC_CR1_POS								0x01UL		/** Port 1 Clear bit */
#define GPIO_BC_CR1_MSK								(0x01UL << GPIO_BC_CR1_POS)		/** Port 1 Clear bit */
#define GPIO_BC_CR0_POS								0x00UL		/** Port 0 Clear bit */
#define GPIO_BC_CR0_MSK								(0x01UL << GPIO_BC_CR0_POS)		/** Port 0 Clear bit */
#define GPIO_LOCK_LKK_POS								0x10UL		/** Lock sequence key  */
#define GPIO_LOCK_LKK_MSK								(0x01UL << GPIO_LOCK_LKK_POS)		/** Lock sequence key  */
#define GPIO_LOCK_LK15_POS								0x0FUL		/** Port Lock bit 15 */
#define GPIO_LOCK_LK15_MSK								(0x01UL << GPIO_LOCK_LK15_POS)		/** Port Lock bit 15 */
#define GPIO_LOCK_LK14_POS								0x0EUL		/** Port Lock bit 14 */
#define GPIO_LOCK_LK14_MSK								(0x01UL << GPIO_LOCK_LK14_POS)		/** Port Lock bit 14 */
#define GPIO_LOCK_LK13_POS								0x0DUL		/** Port Lock bit 13 */
#define GPIO_LOCK_LK13_MSK								(0x01UL << GPIO_LOCK_LK13_POS)		/** Port Lock bit 13 */
#define GPIO_LOCK_LK12_POS								0x0CUL		/** Port Lock bit 12 */
#define GPIO_LOCK_LK12_MSK								(0x01UL << GPIO_LOCK_LK12_POS)		/** Port Lock bit 12 */
#define GPIO_LOCK_LK11_POS								0x0BUL		/** Port Lock bit 11 */
#define GPIO_LOCK_LK11_MSK								(0x01UL << GPIO_LOCK_LK11_POS)		/** Port Lock bit 11 */
#define GPIO_LOCK_LK10_POS								0x0AUL		/** Port Lock bit 10 */
#define GPIO_LOCK_LK10_MSK								(0x01UL << GPIO_LOCK_LK10_POS)		/** Port Lock bit 10 */
#define GPIO_LOCK_LK9_POS								0x09UL		/** Port Lock bit 9 */
#define GPIO_LOCK_LK9_MSK								(0x01UL << GPIO_LOCK_LK9_POS)		/** Port Lock bit 9 */
#define GPIO_LOCK_LK8_POS								0x08UL		/** Port Lock bit 8 */
#define GPIO_LOCK_LK8_MSK								(0x01UL << GPIO_LOCK_LK8_POS)		/** Port Lock bit 8 */
#define GPIO_LOCK_LK7_POS								0x07UL		/** Port Lock bit 7 */
#define GPIO_LOCK_LK7_MSK								(0x01UL << GPIO_LOCK_LK7_POS)		/** Port Lock bit 7 */
#define GPIO_LOCK_LK6_POS								0x06UL		/** Port Lock bit 6 */
#define GPIO_LOCK_LK6_MSK								(0x01UL << GPIO_LOCK_LK6_POS)		/** Port Lock bit 6 */
#define GPIO_LOCK_LK5_POS								0x05UL		/** Port Lock bit 5 */
#define GPIO_LOCK_LK5_MSK								(0x01UL << GPIO_LOCK_LK5_POS)		/** Port Lock bit 5 */
#define GPIO_LOCK_LK4_POS								0x04UL		/** Port Lock bit 4 */
#define GPIO_LOCK_LK4_MSK								(0x01UL << GPIO_LOCK_LK4_POS)		/** Port Lock bit 4 */
#define GPIO_LOCK_LK3_POS								0x03UL		/** Port Lock bit 3 */
#define GPIO_LOCK_LK3_MSK								(0x01UL << GPIO_LOCK_LK3_POS)		/** Port Lock bit 3 */
#define GPIO_LOCK_LK2_POS								0x02UL		/** Port Lock bit 2 */
#define GPIO_LOCK_LK2_MSK								(0x01UL << GPIO_LOCK_LK2_POS)		/** Port Lock bit 2 */
#define GPIO_LOCK_LK1_POS								0x01UL		/** Port Lock bit 1 */
#define GPIO_LOCK_LK1_MSK								(0x01UL << GPIO_LOCK_LK1_POS)		/** Port Lock bit 1 */
#define GPIO_LOCK_LK0_POS								0x00UL		/** Port Lock bit 0 */
#define GPIO_LOCK_LK0_MSK								(0x01UL << GPIO_LOCK_LK0_POS)		/** Port Lock bit 0 */
#define I2C_CTL0_SRESET_POS								0x0FUL		/** Software reset */
#define I2C_CTL0_SRESET_MSK								(0x01UL << I2C_CTL0_SRESET_POS)		/** Software reset */
#define I2C_CTL0_SALT_POS								0x0DUL		/** SMBus alert */
#define I2C_CTL0_SALT_MSK								(0x01UL << I2C_CTL0_SALT_POS)		/** SMBus alert */
#define I2C_CTL0_PECTRANS_POS								0x0CUL		/** PEC Transfer */
#define I2C_CTL0_PECTRANS_MSK								(0x01UL << I2C_CTL0_PECTRANS_POS)		/** PEC Transfer */
#define I2C_CTL0_POAP_POS								0x0BUL		/** Position of ACK and PEC when receiving */
#define I2C_CTL0_POAP_MSK								(0x01UL << I2C_CTL0_POAP_POS)		/** Position of ACK and PEC when receiving */
#define I2C_CTL0_ACKEN_POS								0x0AUL		/** Whether or not to send an ACK */
#define I2C_CTL0_ACKEN_MSK								(0x01UL << I2C_CTL0_ACKEN_POS)		/** Whether or not to send an ACK */
#define I2C_CTL0_STOP_POS								0x09UL		/** Generate a STOP condition on I2C bus */
#define I2C_CTL0_STOP_MSK								(0x01UL << I2C_CTL0_STOP_POS)		/** Generate a STOP condition on I2C bus */
#define I2C_CTL0_START_POS								0x08UL		/** Generate a START condition on I2C bus */
#define I2C_CTL0_START_MSK								(0x01UL << I2C_CTL0_START_POS)		/** Generate a START condition on I2C bus */
#define I2C_CTL0_SS_POS								0x07UL		/** Whether to stretch SCL low when data is not ready in slave mode */
#define I2C_CTL0_SS_MSK								(0x01UL << I2C_CTL0_SS_POS)		/** Whether to stretch SCL low when data is not ready in slave mode */
#define I2C_CTL0_GCEN_POS								0x06UL		/** Whether or not to response to a General Call (0x00) */
#define I2C_CTL0_GCEN_MSK								(0x01UL << I2C_CTL0_GCEN_POS)		/** Whether or not to response to a General Call (0x00) */
#define I2C_CTL0_PECEN_POS								0x05UL		/** PEC Calculation Switch */
#define I2C_CTL0_PECEN_MSK								(0x01UL << I2C_CTL0_PECEN_POS)		/** PEC Calculation Switch */
#define I2C_CTL0_ARPEN_POS								0x04UL		/** ARP protocol in SMBus switch */
#define I2C_CTL0_ARPEN_MSK								(0x01UL << I2C_CTL0_ARPEN_POS)		/** ARP protocol in SMBus switch */
#define I2C_CTL0_SMBSEL_POS								0x03UL		/** SMBusType Selection */
#define I2C_CTL0_SMBSEL_MSK								(0x01UL << I2C_CTL0_SMBSEL_POS)		/** SMBusType Selection */
#define I2C_CTL0_SMBEN_POS								0x01UL		/** SMBus/I2C mode switch */
#define I2C_CTL0_SMBEN_MSK								(0x01UL << I2C_CTL0_SMBEN_POS)		/** SMBus/I2C mode switch */
#define I2C_CTL0_I2CEN_POS								0x00UL		/** I2C peripheral enable */
#define I2C_CTL0_I2CEN_MSK								(0x01UL << I2C_CTL0_I2CEN_POS)		/** I2C peripheral enable */
#define I2C_CTL1_DMALST_POS								0x0CUL		/** Flag indicating DMA last transfer */
#define I2C_CTL1_DMALST_MSK								(0x01UL << I2C_CTL1_DMALST_POS)		/** Flag indicating DMA last transfer */
#define I2C_CTL1_DMAON_POS								0x0BUL		/** DMA mode switch */
#define I2C_CTL1_DMAON_MSK								(0x01UL << I2C_CTL1_DMAON_POS)		/** DMA mode switch */
#define I2C_CTL1_BUFIE_POS								0x0AUL		/** Buffer interrupt enable */
#define I2C_CTL1_BUFIE_MSK								(0x01UL << I2C_CTL1_BUFIE_POS)		/** Buffer interrupt enable */
#define I2C_CTL1_EVIE_POS								0x09UL		/** Event interrupt enable */
#define I2C_CTL1_EVIE_MSK								(0x01UL << I2C_CTL1_EVIE_POS)		/** Event interrupt enable */
#define I2C_CTL1_ERRIE_POS								0x08UL		/** Error interrupt enable */
#define I2C_CTL1_ERRIE_MSK								(0x01UL << I2C_CTL1_ERRIE_POS)		/** Error interrupt enable */
#define I2C_CTL1_I2CCLK_POS								0x00UL		/** I2C Peripheral clock frequency */
#define I2C_CTL1_I2CCLK_MSK								(0x3FUL << I2C_CTL1_I2CCLK_POS)		/** I2C Peripheral clock frequency */
#define I2C_SADDR0_ADDFORMAT_POS								0x0FUL		/** Address mode for the I2C slave */
#define I2C_SADDR0_ADDFORMAT_MSK								(0x01UL << I2C_SADDR0_ADDFORMAT_POS)		/** Address mode for the I2C slave */
#define I2C_SADDR0_ADDRESS9_8_POS								0x08UL		/** Highest two bits of a 10-bit address */
#define I2C_SADDR0_ADDRESS9_8_MSK								(0x03UL << I2C_SADDR0_ADDRESS9_8_POS)		/** Highest two bits of a 10-bit address */
#define I2C_SADDR0_ADDRESS7_1_POS								0x01UL		/** 7-bit address or bits 7:1 of a 10-bit address */
#define I2C_SADDR0_ADDRESS7_1_MSK								(0x7FUL << I2C_SADDR0_ADDRESS7_1_POS)		/** 7-bit address or bits 7:1 of a 10-bit address */
#define I2C_SADDR0_ADDRESS0_POS								0x00UL		/** Bit 0 of a 10-bit address */
#define I2C_SADDR0_ADDRESS0_MSK								(0x01UL << I2C_SADDR0_ADDRESS0_POS)		/** Bit 0 of a 10-bit address */
#define I2C_SADDR1_ADDRESS2_POS								0x01UL		/** Second I2C address for the slave in Dual-Address mode */
#define I2C_SADDR1_ADDRESS2_MSK								(0x7FUL << I2C_SADDR1_ADDRESS2_POS)		/** Second I2C address for the slave in Dual-Address mode */
#define I2C_SADDR1_DUADEN_POS								0x00UL		/** Dual-Address mode switch */
#define I2C_SADDR1_DUADEN_MSK								(0x01UL << I2C_SADDR1_DUADEN_POS)		/** Dual-Address mode switch */
#define I2C_DATA_TRB_POS								0x00UL		/** Transmission or reception data buffer register */
#define I2C_DATA_TRB_MSK								(0xFFUL << I2C_DATA_TRB_POS)		/** Transmission or reception data buffer register */
#define I2C_STAT0_SMBALT_POS								0x0FUL		/** SMBus Alert status */
#define I2C_STAT0_SMBALT_MSK								(0x01UL << I2C_STAT0_SMBALT_POS)		/** SMBus Alert status */
#define I2C_STAT0_SMBTO_POS								0x0EUL		/** Timeout signal in SMBus mode */
#define I2C_STAT0_SMBTO_MSK								(0x01UL << I2C_STAT0_SMBTO_POS)		/** Timeout signal in SMBus mode */
#define I2C_STAT0_PECERR_POS								0x0CUL		/** PEC error when receiving data */
#define I2C_STAT0_PECERR_MSK								(0x01UL << I2C_STAT0_PECERR_POS)		/** PEC error when receiving data */
#define I2C_STAT0_OUERR_POS								0x0BUL		/** Over-run or under-run situation occurs in slave mode */
#define I2C_STAT0_OUERR_MSK								(0x01UL << I2C_STAT0_OUERR_POS)		/** Over-run or under-run situation occurs in slave mode */
#define I2C_STAT0_AERR_POS								0x0AUL		/** Acknowledge error */
#define I2C_STAT0_AERR_MSK								(0x01UL << I2C_STAT0_AERR_POS)		/** Acknowledge error */
#define I2C_STAT0_LOSTARB_POS								0x09UL		/** Arbitration Lost in master mode */
#define I2C_STAT0_LOSTARB_MSK								(0x01UL << I2C_STAT0_LOSTARB_POS)		/** Arbitration Lost in master mode */
#define I2C_STAT0_BERR_POS								0x08UL		/** A bus error occurs indication a unexpected START or STOP condition on I2C bus */
#define I2C_STAT0_BERR_MSK								(0x01UL << I2C_STAT0_BERR_POS)		/** A bus error occurs indication a unexpected START or STOP condition on I2C bus */
#define I2C_STAT0_TBE_POS								0x07UL		/** I2C_DATA is Empty during transmitting */
#define I2C_STAT0_TBE_MSK								(0x01UL << I2C_STAT0_TBE_POS)		/** I2C_DATA is Empty during transmitting */
#define I2C_STAT0_RBNE_POS								0x06UL		/** I2C_DATA is not Empty during receiving */
#define I2C_STAT0_RBNE_MSK								(0x01UL << I2C_STAT0_RBNE_POS)		/** I2C_DATA is not Empty during receiving */
#define I2C_STAT0_STPDET_POS								0x04UL		/** STOP condition detected in slave mode */
#define I2C_STAT0_STPDET_MSK								(0x01UL << I2C_STAT0_STPDET_POS)		/** STOP condition detected in slave mode */
#define I2C_STAT0_ADD10SEND_POS								0x03UL		/** Header of 10-bit address is sent in master mode */
#define I2C_STAT0_ADD10SEND_MSK								(0x01UL << I2C_STAT0_ADD10SEND_POS)		/** Header of 10-bit address is sent in master mode */
#define I2C_STAT0_BTC_POS								0x02UL		/** Byte transmission completed */
#define I2C_STAT0_BTC_MSK								(0x01UL << I2C_STAT0_BTC_POS)		/** Byte transmission completed */
#define I2C_STAT0_ADDSEND_POS								0x01UL		/** Address is sent in master mode or received and matches in slave mode */
#define I2C_STAT0_ADDSEND_MSK								(0x01UL << I2C_STAT0_ADDSEND_POS)		/** Address is sent in master mode or received and matches in slave mode */
#define I2C_STAT0_SBSEND_POS								0x00UL		/** START condition sent out in master mode */
#define I2C_STAT0_SBSEND_MSK								(0x01UL << I2C_STAT0_SBSEND_POS)		/** START condition sent out in master mode */
#define I2C_STAT1_PECV_POS								0x08UL		/** Packet Error Checking Value that calculated by hardware when PEC is enabled */
#define I2C_STAT1_PECV_MSK								(0xFFUL << I2C_STAT1_PECV_POS)		/** Packet Error Checking Value that calculated by hardware when PEC is enabled */
#define I2C_STAT1_DUMODF_POS								0x07UL		/** Dual Flag in slave mode */
#define I2C_STAT1_DUMODF_MSK								(0x01UL << I2C_STAT1_DUMODF_POS)		/** Dual Flag in slave mode */
#define I2C_STAT1_HSTSMB_POS								0x06UL		/** SMBus Host Header detected in slave mode */
#define I2C_STAT1_HSTSMB_MSK								(0x01UL << I2C_STAT1_HSTSMB_POS)		/** SMBus Host Header detected in slave mode */
#define I2C_STAT1_DEFSMB_POS								0x05UL		/** Default address of SMBusDevice */
#define I2C_STAT1_DEFSMB_MSK								(0x01UL << I2C_STAT1_DEFSMB_POS)		/** Default address of SMBusDevice */
#define I2C_STAT1_RXGC_POS								0x04UL		/** General call address (00h) received */
#define I2C_STAT1_RXGC_MSK								(0x01UL << I2C_STAT1_RXGC_POS)		/** General call address (00h) received */
#define I2C_STAT1_TR_POS								0x02UL		/** Whether the I2C is a transmitter or a receiver */
#define I2C_STAT1_TR_MSK								(0x01UL << I2C_STAT1_TR_POS)		/** Whether the I2C is a transmitter or a receiver */
#define I2C_STAT1_I2CBSY_POS								0x01UL		/** Busy flag */
#define I2C_STAT1_I2CBSY_MSK								(0x01UL << I2C_STAT1_I2CBSY_POS)		/** Busy flag */
#define I2C_STAT1_MASTER_POS								0x00UL		/** A flag indicating whether I2C block is in master or slave mode */
#define I2C_STAT1_MASTER_MSK								(0x01UL << I2C_STAT1_MASTER_POS)		/** A flag indicating whether I2C block is in master or slave mode */
#define I2C_CKCFG_FAST_POS								0x0FUL		/** I2C speed selection in master mode */
#define I2C_CKCFG_FAST_MSK								(0x01UL << I2C_CKCFG_FAST_POS)		/** I2C speed selection in master mode */
#define I2C_CKCFG_DTCY_POS								0x0EUL		/** Duty cycle in fast mode */
#define I2C_CKCFG_DTCY_MSK								(0x01UL << I2C_CKCFG_DTCY_POS)		/** Duty cycle in fast mode */
#define I2C_CKCFG_CLKC_POS								0x00UL		/** I2C Clock control in master mode */
#define I2C_CKCFG_CLKC_MSK								(0xFFFUL << I2C_CKCFG_CLKC_POS)		/** I2C Clock control in master mode */
#define I2C_RT_RISETIME_POS								0x00UL		/** Maximum rise time in master mode */
#define I2C_RT_RISETIME_MSK								(0x3FUL << I2C_RT_RISETIME_POS)		/** Maximum rise time in master mode */
#define ECLIC_CLICCFG_NLBITS_POS								0x01UL		/** NLBITS */
#define ECLIC_CLICCFG_NLBITS_MSK								(0x0FUL << ECLIC_CLICCFG_NLBITS_POS)		/** NLBITS */
#define ECLIC_CLICINFO_NUM_INTERRUPT_POS								0x00UL		/** NUM_INTERRUPT */
#define ECLIC_CLICINFO_NUM_INTERRUPT_MSK								(0x1FFFUL << ECLIC_CLICINFO_NUM_INTERRUPT_POS)		/** NUM_INTERRUPT */
#define ECLIC_CLICINFO_VERSION_POS								0x0DUL		/** VERSION */
#define ECLIC_CLICINFO_VERSION_MSK								(0xFFUL << ECLIC_CLICINFO_VERSION_POS)		/** VERSION */
#define ECLIC_CLICINFO_CLICINTCTLBITS_POS								0x15UL		/** CLICINTCTLBITS */
#define ECLIC_CLICINFO_CLICINTCTLBITS_MSK								(0x0FUL << ECLIC_CLICINFO_CLICINTCTLBITS_POS)		/** CLICINTCTLBITS */
#define ECLIC_MTH_MTH_POS								0x00UL		/** MTH */
#define ECLIC_MTH_MTH_MSK								(0xFFUL << ECLIC_MTH_MTH_POS)		/** MTH */
#define ECLIC_CLICINTIP_0_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_0_IP_MSK								(0x01UL << ECLIC_CLICINTIP_0_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_1_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_1_IP_MSK								(0x01UL << ECLIC_CLICINTIP_1_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_2_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_2_IP_MSK								(0x01UL << ECLIC_CLICINTIP_2_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_3_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_3_IP_MSK								(0x01UL << ECLIC_CLICINTIP_3_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_4_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_4_IP_MSK								(0x01UL << ECLIC_CLICINTIP_4_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_5_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_5_IP_MSK								(0x01UL << ECLIC_CLICINTIP_5_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_6_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_6_IP_MSK								(0x01UL << ECLIC_CLICINTIP_6_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_7_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_7_IP_MSK								(0x01UL << ECLIC_CLICINTIP_7_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_8_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_8_IP_MSK								(0x01UL << ECLIC_CLICINTIP_8_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_9_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_9_IP_MSK								(0x01UL << ECLIC_CLICINTIP_9_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_10_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_10_IP_MSK								(0x01UL << ECLIC_CLICINTIP_10_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_11_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_11_IP_MSK								(0x01UL << ECLIC_CLICINTIP_11_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_12_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_12_IP_MSK								(0x01UL << ECLIC_CLICINTIP_12_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_13_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_13_IP_MSK								(0x01UL << ECLIC_CLICINTIP_13_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_14_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_14_IP_MSK								(0x01UL << ECLIC_CLICINTIP_14_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_15_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_15_IP_MSK								(0x01UL << ECLIC_CLICINTIP_15_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_16_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_16_IP_MSK								(0x01UL << ECLIC_CLICINTIP_16_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_17_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_17_IP_MSK								(0x01UL << ECLIC_CLICINTIP_17_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_18_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_18_IP_MSK								(0x01UL << ECLIC_CLICINTIP_18_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_19_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_19_IP_MSK								(0x01UL << ECLIC_CLICINTIP_19_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_20_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_20_IP_MSK								(0x01UL << ECLIC_CLICINTIP_20_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_21_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_21_IP_MSK								(0x01UL << ECLIC_CLICINTIP_21_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_22_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_22_IP_MSK								(0x01UL << ECLIC_CLICINTIP_22_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_23_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_23_IP_MSK								(0x01UL << ECLIC_CLICINTIP_23_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_24_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_24_IP_MSK								(0x01UL << ECLIC_CLICINTIP_24_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_25_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_25_IP_MSK								(0x01UL << ECLIC_CLICINTIP_25_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_26_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_26_IP_MSK								(0x01UL << ECLIC_CLICINTIP_26_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_27_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_27_IP_MSK								(0x01UL << ECLIC_CLICINTIP_27_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_28_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_28_IP_MSK								(0x01UL << ECLIC_CLICINTIP_28_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_29_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_29_IP_MSK								(0x01UL << ECLIC_CLICINTIP_29_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_30_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_30_IP_MSK								(0x01UL << ECLIC_CLICINTIP_30_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_31_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_31_IP_MSK								(0x01UL << ECLIC_CLICINTIP_31_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_32_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_32_IP_MSK								(0x01UL << ECLIC_CLICINTIP_32_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_33_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_33_IP_MSK								(0x01UL << ECLIC_CLICINTIP_33_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_34_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_34_IP_MSK								(0x01UL << ECLIC_CLICINTIP_34_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_35_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_35_IP_MSK								(0x01UL << ECLIC_CLICINTIP_35_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_36_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_36_IP_MSK								(0x01UL << ECLIC_CLICINTIP_36_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_37_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_37_IP_MSK								(0x01UL << ECLIC_CLICINTIP_37_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_38_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_38_IP_MSK								(0x01UL << ECLIC_CLICINTIP_38_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_39_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_39_IP_MSK								(0x01UL << ECLIC_CLICINTIP_39_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_40_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_40_IP_MSK								(0x01UL << ECLIC_CLICINTIP_40_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_41_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_41_IP_MSK								(0x01UL << ECLIC_CLICINTIP_41_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_42_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_42_IP_MSK								(0x01UL << ECLIC_CLICINTIP_42_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_43_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_43_IP_MSK								(0x01UL << ECLIC_CLICINTIP_43_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_44_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_44_IP_MSK								(0x01UL << ECLIC_CLICINTIP_44_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_45_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_45_IP_MSK								(0x01UL << ECLIC_CLICINTIP_45_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_46_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_46_IP_MSK								(0x01UL << ECLIC_CLICINTIP_46_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_47_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_47_IP_MSK								(0x01UL << ECLIC_CLICINTIP_47_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_48_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_48_IP_MSK								(0x01UL << ECLIC_CLICINTIP_48_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_49_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_49_IP_MSK								(0x01UL << ECLIC_CLICINTIP_49_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_50_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_50_IP_MSK								(0x01UL << ECLIC_CLICINTIP_50_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_51_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_51_IP_MSK								(0x01UL << ECLIC_CLICINTIP_51_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_52_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_52_IP_MSK								(0x01UL << ECLIC_CLICINTIP_52_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_53_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_53_IP_MSK								(0x01UL << ECLIC_CLICINTIP_53_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_54_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_54_IP_MSK								(0x01UL << ECLIC_CLICINTIP_54_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_55_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_55_IP_MSK								(0x01UL << ECLIC_CLICINTIP_55_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_56_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_56_IP_MSK								(0x01UL << ECLIC_CLICINTIP_56_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_57_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_57_IP_MSK								(0x01UL << ECLIC_CLICINTIP_57_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_58_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_58_IP_MSK								(0x01UL << ECLIC_CLICINTIP_58_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_59_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_59_IP_MSK								(0x01UL << ECLIC_CLICINTIP_59_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_60_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_60_IP_MSK								(0x01UL << ECLIC_CLICINTIP_60_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_61_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_61_IP_MSK								(0x01UL << ECLIC_CLICINTIP_61_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_62_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_62_IP_MSK								(0x01UL << ECLIC_CLICINTIP_62_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_63_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_63_IP_MSK								(0x01UL << ECLIC_CLICINTIP_63_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_64_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_64_IP_MSK								(0x01UL << ECLIC_CLICINTIP_64_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_65_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_65_IP_MSK								(0x01UL << ECLIC_CLICINTIP_65_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_66_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_66_IP_MSK								(0x01UL << ECLIC_CLICINTIP_66_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_67_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_67_IP_MSK								(0x01UL << ECLIC_CLICINTIP_67_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_68_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_68_IP_MSK								(0x01UL << ECLIC_CLICINTIP_68_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_69_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_69_IP_MSK								(0x01UL << ECLIC_CLICINTIP_69_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_70_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_70_IP_MSK								(0x01UL << ECLIC_CLICINTIP_70_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_71_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_71_IP_MSK								(0x01UL << ECLIC_CLICINTIP_71_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_72_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_72_IP_MSK								(0x01UL << ECLIC_CLICINTIP_72_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_73_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_73_IP_MSK								(0x01UL << ECLIC_CLICINTIP_73_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_74_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_74_IP_MSK								(0x01UL << ECLIC_CLICINTIP_74_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_75_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_75_IP_MSK								(0x01UL << ECLIC_CLICINTIP_75_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_76_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_76_IP_MSK								(0x01UL << ECLIC_CLICINTIP_76_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_77_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_77_IP_MSK								(0x01UL << ECLIC_CLICINTIP_77_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_78_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_78_IP_MSK								(0x01UL << ECLIC_CLICINTIP_78_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_79_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_79_IP_MSK								(0x01UL << ECLIC_CLICINTIP_79_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_80_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_80_IP_MSK								(0x01UL << ECLIC_CLICINTIP_80_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_81_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_81_IP_MSK								(0x01UL << ECLIC_CLICINTIP_81_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_82_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_82_IP_MSK								(0x01UL << ECLIC_CLICINTIP_82_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_83_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_83_IP_MSK								(0x01UL << ECLIC_CLICINTIP_83_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_84_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_84_IP_MSK								(0x01UL << ECLIC_CLICINTIP_84_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_85_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_85_IP_MSK								(0x01UL << ECLIC_CLICINTIP_85_IP_POS)		/** IP */
#define ECLIC_CLICINTIP_86_IP_POS								0x00UL		/** IP */
#define ECLIC_CLICINTIP_86_IP_MSK								(0x01UL << ECLIC_CLICINTIP_86_IP_POS)		/** IP */
#define ECLIC_CLICINTIE_0_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_0_IE_MSK								(0x01UL << ECLIC_CLICINTIE_0_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_1_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_1_IE_MSK								(0x01UL << ECLIC_CLICINTIE_1_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_2_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_2_IE_MSK								(0x01UL << ECLIC_CLICINTIE_2_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_3_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_3_IE_MSK								(0x01UL << ECLIC_CLICINTIE_3_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_4_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_4_IE_MSK								(0x01UL << ECLIC_CLICINTIE_4_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_5_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_5_IE_MSK								(0x01UL << ECLIC_CLICINTIE_5_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_6_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_6_IE_MSK								(0x01UL << ECLIC_CLICINTIE_6_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_7_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_7_IE_MSK								(0x01UL << ECLIC_CLICINTIE_7_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_8_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_8_IE_MSK								(0x01UL << ECLIC_CLICINTIE_8_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_9_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_9_IE_MSK								(0x01UL << ECLIC_CLICINTIE_9_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_10_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_10_IE_MSK								(0x01UL << ECLIC_CLICINTIE_10_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_11_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_11_IE_MSK								(0x01UL << ECLIC_CLICINTIE_11_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_12_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_12_IE_MSK								(0x01UL << ECLIC_CLICINTIE_12_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_13_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_13_IE_MSK								(0x01UL << ECLIC_CLICINTIE_13_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_14_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_14_IE_MSK								(0x01UL << ECLIC_CLICINTIE_14_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_15_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_15_IE_MSK								(0x01UL << ECLIC_CLICINTIE_15_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_16_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_16_IE_MSK								(0x01UL << ECLIC_CLICINTIE_16_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_17_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_17_IE_MSK								(0x01UL << ECLIC_CLICINTIE_17_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_18_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_18_IE_MSK								(0x01UL << ECLIC_CLICINTIE_18_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_19_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_19_IE_MSK								(0x01UL << ECLIC_CLICINTIE_19_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_20_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_20_IE_MSK								(0x01UL << ECLIC_CLICINTIE_20_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_21_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_21_IE_MSK								(0x01UL << ECLIC_CLICINTIE_21_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_22_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_22_IE_MSK								(0x01UL << ECLIC_CLICINTIE_22_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_23_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_23_IE_MSK								(0x01UL << ECLIC_CLICINTIE_23_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_24_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_24_IE_MSK								(0x01UL << ECLIC_CLICINTIE_24_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_25_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_25_IE_MSK								(0x01UL << ECLIC_CLICINTIE_25_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_26_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_26_IE_MSK								(0x01UL << ECLIC_CLICINTIE_26_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_27_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_27_IE_MSK								(0x01UL << ECLIC_CLICINTIE_27_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_28_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_28_IE_MSK								(0x01UL << ECLIC_CLICINTIE_28_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_29_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_29_IE_MSK								(0x01UL << ECLIC_CLICINTIE_29_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_30_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_30_IE_MSK								(0x01UL << ECLIC_CLICINTIE_30_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_31_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_31_IE_MSK								(0x01UL << ECLIC_CLICINTIE_31_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_32_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_32_IE_MSK								(0x01UL << ECLIC_CLICINTIE_32_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_33_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_33_IE_MSK								(0x01UL << ECLIC_CLICINTIE_33_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_34_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_34_IE_MSK								(0x01UL << ECLIC_CLICINTIE_34_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_35_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_35_IE_MSK								(0x01UL << ECLIC_CLICINTIE_35_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_36_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_36_IE_MSK								(0x01UL << ECLIC_CLICINTIE_36_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_37_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_37_IE_MSK								(0x01UL << ECLIC_CLICINTIE_37_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_38_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_38_IE_MSK								(0x01UL << ECLIC_CLICINTIE_38_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_39_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_39_IE_MSK								(0x01UL << ECLIC_CLICINTIE_39_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_40_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_40_IE_MSK								(0x01UL << ECLIC_CLICINTIE_40_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_41_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_41_IE_MSK								(0x01UL << ECLIC_CLICINTIE_41_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_42_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_42_IE_MSK								(0x01UL << ECLIC_CLICINTIE_42_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_43_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_43_IE_MSK								(0x01UL << ECLIC_CLICINTIE_43_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_44_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_44_IE_MSK								(0x01UL << ECLIC_CLICINTIE_44_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_45_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_45_IE_MSK								(0x01UL << ECLIC_CLICINTIE_45_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_46_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_46_IE_MSK								(0x01UL << ECLIC_CLICINTIE_46_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_47_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_47_IE_MSK								(0x01UL << ECLIC_CLICINTIE_47_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_48_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_48_IE_MSK								(0x01UL << ECLIC_CLICINTIE_48_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_49_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_49_IE_MSK								(0x01UL << ECLIC_CLICINTIE_49_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_50_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_50_IE_MSK								(0x01UL << ECLIC_CLICINTIE_50_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_51_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_51_IE_MSK								(0x01UL << ECLIC_CLICINTIE_51_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_52_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_52_IE_MSK								(0x01UL << ECLIC_CLICINTIE_52_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_53_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_53_IE_MSK								(0x01UL << ECLIC_CLICINTIE_53_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_54_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_54_IE_MSK								(0x01UL << ECLIC_CLICINTIE_54_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_55_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_55_IE_MSK								(0x01UL << ECLIC_CLICINTIE_55_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_56_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_56_IE_MSK								(0x01UL << ECLIC_CLICINTIE_56_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_57_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_57_IE_MSK								(0x01UL << ECLIC_CLICINTIE_57_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_58_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_58_IE_MSK								(0x01UL << ECLIC_CLICINTIE_58_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_59_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_59_IE_MSK								(0x01UL << ECLIC_CLICINTIE_59_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_60_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_60_IE_MSK								(0x01UL << ECLIC_CLICINTIE_60_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_61_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_61_IE_MSK								(0x01UL << ECLIC_CLICINTIE_61_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_62_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_62_IE_MSK								(0x01UL << ECLIC_CLICINTIE_62_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_63_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_63_IE_MSK								(0x01UL << ECLIC_CLICINTIE_63_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_64_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_64_IE_MSK								(0x01UL << ECLIC_CLICINTIE_64_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_65_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_65_IE_MSK								(0x01UL << ECLIC_CLICINTIE_65_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_66_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_66_IE_MSK								(0x01UL << ECLIC_CLICINTIE_66_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_67_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_67_IE_MSK								(0x01UL << ECLIC_CLICINTIE_67_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_68_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_68_IE_MSK								(0x01UL << ECLIC_CLICINTIE_68_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_69_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_69_IE_MSK								(0x01UL << ECLIC_CLICINTIE_69_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_70_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_70_IE_MSK								(0x01UL << ECLIC_CLICINTIE_70_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_71_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_71_IE_MSK								(0x01UL << ECLIC_CLICINTIE_71_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_72_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_72_IE_MSK								(0x01UL << ECLIC_CLICINTIE_72_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_73_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_73_IE_MSK								(0x01UL << ECLIC_CLICINTIE_73_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_74_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_74_IE_MSK								(0x01UL << ECLIC_CLICINTIE_74_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_75_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_75_IE_MSK								(0x01UL << ECLIC_CLICINTIE_75_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_76_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_76_IE_MSK								(0x01UL << ECLIC_CLICINTIE_76_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_77_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_77_IE_MSK								(0x01UL << ECLIC_CLICINTIE_77_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_78_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_78_IE_MSK								(0x01UL << ECLIC_CLICINTIE_78_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_79_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_79_IE_MSK								(0x01UL << ECLIC_CLICINTIE_79_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_80_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_80_IE_MSK								(0x01UL << ECLIC_CLICINTIE_80_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_81_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_81_IE_MSK								(0x01UL << ECLIC_CLICINTIE_81_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_82_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_82_IE_MSK								(0x01UL << ECLIC_CLICINTIE_82_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_83_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_83_IE_MSK								(0x01UL << ECLIC_CLICINTIE_83_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_84_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_84_IE_MSK								(0x01UL << ECLIC_CLICINTIE_84_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_85_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_85_IE_MSK								(0x01UL << ECLIC_CLICINTIE_85_IE_POS)		/** IE */
#define ECLIC_CLICINTIE_86_IE_POS								0x00UL		/** IE */
#define ECLIC_CLICINTIE_86_IE_MSK								(0x01UL << ECLIC_CLICINTIE_86_IE_POS)		/** IE */
#define ECLIC_CLICINTATTR_0_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_0_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_0_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_0_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_0_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_0_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_1_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_1_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_1_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_1_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_1_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_1_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_2_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_2_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_2_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_2_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_2_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_2_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_3_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_3_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_3_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_3_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_3_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_3_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_4_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_4_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_4_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_4_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_4_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_4_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_5_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_5_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_5_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_5_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_5_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_5_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_6_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_6_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_6_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_6_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_6_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_6_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_7_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_7_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_7_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_7_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_7_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_7_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_8_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_8_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_8_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_8_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_8_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_8_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_9_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_9_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_9_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_9_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_9_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_9_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_10_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_10_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_10_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_10_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_10_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_10_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_11_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_11_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_11_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_11_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_11_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_11_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_12_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_12_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_12_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_12_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_12_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_12_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_13_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_13_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_13_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_13_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_13_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_13_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_14_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_14_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_14_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_14_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_14_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_14_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_15_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_15_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_15_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_15_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_15_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_15_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_16_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_16_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_16_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_16_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_16_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_16_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_17_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_17_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_17_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_17_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_17_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_17_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_18_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_18_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_18_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_18_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_18_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_18_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_19_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_19_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_19_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_19_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_19_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_19_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_20_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_20_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_20_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_20_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_20_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_20_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_21_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_21_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_21_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_21_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_21_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_21_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_22_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_22_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_22_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_22_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_22_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_22_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_23_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_23_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_23_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_23_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_23_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_23_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_24_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_24_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_24_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_24_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_24_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_24_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_25_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_25_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_25_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_25_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_25_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_25_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_26_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_26_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_26_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_26_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_26_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_26_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_27_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_27_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_27_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_27_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_27_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_27_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_28_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_28_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_28_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_28_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_28_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_28_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_29_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_29_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_29_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_29_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_29_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_29_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_30_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_30_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_30_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_30_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_30_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_30_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_31_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_31_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_31_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_31_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_31_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_31_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_32_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_32_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_32_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_32_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_32_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_32_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_33_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_33_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_33_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_33_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_33_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_33_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_34_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_34_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_34_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_34_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_34_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_34_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_35_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_35_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_35_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_35_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_35_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_35_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_36_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_36_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_36_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_36_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_36_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_36_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_37_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_37_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_37_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_37_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_37_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_37_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_38_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_38_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_38_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_38_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_38_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_38_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_39_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_39_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_39_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_39_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_39_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_39_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_40_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_40_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_40_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_40_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_40_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_40_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_41_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_41_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_41_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_41_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_41_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_41_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_42_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_42_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_42_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_42_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_42_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_42_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_43_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_43_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_43_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_43_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_43_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_43_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_44_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_44_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_44_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_44_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_44_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_44_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_45_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_45_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_45_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_45_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_45_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_45_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_46_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_46_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_46_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_46_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_46_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_46_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_47_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_47_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_47_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_47_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_47_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_47_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_48_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_48_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_48_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_48_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_48_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_48_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_49_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_49_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_49_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_49_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_49_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_49_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_50_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_50_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_50_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_50_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_50_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_50_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_51_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_51_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_51_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_51_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_51_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_51_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_52_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_52_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_52_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_52_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_52_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_52_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_53_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_53_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_53_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_53_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_53_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_53_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_54_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_54_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_54_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_54_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_54_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_54_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_55_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_55_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_55_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_55_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_55_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_55_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_56_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_56_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_56_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_56_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_56_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_56_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_57_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_57_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_57_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_57_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_57_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_57_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_58_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_58_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_58_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_58_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_58_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_58_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_59_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_59_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_59_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_59_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_59_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_59_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_60_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_60_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_60_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_60_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_60_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_60_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_61_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_61_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_61_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_61_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_61_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_61_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_62_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_62_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_62_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_62_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_62_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_62_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_63_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_63_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_63_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_63_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_63_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_63_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_64_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_64_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_64_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_64_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_64_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_64_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_65_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_65_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_65_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_65_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_65_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_65_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_66_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_66_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_66_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_66_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_66_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_66_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_67_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_67_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_67_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_67_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_67_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_67_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_68_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_68_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_68_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_68_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_68_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_68_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_69_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_69_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_69_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_69_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_69_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_69_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_70_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_70_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_70_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_70_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_70_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_70_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_71_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_71_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_71_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_71_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_71_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_71_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_72_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_72_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_72_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_72_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_72_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_72_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_73_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_73_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_73_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_73_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_73_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_73_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_74_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_74_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_74_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_74_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_74_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_74_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_75_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_75_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_75_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_75_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_75_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_75_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_76_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_76_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_76_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_76_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_76_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_76_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_77_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_77_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_77_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_77_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_77_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_77_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_78_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_78_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_78_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_78_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_78_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_78_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_79_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_79_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_79_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_79_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_79_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_79_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_80_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_80_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_80_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_80_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_80_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_80_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_81_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_81_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_81_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_81_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_81_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_81_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_82_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_82_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_82_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_82_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_82_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_82_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_83_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_83_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_83_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_83_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_83_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_83_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_84_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_84_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_84_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_84_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_84_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_84_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_85_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_85_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_85_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_85_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_85_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_85_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTATTR_86_SHV_POS								0x00UL		/** SHV */
#define ECLIC_CLICINTATTR_86_SHV_MSK								(0x01UL << ECLIC_CLICINTATTR_86_SHV_POS)		/** SHV */
#define ECLIC_CLICINTATTR_86_TRIG_POS								0x01UL		/** TRIG */
#define ECLIC_CLICINTATTR_86_TRIG_MSK								(0x03UL << ECLIC_CLICINTATTR_86_TRIG_POS)		/** TRIG */
#define ECLIC_CLICINTCTL_0_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_0_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_0_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_1_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_1_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_1_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_2_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_2_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_2_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_3_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_3_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_3_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_4_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_4_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_4_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_5_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_5_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_5_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_6_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_6_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_6_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_7_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_7_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_7_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_8_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_8_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_8_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_9_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_9_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_9_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_10_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_10_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_10_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_11_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_11_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_11_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_12_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_12_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_12_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_13_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_13_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_13_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_14_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_14_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_14_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_15_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_15_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_15_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_16_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_16_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_16_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_17_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_17_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_17_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_18_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_18_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_18_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_19_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_19_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_19_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_20_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_20_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_20_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_21_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_21_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_21_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_22_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_22_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_22_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_23_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_23_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_23_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_24_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_24_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_24_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_25_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_25_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_25_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_26_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_26_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_26_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_27_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_27_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_27_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_28_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_28_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_28_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_29_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_29_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_29_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_30_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_30_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_30_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_31_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_31_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_31_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_32_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_32_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_32_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_33_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_33_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_33_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_34_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_34_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_34_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_35_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_35_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_35_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_36_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_36_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_36_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_37_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_37_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_37_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_38_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_38_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_38_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_39_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_39_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_39_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_40_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_40_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_40_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_41_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_41_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_41_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_42_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_42_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_42_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_43_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_43_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_43_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_44_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_44_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_44_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_45_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_45_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_45_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_46_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_46_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_46_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_47_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_47_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_47_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_48_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_48_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_48_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_49_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_49_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_49_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_50_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_50_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_50_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_51_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_51_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_51_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_52_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_52_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_52_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_53_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_53_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_53_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_54_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_54_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_54_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_55_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_55_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_55_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_56_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_56_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_56_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_57_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_57_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_57_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_58_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_58_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_58_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_59_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_59_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_59_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_60_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_60_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_60_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_61_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_61_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_61_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_62_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_62_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_62_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_63_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_63_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_63_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_64_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_64_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_64_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_65_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_65_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_65_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_66_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_66_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_66_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_67_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_67_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_67_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_68_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_68_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_68_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_69_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_69_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_69_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_70_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_70_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_70_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_71_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_71_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_71_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_72_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_72_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_72_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_73_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_73_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_73_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_74_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_74_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_74_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_75_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_75_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_75_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_76_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_76_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_76_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_77_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_77_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_77_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_78_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_78_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_78_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_79_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_79_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_79_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_80_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_80_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_80_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_81_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_81_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_81_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_82_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_82_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_82_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_83_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_83_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_83_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_84_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_84_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_84_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_85_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_85_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_85_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_86_LEVEL_PRIORITY_POS								0x00UL		/** LEVEL_PRIORITY */
#define ECLIC_CLICINTCTL_86_LEVEL_PRIORITY_MSK								(0xFFUL << ECLIC_CLICINTCTL_86_LEVEL_PRIORITY_POS)		/** LEVEL_PRIORITY */
#define PMU_CTL_BKPWEN_POS								0x08UL		/** Backup Domain Write Enable */
#define PMU_CTL_BKPWEN_MSK								(0x01UL << PMU_CTL_BKPWEN_POS)		/** Backup Domain Write Enable */
#define PMU_CTL_LVDT_POS								0x05UL		/** Low Voltage Detector Threshold */
#define PMU_CTL_LVDT_MSK								(0x07UL << PMU_CTL_LVDT_POS)		/** Low Voltage Detector Threshold */
#define PMU_CTL_LVDEN_POS								0x04UL		/** Low Voltage Detector Enable */
#define PMU_CTL_LVDEN_MSK								(0x01UL << PMU_CTL_LVDEN_POS)		/** Low Voltage Detector Enable */
#define PMU_CTL_STBRST_POS								0x03UL		/** Standby Flag Reset */
#define PMU_CTL_STBRST_MSK								(0x01UL << PMU_CTL_STBRST_POS)		/** Standby Flag Reset */
#define PMU_CTL_WURST_POS								0x02UL		/** Wakeup Flag Reset */
#define PMU_CTL_WURST_MSK								(0x01UL << PMU_CTL_WURST_POS)		/** Wakeup Flag Reset */
#define PMU_CTL_STBMOD_POS								0x01UL		/** Standby Mode */
#define PMU_CTL_STBMOD_MSK								(0x01UL << PMU_CTL_STBMOD_POS)		/** Standby Mode */
#define PMU_CTL_LDOLP_POS								0x00UL		/** LDO Low Power Mode */
#define PMU_CTL_LDOLP_MSK								(0x01UL << PMU_CTL_LDOLP_POS)		/** LDO Low Power Mode */
#define PMU_CS_WUPEN_POS								0x08UL		/** Enable WKUP pin */
#define PMU_CS_WUPEN_MSK								(0x01UL << PMU_CS_WUPEN_POS)		/** Enable WKUP pin */
#define PMU_CS_LVDF_POS								0x02UL		/** Low Voltage Detector Status Flag */
#define PMU_CS_LVDF_MSK								(0x01UL << PMU_CS_LVDF_POS)		/** Low Voltage Detector Status Flag */
#define PMU_CS_STBF_POS								0x01UL		/** Standby flag */
#define PMU_CS_STBF_MSK								(0x01UL << PMU_CS_STBF_POS)		/** Standby flag */
#define PMU_CS_WUF_POS								0x00UL		/** Wakeup flag */
#define PMU_CS_WUF_MSK								(0x01UL << PMU_CS_WUF_POS)		/** Wakeup flag */
#define RCU_CTL_IRC8MEN_POS								0x00UL		/** Internal 8MHz RC oscillator Enable */
#define RCU_CTL_IRC8MEN_MSK								(0x01UL << RCU_CTL_IRC8MEN_POS)		/** Internal 8MHz RC oscillator Enable */
#define RCU_CTL_IRC8MSTB_POS								0x01UL		/** IRC8M Internal 8MHz RC Oscillator stabilization Flag */
#define RCU_CTL_IRC8MSTB_MSK								(0x01UL << RCU_CTL_IRC8MSTB_POS)		/** IRC8M Internal 8MHz RC Oscillator stabilization Flag */
#define RCU_CTL_IRC8MADJ_POS								0x03UL		/** Internal 8MHz RC Oscillator clock trim adjust value */
#define RCU_CTL_IRC8MADJ_MSK								(0x1FUL << RCU_CTL_IRC8MADJ_POS)		/** Internal 8MHz RC Oscillator clock trim adjust value */
#define RCU_CTL_IRC8MCALIB_POS								0x08UL		/** Internal 8MHz RC Oscillator calibration value register */
#define RCU_CTL_IRC8MCALIB_MSK								(0xFFUL << RCU_CTL_IRC8MCALIB_POS)		/** Internal 8MHz RC Oscillator calibration value register */
#define RCU_CTL_HXTALEN_POS								0x10UL		/** External High Speed oscillator Enable */
#define RCU_CTL_HXTALEN_MSK								(0x01UL << RCU_CTL_HXTALEN_POS)		/** External High Speed oscillator Enable */
#define RCU_CTL_HXTALSTB_POS								0x11UL		/** External crystal oscillator (HXTAL) clock stabilization flag */
#define RCU_CTL_HXTALSTB_MSK								(0x01UL << RCU_CTL_HXTALSTB_POS)		/** External crystal oscillator (HXTAL) clock stabilization flag */
#define RCU_CTL_HXTALBPS_POS								0x12UL		/** External crystal oscillator (HXTAL) clock bypass mode enable */
#define RCU_CTL_HXTALBPS_MSK								(0x01UL << RCU_CTL_HXTALBPS_POS)		/** External crystal oscillator (HXTAL) clock bypass mode enable */
#define RCU_CTL_CKMEN_POS								0x13UL		/** HXTAL Clock Monitor Enable */
#define RCU_CTL_CKMEN_MSK								(0x01UL << RCU_CTL_CKMEN_POS)		/** HXTAL Clock Monitor Enable */
#define RCU_CTL_PLLEN_POS								0x18UL		/** PLL enable */
#define RCU_CTL_PLLEN_MSK								(0x01UL << RCU_CTL_PLLEN_POS)		/** PLL enable */
#define RCU_CTL_PLLSTB_POS								0x19UL		/** PLL Clock Stabilization Flag */
#define RCU_CTL_PLLSTB_MSK								(0x01UL << RCU_CTL_PLLSTB_POS)		/** PLL Clock Stabilization Flag */
#define RCU_CTL_PLL1EN_POS								0x1AUL		/** PLL1 enable */
#define RCU_CTL_PLL1EN_MSK								(0x01UL << RCU_CTL_PLL1EN_POS)		/** PLL1 enable */
#define RCU_CTL_PLL1STB_POS								0x1BUL		/** PLL1 Clock Stabilization Flag */
#define RCU_CTL_PLL1STB_MSK								(0x01UL << RCU_CTL_PLL1STB_POS)		/** PLL1 Clock Stabilization Flag */
#define RCU_CTL_PLL2EN_POS								0x1CUL		/** PLL2 enable */
#define RCU_CTL_PLL2EN_MSK								(0x01UL << RCU_CTL_PLL2EN_POS)		/** PLL2 enable */
#define RCU_CTL_PLL2STB_POS								0x1DUL		/** PLL2 Clock Stabilization Flag */
#define RCU_CTL_PLL2STB_MSK								(0x01UL << RCU_CTL_PLL2STB_POS)		/** PLL2 Clock Stabilization Flag */
#define RCU_CFG0_SCS_POS								0x00UL		/** System clock switch */
#define RCU_CFG0_SCS_MSK								(0x03UL << RCU_CFG0_SCS_POS)		/** System clock switch */
#define RCU_CFG0_SCSS_POS								0x02UL		/** System clock switch status */
#define RCU_CFG0_SCSS_MSK								(0x03UL << RCU_CFG0_SCSS_POS)		/** System clock switch status */
#define RCU_CFG0_AHBPSC_POS								0x04UL		/** AHB prescaler selection */
#define RCU_CFG0_AHBPSC_MSK								(0x0FUL << RCU_CFG0_AHBPSC_POS)		/** AHB prescaler selection */
#define RCU_CFG0_APB1PSC_POS								0x08UL		/** APB1 prescaler selection */
#define RCU_CFG0_APB1PSC_MSK								(0x07UL << RCU_CFG0_APB1PSC_POS)		/** APB1 prescaler selection */
#define RCU_CFG0_APB2PSC_POS								0x0BUL		/** APB2 prescaler selection */
#define RCU_CFG0_APB2PSC_MSK								(0x07UL << RCU_CFG0_APB2PSC_POS)		/** APB2 prescaler selection */
#define RCU_CFG0_ADCPSC_1_0_POS								0x0EUL		/** ADC clock prescaler selection */
#define RCU_CFG0_ADCPSC_1_0_MSK								(0x03UL << RCU_CFG0_ADCPSC_1_0_POS)		/** ADC clock prescaler selection */
#define RCU_CFG0_PLLSEL_POS								0x10UL		/** PLL Clock Source Selection */
#define RCU_CFG0_PLLSEL_MSK								(0x01UL << RCU_CFG0_PLLSEL_POS)		/** PLL Clock Source Selection */
#define RCU_CFG0_PREDV0_LSB_POS								0x11UL		/** The LSB of PREDV0 division factor */
#define RCU_CFG0_PREDV0_LSB_MSK								(0x01UL << RCU_CFG0_PREDV0_LSB_POS)		/** The LSB of PREDV0 division factor */
#define RCU_CFG0_PLLMF_3_0_POS								0x12UL		/** The PLL clock multiplication factor */
#define RCU_CFG0_PLLMF_3_0_MSK								(0x0FUL << RCU_CFG0_PLLMF_3_0_POS)		/** The PLL clock multiplication factor */
#define RCU_CFG0_USBFSPSC_POS								0x16UL		/** USBFS clock prescaler selection */
#define RCU_CFG0_USBFSPSC_MSK								(0x03UL << RCU_CFG0_USBFSPSC_POS)		/** USBFS clock prescaler selection */
#define RCU_CFG0_CKOUT0SEL_POS								0x18UL		/** CKOUT0 Clock Source Selection */
#define RCU_CFG0_CKOUT0SEL_MSK								(0x0FUL << RCU_CFG0_CKOUT0SEL_POS)		/** CKOUT0 Clock Source Selection */
#define RCU_CFG0_ADCPSC_2_POS								0x1CUL		/** Bit 2 of ADCPSC */
#define RCU_CFG0_ADCPSC_2_MSK								(0x01UL << RCU_CFG0_ADCPSC_2_POS)		/** Bit 2 of ADCPSC */
#define RCU_CFG0_PLLMF_4_POS								0x1DUL		/** Bit 4 of PLLMF */
#define RCU_CFG0_PLLMF_4_MSK								(0x01UL << RCU_CFG0_PLLMF_4_POS)		/** Bit 4 of PLLMF */
#define RCU_INT_IRC40KSTBIF_POS								0x00UL		/** IRC40K stabilization interrupt flag */
#define RCU_INT_IRC40KSTBIF_MSK								(0x01UL << RCU_INT_IRC40KSTBIF_POS)		/** IRC40K stabilization interrupt flag */
#define RCU_INT_LXTALSTBIF_POS								0x01UL		/** LXTAL stabilization interrupt flag */
#define RCU_INT_LXTALSTBIF_MSK								(0x01UL << RCU_INT_LXTALSTBIF_POS)		/** LXTAL stabilization interrupt flag */
#define RCU_INT_IRC8MSTBIF_POS								0x02UL		/** IRC8M stabilization interrupt flag */
#define RCU_INT_IRC8MSTBIF_MSK								(0x01UL << RCU_INT_IRC8MSTBIF_POS)		/** IRC8M stabilization interrupt flag */
#define RCU_INT_HXTALSTBIF_POS								0x03UL		/** HXTAL stabilization interrupt flag */
#define RCU_INT_HXTALSTBIF_MSK								(0x01UL << RCU_INT_HXTALSTBIF_POS)		/** HXTAL stabilization interrupt flag */
#define RCU_INT_PLLSTBIF_POS								0x04UL		/** PLL stabilization interrupt flag */
#define RCU_INT_PLLSTBIF_MSK								(0x01UL << RCU_INT_PLLSTBIF_POS)		/** PLL stabilization interrupt flag */
#define RCU_INT_PLL1STBIF_POS								0x05UL		/** PLL1 stabilization interrupt flag */
#define RCU_INT_PLL1STBIF_MSK								(0x01UL << RCU_INT_PLL1STBIF_POS)		/** PLL1 stabilization interrupt flag */
#define RCU_INT_PLL2STBIF_POS								0x06UL		/** PLL2 stabilization interrupt flag */
#define RCU_INT_PLL2STBIF_MSK								(0x01UL << RCU_INT_PLL2STBIF_POS)		/** PLL2 stabilization interrupt flag */
#define RCU_INT_CKMIF_POS								0x07UL		/** HXTAL Clock Stuck Interrupt Flag */
#define RCU_INT_CKMIF_MSK								(0x01UL << RCU_INT_CKMIF_POS)		/** HXTAL Clock Stuck Interrupt Flag */
#define RCU_INT_IRC40KSTBIE_POS								0x08UL		/** IRC40K Stabilization interrupt enable */
#define RCU_INT_IRC40KSTBIE_MSK								(0x01UL << RCU_INT_IRC40KSTBIE_POS)		/** IRC40K Stabilization interrupt enable */
#define RCU_INT_LXTALSTBIE_POS								0x09UL		/** LXTAL Stabilization Interrupt Enable */
#define RCU_INT_LXTALSTBIE_MSK								(0x01UL << RCU_INT_LXTALSTBIE_POS)		/** LXTAL Stabilization Interrupt Enable */
#define RCU_INT_IRC8MSTBIE_POS								0x0AUL		/** IRC8M Stabilization Interrupt Enable */
#define RCU_INT_IRC8MSTBIE_MSK								(0x01UL << RCU_INT_IRC8MSTBIE_POS)		/** IRC8M Stabilization Interrupt Enable */
#define RCU_INT_HXTALSTBIE_POS								0x0BUL		/** HXTAL Stabilization Interrupt Enable */
#define RCU_INT_HXTALSTBIE_MSK								(0x01UL << RCU_INT_HXTALSTBIE_POS)		/** HXTAL Stabilization Interrupt Enable */
#define RCU_INT_PLLSTBIE_POS								0x0CUL		/** PLL Stabilization Interrupt Enable */
#define RCU_INT_PLLSTBIE_MSK								(0x01UL << RCU_INT_PLLSTBIE_POS)		/** PLL Stabilization Interrupt Enable */
#define RCU_INT_PLL1STBIE_POS								0x0DUL		/** PLL1 Stabilization Interrupt Enable */
#define RCU_INT_PLL1STBIE_MSK								(0x01UL << RCU_INT_PLL1STBIE_POS)		/** PLL1 Stabilization Interrupt Enable */
#define RCU_INT_PLL2STBIE_POS								0x0EUL		/** PLL2 Stabilization Interrupt Enable */
#define RCU_INT_PLL2STBIE_MSK								(0x01UL << RCU_INT_PLL2STBIE_POS)		/** PLL2 Stabilization Interrupt Enable */
#define RCU_INT_IRC40KSTBIC_POS								0x10UL		/** IRC40K Stabilization Interrupt Clear */
#define RCU_INT_IRC40KSTBIC_MSK								(0x01UL << RCU_INT_IRC40KSTBIC_POS)		/** IRC40K Stabilization Interrupt Clear */
#define RCU_INT_LXTALSTBIC_POS								0x11UL		/** LXTAL Stabilization Interrupt Clear */
#define RCU_INT_LXTALSTBIC_MSK								(0x01UL << RCU_INT_LXTALSTBIC_POS)		/** LXTAL Stabilization Interrupt Clear */
#define RCU_INT_IRC8MSTBIC_POS								0x12UL		/** IRC8M Stabilization Interrupt Clear */
#define RCU_INT_IRC8MSTBIC_MSK								(0x01UL << RCU_INT_IRC8MSTBIC_POS)		/** IRC8M Stabilization Interrupt Clear */
#define RCU_INT_HXTALSTBIC_POS								0x13UL		/** HXTAL Stabilization Interrupt Clear */
#define RCU_INT_HXTALSTBIC_MSK								(0x01UL << RCU_INT_HXTALSTBIC_POS)		/** HXTAL Stabilization Interrupt Clear */
#define RCU_INT_PLLSTBIC_POS								0x14UL		/** PLL stabilization Interrupt Clear */
#define RCU_INT_PLLSTBIC_MSK								(0x01UL << RCU_INT_PLLSTBIC_POS)		/** PLL stabilization Interrupt Clear */
#define RCU_INT_PLL1STBIC_POS								0x15UL		/** PLL1 stabilization Interrupt Clear */
#define RCU_INT_PLL1STBIC_MSK								(0x01UL << RCU_INT_PLL1STBIC_POS)		/** PLL1 stabilization Interrupt Clear */
#define RCU_INT_PLL2STBIC_POS								0x16UL		/** PLL2 stabilization Interrupt Clear */
#define RCU_INT_PLL2STBIC_MSK								(0x01UL << RCU_INT_PLL2STBIC_POS)		/** PLL2 stabilization Interrupt Clear */
#define RCU_INT_CKMIC_POS								0x17UL		/** HXTAL Clock Stuck Interrupt Clear */
#define RCU_INT_CKMIC_MSK								(0x01UL << RCU_INT_CKMIC_POS)		/** HXTAL Clock Stuck Interrupt Clear */
#define RCU_APB2RST_AFRST_POS								0x00UL		/** Alternate function I/O reset */
#define RCU_APB2RST_AFRST_MSK								(0x01UL << RCU_APB2RST_AFRST_POS)		/** Alternate function I/O reset */
#define RCU_APB2RST_PARST_POS								0x02UL		/** GPIO port A reset */
#define RCU_APB2RST_PARST_MSK								(0x01UL << RCU_APB2RST_PARST_POS)		/** GPIO port A reset */
#define RCU_APB2RST_PBRST_POS								0x03UL		/** GPIO port B reset */
#define RCU_APB2RST_PBRST_MSK								(0x01UL << RCU_APB2RST_PBRST_POS)		/** GPIO port B reset */
#define RCU_APB2RST_PCRST_POS								0x04UL		/** GPIO port C reset */
#define RCU_APB2RST_PCRST_MSK								(0x01UL << RCU_APB2RST_PCRST_POS)		/** GPIO port C reset */
#define RCU_APB2RST_PDRST_POS								0x05UL		/** GPIO port D reset */
#define RCU_APB2RST_PDRST_MSK								(0x01UL << RCU_APB2RST_PDRST_POS)		/** GPIO port D reset */
#define RCU_APB2RST_PERST_POS								0x06UL		/** GPIO port E reset */
#define RCU_APB2RST_PERST_MSK								(0x01UL << RCU_APB2RST_PERST_POS)		/** GPIO port E reset */
#define RCU_APB2RST_ADC0RST_POS								0x09UL		/** ADC0 reset */
#define RCU_APB2RST_ADC0RST_MSK								(0x01UL << RCU_APB2RST_ADC0RST_POS)		/** ADC0 reset */
#define RCU_APB2RST_ADC1RST_POS								0x0AUL		/** ADC1 reset */
#define RCU_APB2RST_ADC1RST_MSK								(0x01UL << RCU_APB2RST_ADC1RST_POS)		/** ADC1 reset */
#define RCU_APB2RST_TIMER0RST_POS								0x0BUL		/** Timer 0 reset */
#define RCU_APB2RST_TIMER0RST_MSK								(0x01UL << RCU_APB2RST_TIMER0RST_POS)		/** Timer 0 reset */
#define RCU_APB2RST_SPI0RST_POS								0x0CUL		/** SPI0 reset */
#define RCU_APB2RST_SPI0RST_MSK								(0x01UL << RCU_APB2RST_SPI0RST_POS)		/** SPI0 reset */
#define RCU_APB2RST_USART0RST_POS								0x0EUL		/** USART0 Reset */
#define RCU_APB2RST_USART0RST_MSK								(0x01UL << RCU_APB2RST_USART0RST_POS)		/** USART0 Reset */
#define RCU_APB1RST_TIMER1RST_POS								0x00UL		/** TIMER1 timer reset */
#define RCU_APB1RST_TIMER1RST_MSK								(0x01UL << RCU_APB1RST_TIMER1RST_POS)		/** TIMER1 timer reset */
#define RCU_APB1RST_TIMER2RST_POS								0x01UL		/** TIMER2 timer reset */
#define RCU_APB1RST_TIMER2RST_MSK								(0x01UL << RCU_APB1RST_TIMER2RST_POS)		/** TIMER2 timer reset */
#define RCU_APB1RST_TIMER3RST_POS								0x02UL		/** TIMER3 timer reset */
#define RCU_APB1RST_TIMER3RST_MSK								(0x01UL << RCU_APB1RST_TIMER3RST_POS)		/** TIMER3 timer reset */
#define RCU_APB1RST_TIMER4RST_POS								0x03UL		/** TIMER4 timer reset */
#define RCU_APB1RST_TIMER4RST_MSK								(0x01UL << RCU_APB1RST_TIMER4RST_POS)		/** TIMER4 timer reset */
#define RCU_APB1RST_TIMER5RST_POS								0x04UL		/** TIMER5 timer reset */
#define RCU_APB1RST_TIMER5RST_MSK								(0x01UL << RCU_APB1RST_TIMER5RST_POS)		/** TIMER5 timer reset */
#define RCU_APB1RST_TIMER6RST_POS								0x05UL		/** TIMER6 timer reset */
#define RCU_APB1RST_TIMER6RST_MSK								(0x01UL << RCU_APB1RST_TIMER6RST_POS)		/** TIMER6 timer reset */
#define RCU_APB1RST_WWDGTRST_POS								0x0BUL		/** Window watchdog timer reset */
#define RCU_APB1RST_WWDGTRST_MSK								(0x01UL << RCU_APB1RST_WWDGTRST_POS)		/** Window watchdog timer reset */
#define RCU_APB1RST_SPI1RST_POS								0x0EUL		/** SPI1 reset */
#define RCU_APB1RST_SPI1RST_MSK								(0x01UL << RCU_APB1RST_SPI1RST_POS)		/** SPI1 reset */
#define RCU_APB1RST_SPI2RST_POS								0x0FUL		/** SPI2 reset */
#define RCU_APB1RST_SPI2RST_MSK								(0x01UL << RCU_APB1RST_SPI2RST_POS)		/** SPI2 reset */
#define RCU_APB1RST_USART1RST_POS								0x11UL		/** USART1 reset */
#define RCU_APB1RST_USART1RST_MSK								(0x01UL << RCU_APB1RST_USART1RST_POS)		/** USART1 reset */
#define RCU_APB1RST_USART2RST_POS								0x12UL		/** USART2 reset */
#define RCU_APB1RST_USART2RST_MSK								(0x01UL << RCU_APB1RST_USART2RST_POS)		/** USART2 reset */
#define RCU_APB1RST_UART3RST_POS								0x13UL		/** UART3 reset */
#define RCU_APB1RST_UART3RST_MSK								(0x01UL << RCU_APB1RST_UART3RST_POS)		/** UART3 reset */
#define RCU_APB1RST_UART4RST_POS								0x14UL		/** UART4 reset */
#define RCU_APB1RST_UART4RST_MSK								(0x01UL << RCU_APB1RST_UART4RST_POS)		/** UART4 reset */
#define RCU_APB1RST_I2C0RST_POS								0x15UL		/** I2C0 reset */
#define RCU_APB1RST_I2C0RST_MSK								(0x01UL << RCU_APB1RST_I2C0RST_POS)		/** I2C0 reset */
#define RCU_APB1RST_I2C1RST_POS								0x16UL		/** I2C1 reset */
#define RCU_APB1RST_I2C1RST_MSK								(0x01UL << RCU_APB1RST_I2C1RST_POS)		/** I2C1 reset */
#define RCU_APB1RST_CAN0RST_POS								0x19UL		/** CAN0 reset */
#define RCU_APB1RST_CAN0RST_MSK								(0x01UL << RCU_APB1RST_CAN0RST_POS)		/** CAN0 reset */
#define RCU_APB1RST_CAN1RST_POS								0x1AUL		/** CAN1 reset */
#define RCU_APB1RST_CAN1RST_MSK								(0x01UL << RCU_APB1RST_CAN1RST_POS)		/** CAN1 reset */
#define RCU_APB1RST_BKPIRST_POS								0x1BUL		/** Backup interface reset */
#define RCU_APB1RST_BKPIRST_MSK								(0x01UL << RCU_APB1RST_BKPIRST_POS)		/** Backup interface reset */
#define RCU_APB1RST_PMURST_POS								0x1CUL		/** Power control reset */
#define RCU_APB1RST_PMURST_MSK								(0x01UL << RCU_APB1RST_PMURST_POS)		/** Power control reset */
#define RCU_APB1RST_DACRST_POS								0x1DUL		/** DAC reset */
#define RCU_APB1RST_DACRST_MSK								(0x01UL << RCU_APB1RST_DACRST_POS)		/** DAC reset */
#define RCU_AHBEN_DMA0EN_POS								0x00UL		/** DMA0 clock enable */
#define RCU_AHBEN_DMA0EN_MSK								(0x01UL << RCU_AHBEN_DMA0EN_POS)		/** DMA0 clock enable */
#define RCU_AHBEN_DMA1EN_POS								0x01UL		/** DMA1 clock enable */
#define RCU_AHBEN_DMA1EN_MSK								(0x01UL << RCU_AHBEN_DMA1EN_POS)		/** DMA1 clock enable */
#define RCU_AHBEN_SRAMSPEN_POS								0x02UL		/** SRAM interface clock enable when sleep mode */
#define RCU_AHBEN_SRAMSPEN_MSK								(0x01UL << RCU_AHBEN_SRAMSPEN_POS)		/** SRAM interface clock enable when sleep mode */
#define RCU_AHBEN_FMCSPEN_POS								0x04UL		/** FMC clock enable when sleep mode */
#define RCU_AHBEN_FMCSPEN_MSK								(0x01UL << RCU_AHBEN_FMCSPEN_POS)		/** FMC clock enable when sleep mode */
#define RCU_AHBEN_CRCEN_POS								0x06UL		/** CRC clock enable */
#define RCU_AHBEN_CRCEN_MSK								(0x01UL << RCU_AHBEN_CRCEN_POS)		/** CRC clock enable */
#define RCU_AHBEN_EXMCEN_POS								0x08UL		/** EXMC clock enable */
#define RCU_AHBEN_EXMCEN_MSK								(0x01UL << RCU_AHBEN_EXMCEN_POS)		/** EXMC clock enable */
#define RCU_AHBEN_USBFSEN_POS								0x0CUL		/** USBFS clock enable */
#define RCU_AHBEN_USBFSEN_MSK								(0x01UL << RCU_AHBEN_USBFSEN_POS)		/** USBFS clock enable */
#define RCU_APB2EN_AFEN_POS								0x00UL		/** Alternate function IO clock enable  */
#define RCU_APB2EN_AFEN_MSK								(0x01UL << RCU_APB2EN_AFEN_POS)		/** Alternate function IO clock enable  */
#define RCU_APB2EN_PAEN_POS								0x02UL		/** GPIO port A clock enable */
#define RCU_APB2EN_PAEN_MSK								(0x01UL << RCU_APB2EN_PAEN_POS)		/** GPIO port A clock enable */
#define RCU_APB2EN_PBEN_POS								0x03UL		/** GPIO port B clock enable */
#define RCU_APB2EN_PBEN_MSK								(0x01UL << RCU_APB2EN_PBEN_POS)		/** GPIO port B clock enable */
#define RCU_APB2EN_PCEN_POS								0x04UL		/** GPIO port C clock enable */
#define RCU_APB2EN_PCEN_MSK								(0x01UL << RCU_APB2EN_PCEN_POS)		/** GPIO port C clock enable */
#define RCU_APB2EN_PDEN_POS								0x05UL		/** GPIO port D clock enable  */
#define RCU_APB2EN_PDEN_MSK								(0x01UL << RCU_APB2EN_PDEN_POS)		/** GPIO port D clock enable  */
#define RCU_APB2EN_PEEN_POS								0x06UL		/** GPIO port E clock enable  */
#define RCU_APB2EN_PEEN_MSK								(0x01UL << RCU_APB2EN_PEEN_POS)		/** GPIO port E clock enable  */
#define RCU_APB2EN_ADC0EN_POS								0x09UL		/** ADC0 clock enable */
#define RCU_APB2EN_ADC0EN_MSK								(0x01UL << RCU_APB2EN_ADC0EN_POS)		/** ADC0 clock enable */
#define RCU_APB2EN_ADC1EN_POS								0x0AUL		/** ADC1 clock enable */
#define RCU_APB2EN_ADC1EN_MSK								(0x01UL << RCU_APB2EN_ADC1EN_POS)		/** ADC1 clock enable */
#define RCU_APB2EN_TIMER0EN_POS								0x0BUL		/** TIMER0 clock enable  */
#define RCU_APB2EN_TIMER0EN_MSK								(0x01UL << RCU_APB2EN_TIMER0EN_POS)		/** TIMER0 clock enable  */
#define RCU_APB2EN_SPI0EN_POS								0x0CUL		/** SPI0 clock enable */
#define RCU_APB2EN_SPI0EN_MSK								(0x01UL << RCU_APB2EN_SPI0EN_POS)		/** SPI0 clock enable */
#define RCU_APB2EN_USART0EN_POS								0x0EUL		/** USART0 clock enable */
#define RCU_APB2EN_USART0EN_MSK								(0x01UL << RCU_APB2EN_USART0EN_POS)		/** USART0 clock enable */
#define RCU_APB1EN_TIMER1EN_POS								0x00UL		/** TIMER1 timer clock enable */
#define RCU_APB1EN_TIMER1EN_MSK								(0x01UL << RCU_APB1EN_TIMER1EN_POS)		/** TIMER1 timer clock enable */
#define RCU_APB1EN_TIMER2EN_POS								0x01UL		/** TIMER2 timer clock enable */
#define RCU_APB1EN_TIMER2EN_MSK								(0x01UL << RCU_APB1EN_TIMER2EN_POS)		/** TIMER2 timer clock enable */
#define RCU_APB1EN_TIMER3EN_POS								0x02UL		/** TIMER3 timer clock enable */
#define RCU_APB1EN_TIMER3EN_MSK								(0x01UL << RCU_APB1EN_TIMER3EN_POS)		/** TIMER3 timer clock enable */
#define RCU_APB1EN_TIMER4EN_POS								0x03UL		/** TIMER4 timer clock enable */
#define RCU_APB1EN_TIMER4EN_MSK								(0x01UL << RCU_APB1EN_TIMER4EN_POS)		/** TIMER4 timer clock enable */
#define RCU_APB1EN_TIMER5EN_POS								0x04UL		/** TIMER5 timer clock enable */
#define RCU_APB1EN_TIMER5EN_MSK								(0x01UL << RCU_APB1EN_TIMER5EN_POS)		/** TIMER5 timer clock enable */
#define RCU_APB1EN_TIMER6EN_POS								0x05UL		/** TIMER6 timer clock enable */
#define RCU_APB1EN_TIMER6EN_MSK								(0x01UL << RCU_APB1EN_TIMER6EN_POS)		/** TIMER6 timer clock enable */
#define RCU_APB1EN_WWDGTEN_POS								0x0BUL		/** Window watchdog timer clock enable */
#define RCU_APB1EN_WWDGTEN_MSK								(0x01UL << RCU_APB1EN_WWDGTEN_POS)		/** Window watchdog timer clock enable */
#define RCU_APB1EN_SPI1EN_POS								0x0EUL		/** SPI1 clock enable */
#define RCU_APB1EN_SPI1EN_MSK								(0x01UL << RCU_APB1EN_SPI1EN_POS)		/** SPI1 clock enable */
#define RCU_APB1EN_SPI2EN_POS								0x0FUL		/** SPI2 clock enable */
#define RCU_APB1EN_SPI2EN_MSK								(0x01UL << RCU_APB1EN_SPI2EN_POS)		/** SPI2 clock enable */
#define RCU_APB1EN_USART1EN_POS								0x11UL		/** USART1 clock enable */
#define RCU_APB1EN_USART1EN_MSK								(0x01UL << RCU_APB1EN_USART1EN_POS)		/** USART1 clock enable */
#define RCU_APB1EN_USART2EN_POS								0x12UL		/** USART2 clock enable */
#define RCU_APB1EN_USART2EN_MSK								(0x01UL << RCU_APB1EN_USART2EN_POS)		/** USART2 clock enable */
#define RCU_APB1EN_UART3EN_POS								0x13UL		/** UART3 clock enable */
#define RCU_APB1EN_UART3EN_MSK								(0x01UL << RCU_APB1EN_UART3EN_POS)		/** UART3 clock enable */
#define RCU_APB1EN_UART4EN_POS								0x14UL		/** UART4 clock enable */
#define RCU_APB1EN_UART4EN_MSK								(0x01UL << RCU_APB1EN_UART4EN_POS)		/** UART4 clock enable */
#define RCU_APB1EN_I2C0EN_POS								0x15UL		/** I2C0 clock enable */
#define RCU_APB1EN_I2C0EN_MSK								(0x01UL << RCU_APB1EN_I2C0EN_POS)		/** I2C0 clock enable */
#define RCU_APB1EN_I2C1EN_POS								0x16UL		/** I2C1 clock enable */
#define RCU_APB1EN_I2C1EN_MSK								(0x01UL << RCU_APB1EN_I2C1EN_POS)		/** I2C1 clock enable */
#define RCU_APB1EN_CAN0EN_POS								0x19UL		/** CAN0 clock enable */
#define RCU_APB1EN_CAN0EN_MSK								(0x01UL << RCU_APB1EN_CAN0EN_POS)		/** CAN0 clock enable */
#define RCU_APB1EN_CAN1EN_POS								0x1AUL		/** CAN1 clock enable */
#define RCU_APB1EN_CAN1EN_MSK								(0x01UL << RCU_APB1EN_CAN1EN_POS)		/** CAN1 clock enable */
#define RCU_APB1EN_BKPIEN_POS								0x1BUL		/** Backup interface clock enable  */
#define RCU_APB1EN_BKPIEN_MSK								(0x01UL << RCU_APB1EN_BKPIEN_POS)		/** Backup interface clock enable  */
#define RCU_APB1EN_PMUEN_POS								0x1CUL		/** Power control clock enable  */
#define RCU_APB1EN_PMUEN_MSK								(0x01UL << RCU_APB1EN_PMUEN_POS)		/** Power control clock enable  */
#define RCU_APB1EN_DACEN_POS								0x1DUL		/** DAC clock enable */
#define RCU_APB1EN_DACEN_MSK								(0x01UL << RCU_APB1EN_DACEN_POS)		/** DAC clock enable */
#define RCU_BDCTL_LXTALEN_POS								0x00UL		/** LXTAL enable */
#define RCU_BDCTL_LXTALEN_MSK								(0x01UL << RCU_BDCTL_LXTALEN_POS)		/** LXTAL enable */
#define RCU_BDCTL_LXTALSTB_POS								0x01UL		/** External low-speed oscillator stabilization */
#define RCU_BDCTL_LXTALSTB_MSK								(0x01UL << RCU_BDCTL_LXTALSTB_POS)		/** External low-speed oscillator stabilization */
#define RCU_BDCTL_LXTALBPS_POS								0x02UL		/** LXTAL bypass mode enable */
#define RCU_BDCTL_LXTALBPS_MSK								(0x01UL << RCU_BDCTL_LXTALBPS_POS)		/** LXTAL bypass mode enable */
#define RCU_BDCTL_RTCSRC_POS								0x08UL		/** RTC clock entry selection */
#define RCU_BDCTL_RTCSRC_MSK								(0x03UL << RCU_BDCTL_RTCSRC_POS)		/** RTC clock entry selection */
#define RCU_BDCTL_RTCEN_POS								0x0FUL		/** RTC clock enable */
#define RCU_BDCTL_RTCEN_MSK								(0x01UL << RCU_BDCTL_RTCEN_POS)		/** RTC clock enable */
#define RCU_BDCTL_BKPRST_POS								0x10UL		/** Backup domain reset */
#define RCU_BDCTL_BKPRST_MSK								(0x01UL << RCU_BDCTL_BKPRST_POS)		/** Backup domain reset */
#define RCU_RSTSCK_IRC40KEN_POS								0x00UL		/** IRC40K enable */
#define RCU_RSTSCK_IRC40KEN_MSK								(0x01UL << RCU_RSTSCK_IRC40KEN_POS)		/** IRC40K enable */
#define RCU_RSTSCK_IRC40KSTB_POS								0x01UL		/** IRC40K stabilization */
#define RCU_RSTSCK_IRC40KSTB_MSK								(0x01UL << RCU_RSTSCK_IRC40KSTB_POS)		/** IRC40K stabilization */
#define RCU_RSTSCK_RSTFC_POS								0x18UL		/** Reset flag clear */
#define RCU_RSTSCK_RSTFC_MSK								(0x01UL << RCU_RSTSCK_RSTFC_POS)		/** Reset flag clear */
#define RCU_RSTSCK_EPRSTF_POS								0x1AUL		/** External PIN reset flag */
#define RCU_RSTSCK_EPRSTF_MSK								(0x01UL << RCU_RSTSCK_EPRSTF_POS)		/** External PIN reset flag */
#define RCU_RSTSCK_PORRSTF_POS								0x1BUL		/** Power reset flag */
#define RCU_RSTSCK_PORRSTF_MSK								(0x01UL << RCU_RSTSCK_PORRSTF_POS)		/** Power reset flag */
#define RCU_RSTSCK_SWRSTF_POS								0x1CUL		/** Software reset flag */
#define RCU_RSTSCK_SWRSTF_MSK								(0x01UL << RCU_RSTSCK_SWRSTF_POS)		/** Software reset flag */
#define RCU_RSTSCK_FWDGTRSTF_POS								0x1DUL		/** Free Watchdog timer reset flag */
#define RCU_RSTSCK_FWDGTRSTF_MSK								(0x01UL << RCU_RSTSCK_FWDGTRSTF_POS)		/** Free Watchdog timer reset flag */
#define RCU_RSTSCK_WWDGTRSTF_POS								0x1EUL		/** Window watchdog timer reset flag */
#define RCU_RSTSCK_WWDGTRSTF_MSK								(0x01UL << RCU_RSTSCK_WWDGTRSTF_POS)		/** Window watchdog timer reset flag */
#define RCU_RSTSCK_LPRSTF_POS								0x1FUL		/** Low-power reset flag */
#define RCU_RSTSCK_LPRSTF_MSK								(0x01UL << RCU_RSTSCK_LPRSTF_POS)		/** Low-power reset flag */
#define RCU_AHBRST_USBFSRST_POS								0x0CUL		/** USBFS reset */
#define RCU_AHBRST_USBFSRST_MSK								(0x01UL << RCU_AHBRST_USBFSRST_POS)		/** USBFS reset */
#define RCU_CFG1_PREDV0_POS								0x00UL		/** PREDV0 division factor */
#define RCU_CFG1_PREDV0_MSK								(0x0FUL << RCU_CFG1_PREDV0_POS)		/** PREDV0 division factor */
#define RCU_CFG1_PREDV1_POS								0x04UL		/** PREDV1 division factor */
#define RCU_CFG1_PREDV1_MSK								(0x0FUL << RCU_CFG1_PREDV1_POS)		/** PREDV1 division factor */
#define RCU_CFG1_PLL1MF_POS								0x08UL		/** The PLL1 clock multiplication factor */
#define RCU_CFG1_PLL1MF_MSK								(0x0FUL << RCU_CFG1_PLL1MF_POS)		/** The PLL1 clock multiplication factor */
#define RCU_CFG1_PLL2MF_POS								0x0CUL		/** The PLL2 clock multiplication factor */
#define RCU_CFG1_PLL2MF_MSK								(0x0FUL << RCU_CFG1_PLL2MF_POS)		/** The PLL2 clock multiplication factor */
#define RCU_CFG1_PREDV0SEL_POS								0x10UL		/** PREDV0 input Clock Source Selection */
#define RCU_CFG1_PREDV0SEL_MSK								(0x01UL << RCU_CFG1_PREDV0SEL_POS)		/** PREDV0 input Clock Source Selection */
#define RCU_CFG1_I2S1SEL_POS								0x11UL		/** I2S1 Clock Source Selection */
#define RCU_CFG1_I2S1SEL_MSK								(0x01UL << RCU_CFG1_I2S1SEL_POS)		/** I2S1 Clock Source Selection */
#define RCU_CFG1_I2S2SEL_POS								0x12UL		/** I2S2 Clock Source Selection */
#define RCU_CFG1_I2S2SEL_MSK								(0x01UL << RCU_CFG1_I2S2SEL_POS)		/** I2S2 Clock Source Selection */
#define RCU_DSV_DSLPVS_POS								0x00UL		/** Deep-sleep mode voltage select */
#define RCU_DSV_DSLPVS_MSK								(0x03UL << RCU_DSV_DSLPVS_POS)		/** Deep-sleep mode voltage select */
#define RTC_INTEN_OVIE_POS								0x02UL		/** Overflow interrupt enable */
#define RTC_INTEN_OVIE_MSK								(0x01UL << RTC_INTEN_OVIE_POS)		/** Overflow interrupt enable */
#define RTC_INTEN_ALRMIE_POS								0x01UL		/** Alarm interrupt enable */
#define RTC_INTEN_ALRMIE_MSK								(0x01UL << RTC_INTEN_ALRMIE_POS)		/** Alarm interrupt enable */
#define RTC_INTEN_SCIE_POS								0x00UL		/** Second interrupt */
#define RTC_INTEN_SCIE_MSK								(0x01UL << RTC_INTEN_SCIE_POS)		/** Second interrupt */
#define RTC_CTL_LWOFF_POS								0x05UL		/** Last write operation finished flag */
#define RTC_CTL_LWOFF_MSK								(0x01UL << RTC_CTL_LWOFF_POS)		/** Last write operation finished flag */
#define RTC_CTL_CMF_POS								0x04UL		/** Configuration mode flag */
#define RTC_CTL_CMF_MSK								(0x01UL << RTC_CTL_CMF_POS)		/** Configuration mode flag */
#define RTC_CTL_RSYNF_POS								0x03UL		/** Registers synchronized flag */
#define RTC_CTL_RSYNF_MSK								(0x01UL << RTC_CTL_RSYNF_POS)		/** Registers synchronized flag */
#define RTC_CTL_OVIF_POS								0x02UL		/** Overflow interrupt flag */
#define RTC_CTL_OVIF_MSK								(0x01UL << RTC_CTL_OVIF_POS)		/** Overflow interrupt flag */
#define RTC_CTL_ALRMIF_POS								0x01UL		/** Alarm interrupt flag */
#define RTC_CTL_ALRMIF_MSK								(0x01UL << RTC_CTL_ALRMIF_POS)		/** Alarm interrupt flag */
#define RTC_CTL_SCIF_POS								0x00UL		/** Sencond interrupt flag */
#define RTC_CTL_SCIF_MSK								(0x01UL << RTC_CTL_SCIF_POS)		/** Sencond interrupt flag */
#define RTC_PSCH_PSC_POS								0x00UL		/** RTC prescaler value high */
#define RTC_PSCH_PSC_MSK								(0x0FUL << RTC_PSCH_PSC_POS)		/** RTC prescaler value high */
#define RTC_PSCL_PSC_POS								0x00UL		/** RTC prescaler value low */
#define RTC_PSCL_PSC_MSK								(0xFFFFUL << RTC_PSCL_PSC_POS)		/** RTC prescaler value low */
#define RTC_DIVH_DIV_POS								0x00UL		/** RTC divider value high */
#define RTC_DIVH_DIV_MSK								(0x0FUL << RTC_DIVH_DIV_POS)		/** RTC divider value high */
#define RTC_DIVL_DIV_POS								0x00UL		/** RTC divider value low */
#define RTC_DIVL_DIV_MSK								(0xFFFFUL << RTC_DIVL_DIV_POS)		/** RTC divider value low */
#define RTC_CNTH_CNT_POS								0x00UL		/** RTC counter value high */
#define RTC_CNTH_CNT_MSK								(0xFFFFUL << RTC_CNTH_CNT_POS)		/** RTC counter value high */
#define RTC_CNTL_CNT_POS								0x00UL		/** RTC counter value low */
#define RTC_CNTL_CNT_MSK								(0xFFFFUL << RTC_CNTL_CNT_POS)		/** RTC counter value low */
#define RTC_ALRMH_ALRM_POS								0x00UL		/** Alarm value high */
#define RTC_ALRMH_ALRM_MSK								(0xFFFFUL << RTC_ALRMH_ALRM_POS)		/** Alarm value high */
#define RTC_ALRML_ALRM_POS								0x00UL		/** alarm value low */
#define RTC_ALRML_ALRM_MSK								(0xFFFFUL << RTC_ALRML_ALRM_POS)		/** alarm value low */
#define SPI_CTL0_BDEN_POS								0x0FUL		/** Bidirectional enable */
#define SPI_CTL0_BDEN_MSK								(0x01UL << SPI_CTL0_BDEN_POS)		/** Bidirectional enable */
#define SPI_CTL0_BDOEN_POS								0x0EUL		/** Bidirectional Transmit output enable  */
#define SPI_CTL0_BDOEN_MSK								(0x01UL << SPI_CTL0_BDOEN_POS)		/** Bidirectional Transmit output enable  */
#define SPI_CTL0_CRCEN_POS								0x0DUL		/** CRC Calculation Enable */
#define SPI_CTL0_CRCEN_MSK								(0x01UL << SPI_CTL0_CRCEN_POS)		/** CRC Calculation Enable */
#define SPI_CTL0_CRCNT_POS								0x0CUL		/** CRC Next Transfer */
#define SPI_CTL0_CRCNT_MSK								(0x01UL << SPI_CTL0_CRCNT_POS)		/** CRC Next Transfer */
#define SPI_CTL0_FF16_POS								0x0BUL		/** Data frame format */
#define SPI_CTL0_FF16_MSK								(0x01UL << SPI_CTL0_FF16_POS)		/** Data frame format */
#define SPI_CTL0_RO_POS								0x0AUL		/** Receive only */
#define SPI_CTL0_RO_MSK								(0x01UL << SPI_CTL0_RO_POS)		/** Receive only */
#define SPI_CTL0_SWNSSEN_POS								0x09UL		/** NSS Software Mode Selection */
#define SPI_CTL0_SWNSSEN_MSK								(0x01UL << SPI_CTL0_SWNSSEN_POS)		/** NSS Software Mode Selection */
#define SPI_CTL0_SWNSS_POS								0x08UL		/** NSS Pin Selection In NSS Software Mode */
#define SPI_CTL0_SWNSS_MSK								(0x01UL << SPI_CTL0_SWNSS_POS)		/** NSS Pin Selection In NSS Software Mode */
#define SPI_CTL0_LF_POS								0x07UL		/** LSB First Mode */
#define SPI_CTL0_LF_MSK								(0x01UL << SPI_CTL0_LF_POS)		/** LSB First Mode */
#define SPI_CTL0_SPIEN_POS								0x06UL		/** SPI enable */
#define SPI_CTL0_SPIEN_MSK								(0x01UL << SPI_CTL0_SPIEN_POS)		/** SPI enable */
#define SPI_CTL0_PSC_POS								0x03UL		/** Master Clock Prescaler Selection */
#define SPI_CTL0_PSC_MSK								(0x07UL << SPI_CTL0_PSC_POS)		/** Master Clock Prescaler Selection */
#define SPI_CTL0_MSTMOD_POS								0x02UL		/** Master Mode Enable */
#define SPI_CTL0_MSTMOD_MSK								(0x01UL << SPI_CTL0_MSTMOD_POS)		/** Master Mode Enable */
#define SPI_CTL0_CKPL_POS								0x01UL		/** Clock polarity Selection */
#define SPI_CTL0_CKPL_MSK								(0x01UL << SPI_CTL0_CKPL_POS)		/** Clock polarity Selection */
#define SPI_CTL0_CKPH_POS								0x00UL		/** Clock Phase Selection */
#define SPI_CTL0_CKPH_MSK								(0x01UL << SPI_CTL0_CKPH_POS)		/** Clock Phase Selection */
#define SPI_CTL1_TBEIE_POS								0x07UL		/** Tx buffer empty interrupt enable */
#define SPI_CTL1_TBEIE_MSK								(0x01UL << SPI_CTL1_TBEIE_POS)		/** Tx buffer empty interrupt enable */
#define SPI_CTL1_RBNEIE_POS								0x06UL		/** RX buffer not empty interrupt enable */
#define SPI_CTL1_RBNEIE_MSK								(0x01UL << SPI_CTL1_RBNEIE_POS)		/** RX buffer not empty interrupt enable */
#define SPI_CTL1_ERRIE_POS								0x05UL		/** Error interrupt enable */
#define SPI_CTL1_ERRIE_MSK								(0x01UL << SPI_CTL1_ERRIE_POS)		/** Error interrupt enable */
#define SPI_CTL1_TMOD_POS								0x04UL		/** SPI TI mode enable */
#define SPI_CTL1_TMOD_MSK								(0x01UL << SPI_CTL1_TMOD_POS)		/** SPI TI mode enable */
#define SPI_CTL1_NSSP_POS								0x03UL		/** SPI NSS pulse mode enable */
#define SPI_CTL1_NSSP_MSK								(0x01UL << SPI_CTL1_NSSP_POS)		/** SPI NSS pulse mode enable */
#define SPI_CTL1_NSSDRV_POS								0x02UL		/** Drive NSS Output */
#define SPI_CTL1_NSSDRV_MSK								(0x01UL << SPI_CTL1_NSSDRV_POS)		/** Drive NSS Output */
#define SPI_CTL1_DMATEN_POS								0x01UL		/** Transmit Buffer DMA Enable */
#define SPI_CTL1_DMATEN_MSK								(0x01UL << SPI_CTL1_DMATEN_POS)		/** Transmit Buffer DMA Enable */
#define SPI_CTL1_DMAREN_POS								0x00UL		/** Rx buffer DMA enable */
#define SPI_CTL1_DMAREN_MSK								(0x01UL << SPI_CTL1_DMAREN_POS)		/** Rx buffer DMA enable */
#define SPI_STAT_FERR_POS								0x08UL		/** Format error */
#define SPI_STAT_FERR_MSK								(0x01UL << SPI_STAT_FERR_POS)		/** Format error */
#define SPI_STAT_TRANS_POS								0x07UL		/** Transmitting On-going Bit */
#define SPI_STAT_TRANS_MSK								(0x01UL << SPI_STAT_TRANS_POS)		/** Transmitting On-going Bit */
#define SPI_STAT_RXORERR_POS								0x06UL		/** Reception Overrun Error Bit */
#define SPI_STAT_RXORERR_MSK								(0x01UL << SPI_STAT_RXORERR_POS)		/** Reception Overrun Error Bit */
#define SPI_STAT_CONFERR_POS								0x05UL		/** SPI Configuration error */
#define SPI_STAT_CONFERR_MSK								(0x01UL << SPI_STAT_CONFERR_POS)		/** SPI Configuration error */
#define SPI_STAT_CRCERR_POS								0x04UL		/** SPI CRC Error Bit */
#define SPI_STAT_CRCERR_MSK								(0x01UL << SPI_STAT_CRCERR_POS)		/** SPI CRC Error Bit */
#define SPI_STAT_TXURERR_POS								0x03UL		/** Transmission underrun error bit */
#define SPI_STAT_TXURERR_MSK								(0x01UL << SPI_STAT_TXURERR_POS)		/** Transmission underrun error bit */
#define SPI_STAT_I2SCH_POS								0x02UL		/** I2S channel side */
#define SPI_STAT_I2SCH_MSK								(0x01UL << SPI_STAT_I2SCH_POS)		/** I2S channel side */
#define SPI_STAT_TBE_POS								0x01UL		/** Transmit Buffer Empty */
#define SPI_STAT_TBE_MSK								(0x01UL << SPI_STAT_TBE_POS)		/** Transmit Buffer Empty */
#define SPI_STAT_RBNE_POS								0x00UL		/** Receive Buffer Not Empty */
#define SPI_STAT_RBNE_MSK								(0x01UL << SPI_STAT_RBNE_POS)		/** Receive Buffer Not Empty */
#define SPI_DATA_SPI_DATA_POS								0x00UL		/** Data transfer register */
#define SPI_DATA_SPI_DATA_MSK								(0xFFFFUL << SPI_DATA_SPI_DATA_POS)		/** Data transfer register */
#define SPI_CRCPOLY_CRCPOLY_POS								0x00UL		/** CRC polynomial value */
#define SPI_CRCPOLY_CRCPOLY_MSK								(0xFFFFUL << SPI_CRCPOLY_CRCPOLY_POS)		/** CRC polynomial value */
#define SPI_RCRC_RCRC_POS								0x00UL		/** RX CRC value */
#define SPI_RCRC_RCRC_MSK								(0xFFFFUL << SPI_RCRC_RCRC_POS)		/** RX CRC value */
#define SPI_TCRC_TCRC_POS								0x00UL		/** Tx CRC value */
#define SPI_TCRC_TCRC_MSK								(0xFFFFUL << SPI_TCRC_TCRC_POS)		/** Tx CRC value */
#define SPI_I2SCTL_I2SSEL_POS								0x0BUL		/** I2S mode selection */
#define SPI_I2SCTL_I2SSEL_MSK								(0x01UL << SPI_I2SCTL_I2SSEL_POS)		/** I2S mode selection */
#define SPI_I2SCTL_I2SEN_POS								0x0AUL		/** I2S Enable */
#define SPI_I2SCTL_I2SEN_MSK								(0x01UL << SPI_I2SCTL_I2SEN_POS)		/** I2S Enable */
#define SPI_I2SCTL_I2SOPMOD_POS								0x08UL		/** I2S operation mode */
#define SPI_I2SCTL_I2SOPMOD_MSK								(0x03UL << SPI_I2SCTL_I2SOPMOD_POS)		/** I2S operation mode */
#define SPI_I2SCTL_PCMSMOD_POS								0x07UL		/** PCM frame synchronization mode */
#define SPI_I2SCTL_PCMSMOD_MSK								(0x01UL << SPI_I2SCTL_PCMSMOD_POS)		/** PCM frame synchronization mode */
#define SPI_I2SCTL_I2SSTD_POS								0x04UL		/** I2S standard selection */
#define SPI_I2SCTL_I2SSTD_MSK								(0x03UL << SPI_I2SCTL_I2SSTD_POS)		/** I2S standard selection */
#define SPI_I2SCTL_CKPL_POS								0x03UL		/** Idle state clock polarity */
#define SPI_I2SCTL_CKPL_MSK								(0x01UL << SPI_I2SCTL_CKPL_POS)		/** Idle state clock polarity */
#define SPI_I2SCTL_DTLEN_POS								0x01UL		/** Data length */
#define SPI_I2SCTL_DTLEN_MSK								(0x03UL << SPI_I2SCTL_DTLEN_POS)		/** Data length */
#define SPI_I2SCTL_CHLEN_POS								0x00UL		/** Channel length (number of bits per audio channel) */
#define SPI_I2SCTL_CHLEN_MSK								(0x01UL << SPI_I2SCTL_CHLEN_POS)		/** Channel length (number of bits per audio channel) */
#define SPI_I2SPSC_MCKOEN_POS								0x09UL		/** I2S_MCK output enable */
#define SPI_I2SPSC_MCKOEN_MSK								(0x01UL << SPI_I2SPSC_MCKOEN_POS)		/** I2S_MCK output enable */
#define SPI_I2SPSC_OF_POS								0x08UL		/** Odd factor for the prescaler */
#define SPI_I2SPSC_OF_MSK								(0x01UL << SPI_I2SPSC_OF_POS)		/** Odd factor for the prescaler */
#define SPI_I2SPSC_DIV_POS								0x00UL		/** Dividing factor for the prescaler */
#define SPI_I2SPSC_DIV_MSK								(0xFFUL << SPI_I2SPSC_DIV_POS)		/** Dividing factor for the prescaler */
#define TIMER_CTL0_CKDIV_POS								0x08UL		/** Clock division */
#define TIMER_CTL0_CKDIV_MSK								(0x03UL << TIMER_CTL0_CKDIV_POS)		/** Clock division */
#define TIMER_CTL0_ARSE_POS								0x07UL		/** Auto-reload shadow enable */
#define TIMER_CTL0_ARSE_MSK								(0x01UL << TIMER_CTL0_ARSE_POS)		/** Auto-reload shadow enable */
#define TIMER_CTL0_CAM_POS								0x05UL		/** Counter aligns mode selection */
#define TIMER_CTL0_CAM_MSK								(0x03UL << TIMER_CTL0_CAM_POS)		/** Counter aligns mode selection */
#define TIMER_CTL0_DIR_POS								0x04UL		/** Direction */
#define TIMER_CTL0_DIR_MSK								(0x01UL << TIMER_CTL0_DIR_POS)		/** Direction */
#define TIMER_CTL0_SPM_POS								0x03UL		/** Single pulse mode */
#define TIMER_CTL0_SPM_MSK								(0x01UL << TIMER_CTL0_SPM_POS)		/** Single pulse mode */
#define TIMER_CTL0_UPS_POS								0x02UL		/** Update source */
#define TIMER_CTL0_UPS_MSK								(0x01UL << TIMER_CTL0_UPS_POS)		/** Update source */
#define TIMER_CTL0_UPDIS_POS								0x01UL		/** Update disable */
#define TIMER_CTL0_UPDIS_MSK								(0x01UL << TIMER_CTL0_UPDIS_POS)		/** Update disable */
#define TIMER_CTL0_CEN_POS								0x00UL		/** Counter enable */
#define TIMER_CTL0_CEN_MSK								(0x01UL << TIMER_CTL0_CEN_POS)		/** Counter enable */
#define TIMER_CTL1_ISO3_POS								0x0EUL		/** Idle state of channel 3 output */
#define TIMER_CTL1_ISO3_MSK								(0x01UL << TIMER_CTL1_ISO3_POS)		/** Idle state of channel 3 output */
#define TIMER_CTL1_ISO2N_POS								0x0DUL		/** Idle state of channel 2 complementary output */
#define TIMER_CTL1_ISO2N_MSK								(0x01UL << TIMER_CTL1_ISO2N_POS)		/** Idle state of channel 2 complementary output */
#define TIMER_CTL1_ISO2_POS								0x0CUL		/** Idle state of channel 2 output */
#define TIMER_CTL1_ISO2_MSK								(0x01UL << TIMER_CTL1_ISO2_POS)		/** Idle state of channel 2 output */
#define TIMER_CTL1_ISO1N_POS								0x0BUL		/** Idle state of channel 1 complementary output */
#define TIMER_CTL1_ISO1N_MSK								(0x01UL << TIMER_CTL1_ISO1N_POS)		/** Idle state of channel 1 complementary output */
#define TIMER_CTL1_ISO1_POS								0x0AUL		/** Idle state of channel 1 output */
#define TIMER_CTL1_ISO1_MSK								(0x01UL << TIMER_CTL1_ISO1_POS)		/** Idle state of channel 1 output */
#define TIMER_CTL1_ISO0N_POS								0x09UL		/** Idle state of channel 0 complementary output */
#define TIMER_CTL1_ISO0N_MSK								(0x01UL << TIMER_CTL1_ISO0N_POS)		/** Idle state of channel 0 complementary output */
#define TIMER_CTL1_ISO0_POS								0x08UL		/** Idle state of channel 0 output */
#define TIMER_CTL1_ISO0_MSK								(0x01UL << TIMER_CTL1_ISO0_POS)		/** Idle state of channel 0 output */
#define TIMER_CTL1_TI0S_POS								0x07UL		/** Channel 0 trigger input selection */
#define TIMER_CTL1_TI0S_MSK								(0x01UL << TIMER_CTL1_TI0S_POS)		/** Channel 0 trigger input selection */
#define TIMER_CTL1_MMC_POS								0x04UL		/** Master mode control */
#define TIMER_CTL1_MMC_MSK								(0x07UL << TIMER_CTL1_MMC_POS)		/** Master mode control */
#define TIMER_CTL1_DMAS_POS								0x03UL		/** DMA request source selection */
#define TIMER_CTL1_DMAS_MSK								(0x01UL << TIMER_CTL1_DMAS_POS)		/** DMA request source selection */
#define TIMER_CTL1_CCUC_POS								0x02UL		/** Commutation control shadow register update control */
#define TIMER_CTL1_CCUC_MSK								(0x01UL << TIMER_CTL1_CCUC_POS)		/** Commutation control shadow register update control */
#define TIMER_CTL1_CCSE_POS								0x00UL		/** Commutation control shadow enable */
#define TIMER_CTL1_CCSE_MSK								(0x01UL << TIMER_CTL1_CCSE_POS)		/** Commutation control shadow enable */
#define TIMER_SMCFG_ETP_POS								0x0FUL		/** External trigger polarity */
#define TIMER_SMCFG_ETP_MSK								(0x01UL << TIMER_SMCFG_ETP_POS)		/** External trigger polarity */
#define TIMER_SMCFG_SMC1_POS								0x0EUL		/** Part of SMC for enable External clock mode1 */
#define TIMER_SMCFG_SMC1_MSK								(0x01UL << TIMER_SMCFG_SMC1_POS)		/** Part of SMC for enable External clock mode1 */
#define TIMER_SMCFG_ETPSC_POS								0x0CUL		/** External trigger prescaler */
#define TIMER_SMCFG_ETPSC_MSK								(0x03UL << TIMER_SMCFG_ETPSC_POS)		/** External trigger prescaler */
#define TIMER_SMCFG_ETFC_POS								0x08UL		/** External trigger filter control */
#define TIMER_SMCFG_ETFC_MSK								(0x0FUL << TIMER_SMCFG_ETFC_POS)		/** External trigger filter control */
#define TIMER_SMCFG_MSM_POS								0x07UL		/** Master/Slave mode */
#define TIMER_SMCFG_MSM_MSK								(0x01UL << TIMER_SMCFG_MSM_POS)		/** Master/Slave mode */
#define TIMER_SMCFG_TRGS_POS								0x04UL		/** Trigger selection */
#define TIMER_SMCFG_TRGS_MSK								(0x07UL << TIMER_SMCFG_TRGS_POS)		/** Trigger selection */
#define TIMER_SMCFG_SMC_POS								0x00UL		/** Slave mode selection */
#define TIMER_SMCFG_SMC_MSK								(0x07UL << TIMER_SMCFG_SMC_POS)		/** Slave mode selection */
#define TIMER_DMAINTEN_TRGDEN_POS								0x0EUL		/** Trigger DMA request enable */
#define TIMER_DMAINTEN_TRGDEN_MSK								(0x01UL << TIMER_DMAINTEN_TRGDEN_POS)		/** Trigger DMA request enable */
#define TIMER_DMAINTEN_CMTDEN_POS								0x0DUL		/** Commutation DMA request enable */
#define TIMER_DMAINTEN_CMTDEN_MSK								(0x01UL << TIMER_DMAINTEN_CMTDEN_POS)		/** Commutation DMA request enable */
#define TIMER_DMAINTEN_CH3DEN_POS								0x0CUL		/** Channel 3 capture/compare DMA request enable */
#define TIMER_DMAINTEN_CH3DEN_MSK								(0x01UL << TIMER_DMAINTEN_CH3DEN_POS)		/** Channel 3 capture/compare DMA request enable */
#define TIMER_DMAINTEN_CH2DEN_POS								0x0BUL		/** Channel 2 capture/compare DMA request enable */
#define TIMER_DMAINTEN_CH2DEN_MSK								(0x01UL << TIMER_DMAINTEN_CH2DEN_POS)		/** Channel 2 capture/compare DMA request enable */
#define TIMER_DMAINTEN_CH1DEN_POS								0x0AUL		/** Channel 1 capture/compare DMA request enable */
#define TIMER_DMAINTEN_CH1DEN_MSK								(0x01UL << TIMER_DMAINTEN_CH1DEN_POS)		/** Channel 1 capture/compare DMA request enable */
#define TIMER_DMAINTEN_CH0DEN_POS								0x09UL		/** Channel 0 capture/compare DMA request enable */
#define TIMER_DMAINTEN_CH0DEN_MSK								(0x01UL << TIMER_DMAINTEN_CH0DEN_POS)		/** Channel 0 capture/compare DMA request enable */
#define TIMER_DMAINTEN_UPDEN_POS								0x08UL		/** Update DMA request enable */
#define TIMER_DMAINTEN_UPDEN_MSK								(0x01UL << TIMER_DMAINTEN_UPDEN_POS)		/** Update DMA request enable */
#define TIMER_DMAINTEN_BRKIE_POS								0x07UL		/** Break interrupt enable */
#define TIMER_DMAINTEN_BRKIE_MSK								(0x01UL << TIMER_DMAINTEN_BRKIE_POS)		/** Break interrupt enable */
#define TIMER_DMAINTEN_TRGIE_POS								0x06UL		/** Trigger interrupt enable */
#define TIMER_DMAINTEN_TRGIE_MSK								(0x01UL << TIMER_DMAINTEN_TRGIE_POS)		/** Trigger interrupt enable */
#define TIMER_DMAINTEN_CMTIE_POS								0x05UL		/** commutation interrupt enable */
#define TIMER_DMAINTEN_CMTIE_MSK								(0x01UL << TIMER_DMAINTEN_CMTIE_POS)		/** commutation interrupt enable */
#define TIMER_DMAINTEN_CH3IE_POS								0x04UL		/** Channel 3 capture/compare interrupt enable */
#define TIMER_DMAINTEN_CH3IE_MSK								(0x01UL << TIMER_DMAINTEN_CH3IE_POS)		/** Channel 3 capture/compare interrupt enable */
#define TIMER_DMAINTEN_CH2IE_POS								0x03UL		/** Channel 2 capture/compare interrupt enable */
#define TIMER_DMAINTEN_CH2IE_MSK								(0x01UL << TIMER_DMAINTEN_CH2IE_POS)		/** Channel 2 capture/compare interrupt enable */
#define TIMER_DMAINTEN_CH1IE_POS								0x02UL		/** Channel 1 capture/compare interrupt enable */
#define TIMER_DMAINTEN_CH1IE_MSK								(0x01UL << TIMER_DMAINTEN_CH1IE_POS)		/** Channel 1 capture/compare interrupt enable */
#define TIMER_DMAINTEN_CH0IE_POS								0x01UL		/** Channel 0 capture/compare interrupt enable */
#define TIMER_DMAINTEN_CH0IE_MSK								(0x01UL << TIMER_DMAINTEN_CH0IE_POS)		/** Channel 0 capture/compare interrupt enable */
#define TIMER_DMAINTEN_UPIE_POS								0x00UL		/** Update interrupt enable */
#define TIMER_DMAINTEN_UPIE_MSK								(0x01UL << TIMER_DMAINTEN_UPIE_POS)		/** Update interrupt enable */
#define TIMER_INTF_CH3OF_POS								0x0CUL		/** Channel 3 over capture flag */
#define TIMER_INTF_CH3OF_MSK								(0x01UL << TIMER_INTF_CH3OF_POS)		/** Channel 3 over capture flag */
#define TIMER_INTF_CH2OF_POS								0x0BUL		/** Channel 2 over capture flag */
#define TIMER_INTF_CH2OF_MSK								(0x01UL << TIMER_INTF_CH2OF_POS)		/** Channel 2 over capture flag */
#define TIMER_INTF_CH1OF_POS								0x0AUL		/** Channel 1 over capture flag */
#define TIMER_INTF_CH1OF_MSK								(0x01UL << TIMER_INTF_CH1OF_POS)		/** Channel 1 over capture flag */
#define TIMER_INTF_CH0OF_POS								0x09UL		/** Channel 0 over capture flag */
#define TIMER_INTF_CH0OF_MSK								(0x01UL << TIMER_INTF_CH0OF_POS)		/** Channel 0 over capture flag */
#define TIMER_INTF_BRKIF_POS								0x07UL		/** Break interrupt flag */
#define TIMER_INTF_BRKIF_MSK								(0x01UL << TIMER_INTF_BRKIF_POS)		/** Break interrupt flag */
#define TIMER_INTF_TRGIF_POS								0x06UL		/** Trigger interrupt flag */
#define TIMER_INTF_TRGIF_MSK								(0x01UL << TIMER_INTF_TRGIF_POS)		/** Trigger interrupt flag */
#define TIMER_INTF_CMTIF_POS								0x05UL		/** Channel commutation interrupt flag */
#define TIMER_INTF_CMTIF_MSK								(0x01UL << TIMER_INTF_CMTIF_POS)		/** Channel commutation interrupt flag */
#define TIMER_INTF_CH3IF_POS								0x04UL		/** Channel 3 capture/compare interrupt flag */
#define TIMER_INTF_CH3IF_MSK								(0x01UL << TIMER_INTF_CH3IF_POS)		/** Channel 3 capture/compare interrupt flag */
#define TIMER_INTF_CH2IF_POS								0x03UL		/**  Channel 2 capture/compare interrupt flag */
#define TIMER_INTF_CH2IF_MSK								(0x01UL << TIMER_INTF_CH2IF_POS)		/**  Channel 2 capture/compare interrupt flag */
#define TIMER_INTF_CH1IF_POS								0x02UL		/** Channel 1 capture/compare interrupt flag */
#define TIMER_INTF_CH1IF_MSK								(0x01UL << TIMER_INTF_CH1IF_POS)		/** Channel 1 capture/compare interrupt flag */
#define TIMER_INTF_CH0IF_POS								0x01UL		/** Channel 0 capture/compare interrupt flag */
#define TIMER_INTF_CH0IF_MSK								(0x01UL << TIMER_INTF_CH0IF_POS)		/** Channel 0 capture/compare interrupt flag */
#define TIMER_INTF_UPIF_POS								0x00UL		/** Update interrupt flag */
#define TIMER_INTF_UPIF_MSK								(0x01UL << TIMER_INTF_UPIF_POS)		/** Update interrupt flag */
#define TIMER_SWEVG_BRKG_POS								0x07UL		/** Break event generation */
#define TIMER_SWEVG_BRKG_MSK								(0x01UL << TIMER_SWEVG_BRKG_POS)		/** Break event generation */
#define TIMER_SWEVG_TRGG_POS								0x06UL		/** Trigger event generation */
#define TIMER_SWEVG_TRGG_MSK								(0x01UL << TIMER_SWEVG_TRGG_POS)		/** Trigger event generation */
#define TIMER_SWEVG_CMTG_POS								0x05UL		/** Channel commutation event generation */
#define TIMER_SWEVG_CMTG_MSK								(0x01UL << TIMER_SWEVG_CMTG_POS)		/** Channel commutation event generation */
#define TIMER_SWEVG_CH3G_POS								0x04UL		/** Channel 3 capture or compare event generation */
#define TIMER_SWEVG_CH3G_MSK								(0x01UL << TIMER_SWEVG_CH3G_POS)		/** Channel 3 capture or compare event generation */
#define TIMER_SWEVG_CH2G_POS								0x03UL		/** Channel 2 capture or compare event generation */
#define TIMER_SWEVG_CH2G_MSK								(0x01UL << TIMER_SWEVG_CH2G_POS)		/** Channel 2 capture or compare event generation */
#define TIMER_SWEVG_CH1G_POS								0x02UL		/** Channel 1 capture or compare event generation */
#define TIMER_SWEVG_CH1G_MSK								(0x01UL << TIMER_SWEVG_CH1G_POS)		/** Channel 1 capture or compare event generation */
#define TIMER_SWEVG_CH0G_POS								0x01UL		/** Channel 0 capture or compare event generation */
#define TIMER_SWEVG_CH0G_MSK								(0x01UL << TIMER_SWEVG_CH0G_POS)		/** Channel 0 capture or compare event generation */
#define TIMER_SWEVG_UPG_POS								0x00UL		/** Update event generation */
#define TIMER_SWEVG_UPG_MSK								(0x01UL << TIMER_SWEVG_UPG_POS)		/** Update event generation */
#define TIMER_CHCTL0_Output_CH1COMCEN_POS								0x0FUL		/** Channel 1 output compare clear enable */
#define TIMER_CHCTL0_Output_CH1COMCEN_MSK								(0x01UL << TIMER_CHCTL0_Output_CH1COMCEN_POS)		/** Channel 1 output compare clear enable */
#define TIMER_CHCTL0_Output_CH1COMCTL_POS								0x0CUL		/** Channel 1 compare output control */
#define TIMER_CHCTL0_Output_CH1COMCTL_MSK								(0x07UL << TIMER_CHCTL0_Output_CH1COMCTL_POS)		/** Channel 1 compare output control */
#define TIMER_CHCTL0_Output_CH1COMSEN_POS								0x0BUL		/** Channel 1 output compare shadow enable */
#define TIMER_CHCTL0_Output_CH1COMSEN_MSK								(0x01UL << TIMER_CHCTL0_Output_CH1COMSEN_POS)		/** Channel 1 output compare shadow enable */
#define TIMER_CHCTL0_Output_CH1COMFEN_POS								0x0AUL		/** Channel 1 output compare fast enable */
#define TIMER_CHCTL0_Output_CH1COMFEN_MSK								(0x01UL << TIMER_CHCTL0_Output_CH1COMFEN_POS)		/** Channel 1 output compare fast enable */
#define TIMER_CHCTL0_Output_CH1MS_POS								0x08UL		/** Channel 1 mode selection */
#define TIMER_CHCTL0_Output_CH1MS_MSK								(0x03UL << TIMER_CHCTL0_Output_CH1MS_POS)		/** Channel 1 mode selection */
#define TIMER_CHCTL0_Output_CH0COMCEN_POS								0x07UL		/** Channel 0 output compare clear enable */
#define TIMER_CHCTL0_Output_CH0COMCEN_MSK								(0x01UL << TIMER_CHCTL0_Output_CH0COMCEN_POS)		/** Channel 0 output compare clear enable */
#define TIMER_CHCTL0_Output_CH0COMCTL_POS								0x04UL		/** Channel 0 compare output control */
#define TIMER_CHCTL0_Output_CH0COMCTL_MSK								(0x07UL << TIMER_CHCTL0_Output_CH0COMCTL_POS)		/** Channel 0 compare output control */
#define TIMER_CHCTL0_Output_CH0COMSEN_POS								0x03UL		/** Channel 0 compare output shadow enable */
#define TIMER_CHCTL0_Output_CH0COMSEN_MSK								(0x01UL << TIMER_CHCTL0_Output_CH0COMSEN_POS)		/** Channel 0 compare output shadow enable */
#define TIMER_CHCTL0_Output_CH0COMFEN_POS								0x02UL		/** Channel 0 output compare fast enable */
#define TIMER_CHCTL0_Output_CH0COMFEN_MSK								(0x01UL << TIMER_CHCTL0_Output_CH0COMFEN_POS)		/** Channel 0 output compare fast enable */
#define TIMER_CHCTL0_Output_CH0MS_POS								0x00UL		/** Channel 0 I/O mode selection */
#define TIMER_CHCTL0_Output_CH0MS_MSK								(0x03UL << TIMER_CHCTL0_Output_CH0MS_POS)		/** Channel 0 I/O mode selection */
#define TIMER_CHCTL0_Input_CH1CAPFLT_POS								0x0CUL		/** Channel 1 input capture filter control */
#define TIMER_CHCTL0_Input_CH1CAPFLT_MSK								(0x0FUL << TIMER_CHCTL0_Input_CH1CAPFLT_POS)		/** Channel 1 input capture filter control */
#define TIMER_CHCTL0_Input_CH1CAPPSC_POS								0x0AUL		/** Channel 1 input capture prescaler */
#define TIMER_CHCTL0_Input_CH1CAPPSC_MSK								(0x03UL << TIMER_CHCTL0_Input_CH1CAPPSC_POS)		/** Channel 1 input capture prescaler */
#define TIMER_CHCTL0_Input_CH1MS_POS								0x08UL		/** Channel 1 mode selection */
#define TIMER_CHCTL0_Input_CH1MS_MSK								(0x03UL << TIMER_CHCTL0_Input_CH1MS_POS)		/** Channel 1 mode selection */
#define TIMER_CHCTL0_Input_CH0CAPFLT_POS								0x04UL		/** Channel 0 input capture filter control */
#define TIMER_CHCTL0_Input_CH0CAPFLT_MSK								(0x0FUL << TIMER_CHCTL0_Input_CH0CAPFLT_POS)		/** Channel 0 input capture filter control */
#define TIMER_CHCTL0_Input_CH0CAPPSC_POS								0x02UL		/** Channel 0 input capture prescaler */
#define TIMER_CHCTL0_Input_CH0CAPPSC_MSK								(0x03UL << TIMER_CHCTL0_Input_CH0CAPPSC_POS)		/** Channel 0 input capture prescaler */
#define TIMER_CHCTL0_Input_CH0MS_POS								0x00UL		/** Channel 0 mode selection */
#define TIMER_CHCTL0_Input_CH0MS_MSK								(0x03UL << TIMER_CHCTL0_Input_CH0MS_POS)		/** Channel 0 mode selection */
#define TIMER_CHCTL1_Output_CH3COMCEN_POS								0x0FUL		/** Channel 3 output compare clear enable */
#define TIMER_CHCTL1_Output_CH3COMCEN_MSK								(0x01UL << TIMER_CHCTL1_Output_CH3COMCEN_POS)		/** Channel 3 output compare clear enable */
#define TIMER_CHCTL1_Output_CH3COMCTL_POS								0x0CUL		/** Channel 3 compare output control */
#define TIMER_CHCTL1_Output_CH3COMCTL_MSK								(0x07UL << TIMER_CHCTL1_Output_CH3COMCTL_POS)		/** Channel 3 compare output control */
#define TIMER_CHCTL1_Output_CH3COMSEN_POS								0x0BUL		/** Channel 3 output compare shadow enable */
#define TIMER_CHCTL1_Output_CH3COMSEN_MSK								(0x01UL << TIMER_CHCTL1_Output_CH3COMSEN_POS)		/** Channel 3 output compare shadow enable */
#define TIMER_CHCTL1_Output_CH3COMFEN_POS								0x0AUL		/** Channel 3 output compare fast enable */
#define TIMER_CHCTL1_Output_CH3COMFEN_MSK								(0x01UL << TIMER_CHCTL1_Output_CH3COMFEN_POS)		/** Channel 3 output compare fast enable */
#define TIMER_CHCTL1_Output_CH3MS_POS								0x08UL		/** Channel 3 mode selection */
#define TIMER_CHCTL1_Output_CH3MS_MSK								(0x03UL << TIMER_CHCTL1_Output_CH3MS_POS)		/** Channel 3 mode selection */
#define TIMER_CHCTL1_Output_CH2COMCEN_POS								0x07UL		/** Channel 2 output compare clear enable */
#define TIMER_CHCTL1_Output_CH2COMCEN_MSK								(0x01UL << TIMER_CHCTL1_Output_CH2COMCEN_POS)		/** Channel 2 output compare clear enable */
#define TIMER_CHCTL1_Output_CH2COMCTL_POS								0x04UL		/** Channel 2 compare output control */
#define TIMER_CHCTL1_Output_CH2COMCTL_MSK								(0x07UL << TIMER_CHCTL1_Output_CH2COMCTL_POS)		/** Channel 2 compare output control */
#define TIMER_CHCTL1_Output_CH2COMSEN_POS								0x03UL		/** Channel 2 compare output shadow enable */
#define TIMER_CHCTL1_Output_CH2COMSEN_MSK								(0x01UL << TIMER_CHCTL1_Output_CH2COMSEN_POS)		/** Channel 2 compare output shadow enable */
#define TIMER_CHCTL1_Output_CH2COMFEN_POS								0x02UL		/** Channel 2 output compare fast enable */
#define TIMER_CHCTL1_Output_CH2COMFEN_MSK								(0x01UL << TIMER_CHCTL1_Output_CH2COMFEN_POS)		/** Channel 2 output compare fast enable */
#define TIMER_CHCTL1_Output_CH2MS_POS								0x00UL		/** Channel 2 I/O mode selection */
#define TIMER_CHCTL1_Output_CH2MS_MSK								(0x03UL << TIMER_CHCTL1_Output_CH2MS_POS)		/** Channel 2 I/O mode selection */
#define TIMER_CHCTL1_Input_CH3CAPFLT_POS								0x0CUL		/** Channel 3 input capture filter control */
#define TIMER_CHCTL1_Input_CH3CAPFLT_MSK								(0x0FUL << TIMER_CHCTL1_Input_CH3CAPFLT_POS)		/** Channel 3 input capture filter control */
#define TIMER_CHCTL1_Input_CH3CAPPSC_POS								0x0AUL		/** Channel 3 input capture prescaler */
#define TIMER_CHCTL1_Input_CH3CAPPSC_MSK								(0x03UL << TIMER_CHCTL1_Input_CH3CAPPSC_POS)		/** Channel 3 input capture prescaler */
#define TIMER_CHCTL1_Input_CH3MS_POS								0x08UL		/** Channel 3 mode selection */
#define TIMER_CHCTL1_Input_CH3MS_MSK								(0x03UL << TIMER_CHCTL1_Input_CH3MS_POS)		/** Channel 3 mode selection */
#define TIMER_CHCTL1_Input_CH2CAPFLT_POS								0x04UL		/** Channel 2 input capture filter control */
#define TIMER_CHCTL1_Input_CH2CAPFLT_MSK								(0x0FUL << TIMER_CHCTL1_Input_CH2CAPFLT_POS)		/** Channel 2 input capture filter control */
#define TIMER_CHCTL1_Input_CH2CAPPSC_POS								0x02UL		/** Channel 2 input capture prescaler */
#define TIMER_CHCTL1_Input_CH2CAPPSC_MSK								(0x03UL << TIMER_CHCTL1_Input_CH2CAPPSC_POS)		/** Channel 2 input capture prescaler */
#define TIMER_CHCTL1_Input_CH2MS_POS								0x00UL		/** Channel 2 mode selection */
#define TIMER_CHCTL1_Input_CH2MS_MSK								(0x03UL << TIMER_CHCTL1_Input_CH2MS_POS)		/** Channel 2 mode selection */
#define TIMER_CHCTL2_CH3P_POS								0x0DUL		/** Channel 3 capture/compare function polarity */
#define TIMER_CHCTL2_CH3P_MSK								(0x01UL << TIMER_CHCTL2_CH3P_POS)		/** Channel 3 capture/compare function polarity */
#define TIMER_CHCTL2_CH3EN_POS								0x0CUL		/** Channel 3 capture/compare function enable */
#define TIMER_CHCTL2_CH3EN_MSK								(0x01UL << TIMER_CHCTL2_CH3EN_POS)		/** Channel 3 capture/compare function enable */
#define TIMER_CHCTL2_CH2NP_POS								0x0BUL		/** Channel 2 complementary output polarity */
#define TIMER_CHCTL2_CH2NP_MSK								(0x01UL << TIMER_CHCTL2_CH2NP_POS)		/** Channel 2 complementary output polarity */
#define TIMER_CHCTL2_CH2NEN_POS								0x0AUL		/** Channel 2 complementary output enable */
#define TIMER_CHCTL2_CH2NEN_MSK								(0x01UL << TIMER_CHCTL2_CH2NEN_POS)		/** Channel 2 complementary output enable */
#define TIMER_CHCTL2_CH2P_POS								0x09UL		/** Channel 2 capture/compare function polarity */
#define TIMER_CHCTL2_CH2P_MSK								(0x01UL << TIMER_CHCTL2_CH2P_POS)		/** Channel 2 capture/compare function polarity */
#define TIMER_CHCTL2_CH2EN_POS								0x08UL		/** Channel 2 capture/compare function enable */
#define TIMER_CHCTL2_CH2EN_MSK								(0x01UL << TIMER_CHCTL2_CH2EN_POS)		/** Channel 2 capture/compare function enable */
#define TIMER_CHCTL2_CH1NP_POS								0x07UL		/** Channel 1 complementary output polarity */
#define TIMER_CHCTL2_CH1NP_MSK								(0x01UL << TIMER_CHCTL2_CH1NP_POS)		/** Channel 1 complementary output polarity */
#define TIMER_CHCTL2_CH1NEN_POS								0x06UL		/** Channel 1 complementary output enable */
#define TIMER_CHCTL2_CH1NEN_MSK								(0x01UL << TIMER_CHCTL2_CH1NEN_POS)		/** Channel 1 complementary output enable */
#define TIMER_CHCTL2_CH1P_POS								0x05UL		/** Channel 1 capture/compare function polarity */
#define TIMER_CHCTL2_CH1P_MSK								(0x01UL << TIMER_CHCTL2_CH1P_POS)		/** Channel 1 capture/compare function polarity */
#define TIMER_CHCTL2_CH1EN_POS								0x04UL		/** Channel 1 capture/compare function enable */
#define TIMER_CHCTL2_CH1EN_MSK								(0x01UL << TIMER_CHCTL2_CH1EN_POS)		/** Channel 1 capture/compare function enable */
#define TIMER_CHCTL2_CH0NP_POS								0x03UL		/** Channel 0 complementary output polarity */
#define TIMER_CHCTL2_CH0NP_MSK								(0x01UL << TIMER_CHCTL2_CH0NP_POS)		/** Channel 0 complementary output polarity */
#define TIMER_CHCTL2_CH0NEN_POS								0x02UL		/** Channel 0 complementary output enable */
#define TIMER_CHCTL2_CH0NEN_MSK								(0x01UL << TIMER_CHCTL2_CH0NEN_POS)		/** Channel 0 complementary output enable */
#define TIMER_CHCTL2_CH0P_POS								0x01UL		/** Channel 0 capture/compare function polarity */
#define TIMER_CHCTL2_CH0P_MSK								(0x01UL << TIMER_CHCTL2_CH0P_POS)		/** Channel 0 capture/compare function polarity */
#define TIMER_CHCTL2_CH0EN_POS								0x00UL		/** Channel 0 capture/compare function enable */
#define TIMER_CHCTL2_CH0EN_MSK								(0x01UL << TIMER_CHCTL2_CH0EN_POS)		/** Channel 0 capture/compare function enable */
#define TIMER_CNT_CNT_POS								0x00UL		/** current counter value */
#define TIMER_CNT_CNT_MSK								(0xFFFFUL << TIMER_CNT_CNT_POS)		/** current counter value */
#define TIMER_PSC_PSC_POS								0x00UL		/** Prescaler value of the counter clock */
#define TIMER_PSC_PSC_MSK								(0xFFFFUL << TIMER_PSC_PSC_POS)		/** Prescaler value of the counter clock */
#define TIMER_CAR_CARL_POS								0x00UL		/** Counter auto reload value */
#define TIMER_CAR_CARL_MSK								(0xFFFFUL << TIMER_CAR_CARL_POS)		/** Counter auto reload value */
#define TIMER_CREP_CREP_POS								0x00UL		/** Counter repetition value */
#define TIMER_CREP_CREP_MSK								(0xFFUL << TIMER_CREP_CREP_POS)		/** Counter repetition value */
#define TIMER_CH0CV_CH0VAL_POS								0x00UL		/** Capture or compare value of channel0 */
#define TIMER_CH0CV_CH0VAL_MSK								(0xFFFFUL << TIMER_CH0CV_CH0VAL_POS)		/** Capture or compare value of channel0 */
#define TIMER_CH1CV_CH1VAL_POS								0x00UL		/** Capture or compare value of channel1 */
#define TIMER_CH1CV_CH1VAL_MSK								(0xFFFFUL << TIMER_CH1CV_CH1VAL_POS)		/** Capture or compare value of channel1 */
#define TIMER_CH2CV_CH2VAL_POS								0x00UL		/** Capture or compare value of channel 2 */
#define TIMER_CH2CV_CH2VAL_MSK								(0xFFFFUL << TIMER_CH2CV_CH2VAL_POS)		/** Capture or compare value of channel 2 */
#define TIMER_CH3CV_CH3VAL_POS								0x00UL		/** Capture or compare value of channel 3 */
#define TIMER_CH3CV_CH3VAL_MSK								(0xFFFFUL << TIMER_CH3CV_CH3VAL_POS)		/** Capture or compare value of channel 3 */
#define TIMER_CCHP_POEN_POS								0x0FUL		/** Primary output enable */
#define TIMER_CCHP_POEN_MSK								(0x01UL << TIMER_CCHP_POEN_POS)		/** Primary output enable */
#define TIMER_CCHP_OAEN_POS								0x0EUL		/** Output automatic enable */
#define TIMER_CCHP_OAEN_MSK								(0x01UL << TIMER_CCHP_OAEN_POS)		/** Output automatic enable */
#define TIMER_CCHP_BRKP_POS								0x0DUL		/** Break polarity */
#define TIMER_CCHP_BRKP_MSK								(0x01UL << TIMER_CCHP_BRKP_POS)		/** Break polarity */
#define TIMER_CCHP_BRKEN_POS								0x0CUL		/** Break enable */
#define TIMER_CCHP_BRKEN_MSK								(0x01UL << TIMER_CCHP_BRKEN_POS)		/** Break enable */
#define TIMER_CCHP_ROS_POS								0x0BUL		/** Run mode off-state configure */
#define TIMER_CCHP_ROS_MSK								(0x01UL << TIMER_CCHP_ROS_POS)		/** Run mode off-state configure */
#define TIMER_CCHP_IOS_POS								0x0AUL		/** Idle mode off-state configure */
#define TIMER_CCHP_IOS_MSK								(0x01UL << TIMER_CCHP_IOS_POS)		/** Idle mode off-state configure */
#define TIMER_CCHP_PROT_POS								0x08UL		/** Complementary register protect control */
#define TIMER_CCHP_PROT_MSK								(0x03UL << TIMER_CCHP_PROT_POS)		/** Complementary register protect control */
#define TIMER_CCHP_DTCFG_POS								0x00UL		/** Dead time configure */
#define TIMER_CCHP_DTCFG_MSK								(0xFFUL << TIMER_CCHP_DTCFG_POS)		/** Dead time configure */
#define TIMER_DMACFG_DMATC_POS								0x08UL		/** DMA transfer count */
#define TIMER_DMACFG_DMATC_MSK								(0x1FUL << TIMER_DMACFG_DMATC_POS)		/** DMA transfer count */
#define TIMER_DMACFG_DMATA_POS								0x00UL		/** DMA transfer access start address */
#define TIMER_DMACFG_DMATA_MSK								(0x1FUL << TIMER_DMACFG_DMATA_POS)		/** DMA transfer access start address */
#define TIMER_DMATB_DMATB_POS								0x00UL		/** DMA transfer buffer */
#define TIMER_DMATB_DMATB_MSK								(0xFFFFUL << TIMER_DMATB_DMATB_POS)		/** DMA transfer buffer */
#define TIMER_SMCFG_MSM_POS								0x07UL		/** Master-slave mode */
#define TIMER_SMCFG_MSM_MSK								(0x01UL << TIMER_SMCFG_MSM_POS)		/** Master-slave mode */
#define TIMER_SMCFG_SMC_POS								0x00UL		/** Slave mode control */
#define TIMER_SMCFG_SMC_MSK								(0x07UL << TIMER_SMCFG_SMC_POS)		/** Slave mode control */
#define TIMER_INTF_CH3IF_POS								0x04UL		/** Channel 3 capture/compare interrupt enable */
#define TIMER_INTF_CH3IF_MSK								(0x01UL << TIMER_INTF_CH3IF_POS)		/** Channel 3 capture/compare interrupt enable */
#define TIMER_INTF_CH2IF_POS								0x03UL		/** Channel 2 capture/compare interrupt enable */
#define TIMER_INTF_CH2IF_MSK								(0x01UL << TIMER_INTF_CH2IF_POS)		/** Channel 2 capture/compare interrupt enable */
#define TIMER_SWEVG_UPG_POS								0x00UL		/** Update generation */
#define TIMER_SWEVG_UPG_MSK								(0x01UL << TIMER_SWEVG_UPG_POS)		/** Update generation */
#define TIMER_CHCTL0_Output_CH0COMCTL_POS								0x04UL		/**  Channel 0 compare output control */
#define TIMER_CHCTL0_Output_CH0COMCTL_MSK								(0x07UL << TIMER_CHCTL0_Output_CH0COMCTL_POS)		/**  Channel 0 compare output control */
#define TIMER_CNT_CNT_POS								0x00UL		/** counter value */
#define TIMER_CNT_CNT_MSK								(0xFFFFUL << TIMER_CNT_CNT_POS)		/** counter value */
#define TIMER_CH0CV_CH0VAL_POS								0x00UL		/** Capture or compare value of channel 0 */
#define TIMER_CH0CV_CH0VAL_MSK								(0xFFFFUL << TIMER_CH0CV_CH0VAL_POS)		/** Capture or compare value of channel 0 */
#define TIMER_CNT_CNT_POS								0x00UL		/** Low counter value */
#define TIMER_CNT_CNT_MSK								(0xFFFFUL << TIMER_CNT_CNT_POS)		/** Low counter value */
#define USART_STAT_CTSF_POS								0x09UL		/** CTS change flag */
#define USART_STAT_CTSF_MSK								(0x01UL << USART_STAT_CTSF_POS)		/** CTS change flag */
#define USART_STAT_LBDF_POS								0x08UL		/** LIN break detection flag */
#define USART_STAT_LBDF_MSK								(0x01UL << USART_STAT_LBDF_POS)		/** LIN break detection flag */
#define USART_STAT_TBE_POS								0x07UL		/** Transmit data buffer empty */
#define USART_STAT_TBE_MSK								(0x01UL << USART_STAT_TBE_POS)		/** Transmit data buffer empty */
#define USART_STAT_TC_POS								0x06UL		/** Transmission complete */
#define USART_STAT_TC_MSK								(0x01UL << USART_STAT_TC_POS)		/** Transmission complete */
#define USART_STAT_RBNE_POS								0x05UL		/** Read data buffer not empty */
#define USART_STAT_RBNE_MSK								(0x01UL << USART_STAT_RBNE_POS)		/** Read data buffer not empty */
#define USART_STAT_IDLEF_POS								0x04UL		/** IDLE frame detected flag */
#define USART_STAT_IDLEF_MSK								(0x01UL << USART_STAT_IDLEF_POS)		/** IDLE frame detected flag */
#define USART_STAT_ORERR_POS								0x03UL		/** Overrun error */
#define USART_STAT_ORERR_MSK								(0x01UL << USART_STAT_ORERR_POS)		/** Overrun error */
#define USART_STAT_NERR_POS								0x02UL		/** Noise error flag */
#define USART_STAT_NERR_MSK								(0x01UL << USART_STAT_NERR_POS)		/** Noise error flag */
#define USART_STAT_FERR_POS								0x01UL		/** Frame error flag */
#define USART_STAT_FERR_MSK								(0x01UL << USART_STAT_FERR_POS)		/** Frame error flag */
#define USART_STAT_PERR_POS								0x00UL		/** Parity error flag */
#define USART_STAT_PERR_MSK								(0x01UL << USART_STAT_PERR_POS)		/** Parity error flag */
#define USART_DATA_DATA_POS								0x00UL		/** Transmit or read data value */
#define USART_DATA_DATA_MSK								(0x1FFUL << USART_DATA_DATA_POS)		/** Transmit or read data value */
#define USART_BAUD_INTDIV_POS								0x04UL		/** Integer part of baud-rate divider */
#define USART_BAUD_INTDIV_MSK								(0xFFFUL << USART_BAUD_INTDIV_POS)		/** Integer part of baud-rate divider */
#define USART_BAUD_FRADIV_POS								0x00UL		/** Fraction part of baud-rate divider */
#define USART_BAUD_FRADIV_MSK								(0x0FUL << USART_BAUD_FRADIV_POS)		/** Fraction part of baud-rate divider */
#define USART_CTL0_UEN_POS								0x0DUL		/** USART enable */
#define USART_CTL0_UEN_MSK								(0x01UL << USART_CTL0_UEN_POS)		/** USART enable */
#define USART_CTL0_WL_POS								0x0CUL		/** Word length */
#define USART_CTL0_WL_MSK								(0x01UL << USART_CTL0_WL_POS)		/** Word length */
#define USART_CTL0_WM_POS								0x0BUL		/** Wakeup method in mute mode */
#define USART_CTL0_WM_MSK								(0x01UL << USART_CTL0_WM_POS)		/** Wakeup method in mute mode */
#define USART_CTL0_PCEN_POS								0x0AUL		/** Parity check function enable */
#define USART_CTL0_PCEN_MSK								(0x01UL << USART_CTL0_PCEN_POS)		/** Parity check function enable */
#define USART_CTL0_PM_POS								0x09UL		/** Parity mode */
#define USART_CTL0_PM_MSK								(0x01UL << USART_CTL0_PM_POS)		/** Parity mode */
#define USART_CTL0_PERRIE_POS								0x08UL		/** Parity error interrupt enable */
#define USART_CTL0_PERRIE_MSK								(0x01UL << USART_CTL0_PERRIE_POS)		/** Parity error interrupt enable */
#define USART_CTL0_TBEIE_POS								0x07UL		/** Transmitter buffer empty interrupt enable */
#define USART_CTL0_TBEIE_MSK								(0x01UL << USART_CTL0_TBEIE_POS)		/** Transmitter buffer empty interrupt enable */
#define USART_CTL0_TCIE_POS								0x06UL		/** Transmission complete interrupt enable */
#define USART_CTL0_TCIE_MSK								(0x01UL << USART_CTL0_TCIE_POS)		/** Transmission complete interrupt enable */
#define USART_CTL0_RBNEIE_POS								0x05UL		/** Read data buffer not empty interrupt and overrun error interrupt enable */
#define USART_CTL0_RBNEIE_MSK								(0x01UL << USART_CTL0_RBNEIE_POS)		/** Read data buffer not empty interrupt and overrun error interrupt enable */
#define USART_CTL0_IDLEIE_POS								0x04UL		/** IDLE line detected interrupt enable */
#define USART_CTL0_IDLEIE_MSK								(0x01UL << USART_CTL0_IDLEIE_POS)		/** IDLE line detected interrupt enable */
#define USART_CTL0_TEN_POS								0x03UL		/** Transmitter enable */
#define USART_CTL0_TEN_MSK								(0x01UL << USART_CTL0_TEN_POS)		/** Transmitter enable */
#define USART_CTL0_REN_POS								0x02UL		/** Receiver enable */
#define USART_CTL0_REN_MSK								(0x01UL << USART_CTL0_REN_POS)		/** Receiver enable */
#define USART_CTL0_RWU_POS								0x01UL		/** Receiver wakeup from mute mode */
#define USART_CTL0_RWU_MSK								(0x01UL << USART_CTL0_RWU_POS)		/** Receiver wakeup from mute mode */
#define USART_CTL0_SBKCMD_POS								0x00UL		/** Send break command */
#define USART_CTL0_SBKCMD_MSK								(0x01UL << USART_CTL0_SBKCMD_POS)		/** Send break command */
#define USART_CTL1_LMEN_POS								0x0EUL		/** LIN mode enable */
#define USART_CTL1_LMEN_MSK								(0x01UL << USART_CTL1_LMEN_POS)		/** LIN mode enable */
#define USART_CTL1_STB_POS								0x0CUL		/** STOP bits length */
#define USART_CTL1_STB_MSK								(0x03UL << USART_CTL1_STB_POS)		/** STOP bits length */
#define USART_CTL1_CKEN_POS								0x0BUL		/** CK pin enable */
#define USART_CTL1_CKEN_MSK								(0x01UL << USART_CTL1_CKEN_POS)		/** CK pin enable */
#define USART_CTL1_CPL_POS								0x0AUL		/** Clock polarity */
#define USART_CTL1_CPL_MSK								(0x01UL << USART_CTL1_CPL_POS)		/** Clock polarity */
#define USART_CTL1_CPH_POS								0x09UL		/** Clock phase */
#define USART_CTL1_CPH_MSK								(0x01UL << USART_CTL1_CPH_POS)		/** Clock phase */
#define USART_CTL1_CLEN_POS								0x08UL		/** CK Length */
#define USART_CTL1_CLEN_MSK								(0x01UL << USART_CTL1_CLEN_POS)		/** CK Length */
#define USART_CTL1_LBDIE_POS								0x06UL		/** LIN break detection interrupt enable */
#define USART_CTL1_LBDIE_MSK								(0x01UL << USART_CTL1_LBDIE_POS)		/** LIN break detection interrupt enable */
#define USART_CTL1_LBLEN_POS								0x05UL		/** LIN break frame length */
#define USART_CTL1_LBLEN_MSK								(0x01UL << USART_CTL1_LBLEN_POS)		/** LIN break frame length */
#define USART_CTL1_ADDR_POS								0x00UL		/** Address of the USART */
#define USART_CTL1_ADDR_MSK								(0x0FUL << USART_CTL1_ADDR_POS)		/** Address of the USART */
#define USART_CTL2_CTSIE_POS								0x0AUL		/** CTS interrupt enable */
#define USART_CTL2_CTSIE_MSK								(0x01UL << USART_CTL2_CTSIE_POS)		/** CTS interrupt enable */
#define USART_CTL2_CTSEN_POS								0x09UL		/** CTS enable */
#define USART_CTL2_CTSEN_MSK								(0x01UL << USART_CTL2_CTSEN_POS)		/** CTS enable */
#define USART_CTL2_RTSEN_POS								0x08UL		/** RTS enable */
#define USART_CTL2_RTSEN_MSK								(0x01UL << USART_CTL2_RTSEN_POS)		/** RTS enable */
#define USART_CTL2_DENT_POS								0x07UL		/** DMA request enable for transmission */
#define USART_CTL2_DENT_MSK								(0x01UL << USART_CTL2_DENT_POS)		/** DMA request enable for transmission */
#define USART_CTL2_DENR_POS								0x06UL		/** DMA request enable for reception */
#define USART_CTL2_DENR_MSK								(0x01UL << USART_CTL2_DENR_POS)		/** DMA request enable for reception */
#define USART_CTL2_SCEN_POS								0x05UL		/** Smartcard mode enable */
#define USART_CTL2_SCEN_MSK								(0x01UL << USART_CTL2_SCEN_POS)		/** Smartcard mode enable */
#define USART_CTL2_NKEN_POS								0x04UL		/** Smartcard NACK enable */
#define USART_CTL2_NKEN_MSK								(0x01UL << USART_CTL2_NKEN_POS)		/** Smartcard NACK enable */
#define USART_CTL2_HDEN_POS								0x03UL		/** Half-duplex selection */
#define USART_CTL2_HDEN_MSK								(0x01UL << USART_CTL2_HDEN_POS)		/** Half-duplex selection */
#define USART_CTL2_IRLP_POS								0x02UL		/** IrDA low-power */
#define USART_CTL2_IRLP_MSK								(0x01UL << USART_CTL2_IRLP_POS)		/** IrDA low-power */
#define USART_CTL2_IREN_POS								0x01UL		/** IrDA mode enable */
#define USART_CTL2_IREN_MSK								(0x01UL << USART_CTL2_IREN_POS)		/** IrDA mode enable */
#define USART_CTL2_ERRIE_POS								0x00UL		/** Error interrupt enable */
#define USART_CTL2_ERRIE_MSK								(0x01UL << USART_CTL2_ERRIE_POS)		/** Error interrupt enable */
#define USART_GP_GUAT_POS								0x08UL		/** Guard time value in Smartcard mode */
#define USART_GP_GUAT_MSK								(0xFFUL << USART_GP_GUAT_POS)		/** Guard time value in Smartcard mode */
#define USART_GP_PSC_POS								0x00UL		/** Prescaler value */
#define USART_GP_PSC_MSK								(0xFFUL << USART_GP_PSC_POS)		/** Prescaler value */
#define UART_STAT_LBDF_POS								0x08UL		/** LIN break detection flag */
#define UART_STAT_LBDF_MSK								(0x01UL << UART_STAT_LBDF_POS)		/** LIN break detection flag */
#define UART_STAT_TBE_POS								0x07UL		/** Transmit data buffer empty */
#define UART_STAT_TBE_MSK								(0x01UL << UART_STAT_TBE_POS)		/** Transmit data buffer empty */
#define UART_STAT_TC_POS								0x06UL		/** Transmission complete */
#define UART_STAT_TC_MSK								(0x01UL << UART_STAT_TC_POS)		/** Transmission complete */
#define UART_STAT_RBNE_POS								0x05UL		/** Read data buffer not empty */
#define UART_STAT_RBNE_MSK								(0x01UL << UART_STAT_RBNE_POS)		/** Read data buffer not empty */
#define UART_STAT_IDLEF_POS								0x04UL		/** IDLE frame detected flag */
#define UART_STAT_IDLEF_MSK								(0x01UL << UART_STAT_IDLEF_POS)		/** IDLE frame detected flag */
#define UART_STAT_ORERR_POS								0x03UL		/** Overrun error */
#define UART_STAT_ORERR_MSK								(0x01UL << UART_STAT_ORERR_POS)		/** Overrun error */
#define UART_STAT_NERR_POS								0x02UL		/** Noise error flag */
#define UART_STAT_NERR_MSK								(0x01UL << UART_STAT_NERR_POS)		/** Noise error flag */
#define UART_STAT_FERR_POS								0x01UL		/** Frame error flag */
#define UART_STAT_FERR_MSK								(0x01UL << UART_STAT_FERR_POS)		/** Frame error flag */
#define UART_STAT_PERR_POS								0x00UL		/** Parity error flag */
#define UART_STAT_PERR_MSK								(0x01UL << UART_STAT_PERR_POS)		/** Parity error flag */
#define UART_DATA_DATA_POS								0x00UL		/** Transmit or read data value */
#define UART_DATA_DATA_MSK								(0x1FFUL << UART_DATA_DATA_POS)		/** Transmit or read data value */
#define UART_BAUD_INTDIV_POS								0x04UL		/** Integer part of baud-rate divider */
#define UART_BAUD_INTDIV_MSK								(0xFFFUL << UART_BAUD_INTDIV_POS)		/** Integer part of baud-rate divider */
#define UART_BAUD_FRADIV_POS								0x00UL		/** Fraction part of baud-rate divider */
#define UART_BAUD_FRADIV_MSK								(0x0FUL << UART_BAUD_FRADIV_POS)		/** Fraction part of baud-rate divider */
#define UART_CTL0_UEN_POS								0x0DUL		/** USART enable */
#define UART_CTL0_UEN_MSK								(0x01UL << UART_CTL0_UEN_POS)		/** USART enable */
#define UART_CTL0_WL_POS								0x0CUL		/** Word length */
#define UART_CTL0_WL_MSK								(0x01UL << UART_CTL0_WL_POS)		/** Word length */
#define UART_CTL0_WM_POS								0x0BUL		/** Wakeup method in mute mode */
#define UART_CTL0_WM_MSK								(0x01UL << UART_CTL0_WM_POS)		/** Wakeup method in mute mode */
#define UART_CTL0_PCEN_POS								0x0AUL		/** Parity check function enable */
#define UART_CTL0_PCEN_MSK								(0x01UL << UART_CTL0_PCEN_POS)		/** Parity check function enable */
#define UART_CTL0_PM_POS								0x09UL		/** Parity mode */
#define UART_CTL0_PM_MSK								(0x01UL << UART_CTL0_PM_POS)		/** Parity mode */
#define UART_CTL0_PERRIE_POS								0x08UL		/** Parity error interrupt enable */
#define UART_CTL0_PERRIE_MSK								(0x01UL << UART_CTL0_PERRIE_POS)		/** Parity error interrupt enable */
#define UART_CTL0_TBEIE_POS								0x07UL		/** Transmitter buffer empty interrupt enable */
#define UART_CTL0_TBEIE_MSK								(0x01UL << UART_CTL0_TBEIE_POS)		/** Transmitter buffer empty interrupt enable */
#define UART_CTL0_TCIE_POS								0x06UL		/** Transmission complete interrupt enable */
#define UART_CTL0_TCIE_MSK								(0x01UL << UART_CTL0_TCIE_POS)		/** Transmission complete interrupt enable */
#define UART_CTL0_RBNEIE_POS								0x05UL		/** Read data buffer not empty interrupt and overrun error interrupt enable */
#define UART_CTL0_RBNEIE_MSK								(0x01UL << UART_CTL0_RBNEIE_POS)		/** Read data buffer not empty interrupt and overrun error interrupt enable */
#define UART_CTL0_IDLEIE_POS								0x04UL		/** IDLE line detected interrupt enable */
#define UART_CTL0_IDLEIE_MSK								(0x01UL << UART_CTL0_IDLEIE_POS)		/** IDLE line detected interrupt enable */
#define UART_CTL0_TEN_POS								0x03UL		/** Transmitter enable */
#define UART_CTL0_TEN_MSK								(0x01UL << UART_CTL0_TEN_POS)		/** Transmitter enable */
#define UART_CTL0_REN_POS								0x02UL		/** Receiver enable */
#define UART_CTL0_REN_MSK								(0x01UL << UART_CTL0_REN_POS)		/** Receiver enable */
#define UART_CTL0_RWU_POS								0x01UL		/** Receiver wakeup from mute mode */
#define UART_CTL0_RWU_MSK								(0x01UL << UART_CTL0_RWU_POS)		/** Receiver wakeup from mute mode */
#define UART_CTL0_SBKCMD_POS								0x00UL		/** Send break command */
#define UART_CTL0_SBKCMD_MSK								(0x01UL << UART_CTL0_SBKCMD_POS)		/** Send break command */
#define UART_CTL1_LMEN_POS								0x0EUL		/** LIN mode enable */
#define UART_CTL1_LMEN_MSK								(0x01UL << UART_CTL1_LMEN_POS)		/** LIN mode enable */
#define UART_CTL1_STB_POS								0x0CUL		/** STOP bits length */
#define UART_CTL1_STB_MSK								(0x03UL << UART_CTL1_STB_POS)		/** STOP bits length */
#define UART_CTL1_LBDIE_POS								0x06UL		/** LIN break detection interrupt enable */
#define UART_CTL1_LBDIE_MSK								(0x01UL << UART_CTL1_LBDIE_POS)		/** LIN break detection interrupt enable */
#define UART_CTL1_LBLEN_POS								0x05UL		/** LIN break frame length */
#define UART_CTL1_LBLEN_MSK								(0x01UL << UART_CTL1_LBLEN_POS)		/** LIN break frame length */
#define UART_CTL1_ADDR_POS								0x00UL		/** Address of the USART */
#define UART_CTL1_ADDR_MSK								(0x0FUL << UART_CTL1_ADDR_POS)		/** Address of the USART */
#define UART_CTL2_DENT_POS								0x07UL		/** DMA request enable for transmission */
#define UART_CTL2_DENT_MSK								(0x01UL << UART_CTL2_DENT_POS)		/** DMA request enable for transmission */
#define UART_CTL2_DENR_POS								0x06UL		/** DMA request enable for reception */
#define UART_CTL2_DENR_MSK								(0x01UL << UART_CTL2_DENR_POS)		/** DMA request enable for reception */
#define UART_CTL2_HDEN_POS								0x03UL		/** Half-duplex selection */
#define UART_CTL2_HDEN_MSK								(0x01UL << UART_CTL2_HDEN_POS)		/** Half-duplex selection */
#define UART_CTL2_IRLP_POS								0x02UL		/** IrDA low-power */
#define UART_CTL2_IRLP_MSK								(0x01UL << UART_CTL2_IRLP_POS)		/** IrDA low-power */
#define UART_CTL2_IREN_POS								0x01UL		/** IrDA mode enable */
#define UART_CTL2_IREN_MSK								(0x01UL << UART_CTL2_IREN_POS)		/** IrDA mode enable */
#define UART_CTL2_ERRIE_POS								0x00UL		/** Error interrupt enable */
#define UART_CTL2_ERRIE_MSK								(0x01UL << UART_CTL2_ERRIE_POS)		/** Error interrupt enable */
#define UART_GP_PSC_POS								0x00UL		/** Prescaler value */
#define UART_GP_PSC_MSK								(0xFFUL << UART_GP_PSC_POS)		/** Prescaler value */
#define USBFS_GOTGCS_SRPS_POS								0x00UL		/** SRP success */
#define USBFS_GOTGCS_SRPS_MSK								(0x01UL << USBFS_GOTGCS_SRPS_POS)		/** SRP success */
#define USBFS_GOTGCS_SRPREQ_POS								0x01UL		/** SRP request */
#define USBFS_GOTGCS_SRPREQ_MSK								(0x01UL << USBFS_GOTGCS_SRPREQ_POS)		/** SRP request */
#define USBFS_GOTGCS_HNPS_POS								0x08UL		/** Host success */
#define USBFS_GOTGCS_HNPS_MSK								(0x01UL << USBFS_GOTGCS_HNPS_POS)		/** Host success */
#define USBFS_GOTGCS_HNPREQ_POS								0x09UL		/** HNP request */
#define USBFS_GOTGCS_HNPREQ_MSK								(0x01UL << USBFS_GOTGCS_HNPREQ_POS)		/** HNP request */
#define USBFS_GOTGCS_HHNPEN_POS								0x0AUL		/** Host HNP enable */
#define USBFS_GOTGCS_HHNPEN_MSK								(0x01UL << USBFS_GOTGCS_HHNPEN_POS)		/** Host HNP enable */
#define USBFS_GOTGCS_DHNPEN_POS								0x0BUL		/** Device HNP enabled */
#define USBFS_GOTGCS_DHNPEN_MSK								(0x01UL << USBFS_GOTGCS_DHNPEN_POS)		/** Device HNP enabled */
#define USBFS_GOTGCS_IDPS_POS								0x10UL		/** ID pin status */
#define USBFS_GOTGCS_IDPS_MSK								(0x01UL << USBFS_GOTGCS_IDPS_POS)		/** ID pin status */
#define USBFS_GOTGCS_DI_POS								0x11UL		/** Debounce interval */
#define USBFS_GOTGCS_DI_MSK								(0x01UL << USBFS_GOTGCS_DI_POS)		/** Debounce interval */
#define USBFS_GOTGCS_ASV_POS								0x12UL		/** A-session valid */
#define USBFS_GOTGCS_ASV_MSK								(0x01UL << USBFS_GOTGCS_ASV_POS)		/** A-session valid */
#define USBFS_GOTGCS_BSV_POS								0x13UL		/** B-session valid */
#define USBFS_GOTGCS_BSV_MSK								(0x01UL << USBFS_GOTGCS_BSV_POS)		/** B-session valid */
#define USBFS_GOTGINTF_SESEND_POS								0x02UL		/** Session end  */
#define USBFS_GOTGINTF_SESEND_MSK								(0x01UL << USBFS_GOTGINTF_SESEND_POS)		/** Session end  */
#define USBFS_GOTGINTF_SRPEND_POS								0x08UL		/** Session request success status change */
#define USBFS_GOTGINTF_SRPEND_MSK								(0x01UL << USBFS_GOTGINTF_SRPEND_POS)		/** Session request success status change */
#define USBFS_GOTGINTF_HNPEND_POS								0x09UL		/** HNP end */
#define USBFS_GOTGINTF_HNPEND_MSK								(0x01UL << USBFS_GOTGINTF_HNPEND_POS)		/** HNP end */
#define USBFS_GOTGINTF_HNPDET_POS								0x11UL		/** Host negotiation request detected */
#define USBFS_GOTGINTF_HNPDET_MSK								(0x01UL << USBFS_GOTGINTF_HNPDET_POS)		/** Host negotiation request detected */
#define USBFS_GOTGINTF_ADTO_POS								0x12UL		/** A-device timeout */
#define USBFS_GOTGINTF_ADTO_MSK								(0x01UL << USBFS_GOTGINTF_ADTO_POS)		/** A-device timeout */
#define USBFS_GOTGINTF_DF_POS								0x13UL		/** Debounce finish */
#define USBFS_GOTGINTF_DF_MSK								(0x01UL << USBFS_GOTGINTF_DF_POS)		/** Debounce finish */
#define USBFS_GAHBCS_GINTEN_POS								0x00UL		/** Global interrupt enable */
#define USBFS_GAHBCS_GINTEN_MSK								(0x01UL << USBFS_GAHBCS_GINTEN_POS)		/** Global interrupt enable */
#define USBFS_GAHBCS_TXFTH_POS								0x07UL		/** Tx FIFO threshold */
#define USBFS_GAHBCS_TXFTH_MSK								(0x01UL << USBFS_GAHBCS_TXFTH_POS)		/** Tx FIFO threshold */
#define USBFS_GAHBCS_PTXFTH_POS								0x08UL		/** Periodic Tx FIFO threshold */
#define USBFS_GAHBCS_PTXFTH_MSK								(0x01UL << USBFS_GAHBCS_PTXFTH_POS)		/** Periodic Tx FIFO threshold */
#define USBFS_GUSBCS_TOC_POS								0x00UL		/** Timeout calibration */
#define USBFS_GUSBCS_TOC_MSK								(0x07UL << USBFS_GUSBCS_TOC_POS)		/** Timeout calibration */
#define USBFS_GUSBCS_SRPCEN_POS								0x08UL		/** SRP capability enable */
#define USBFS_GUSBCS_SRPCEN_MSK								(0x01UL << USBFS_GUSBCS_SRPCEN_POS)		/** SRP capability enable */
#define USBFS_GUSBCS_HNPCEN_POS								0x09UL		/** HNP capability enable */
#define USBFS_GUSBCS_HNPCEN_MSK								(0x01UL << USBFS_GUSBCS_HNPCEN_POS)		/** HNP capability enable */
#define USBFS_GUSBCS_UTT_POS								0x0AUL		/** USB turnaround time */
#define USBFS_GUSBCS_UTT_MSK								(0x0FUL << USBFS_GUSBCS_UTT_POS)		/** USB turnaround time */
#define USBFS_GUSBCS_FHM_POS								0x1DUL		/** Force host mode */
#define USBFS_GUSBCS_FHM_MSK								(0x01UL << USBFS_GUSBCS_FHM_POS)		/** Force host mode */
#define USBFS_GUSBCS_FDM_POS								0x1EUL		/** Force device mode */
#define USBFS_GUSBCS_FDM_MSK								(0x01UL << USBFS_GUSBCS_FDM_POS)		/** Force device mode */
#define USBFS_GRSTCTL_CSRST_POS								0x00UL		/** Core soft reset */
#define USBFS_GRSTCTL_CSRST_MSK								(0x01UL << USBFS_GRSTCTL_CSRST_POS)		/** Core soft reset */
#define USBFS_GRSTCTL_HCSRST_POS								0x01UL		/** HCLK soft reset */
#define USBFS_GRSTCTL_HCSRST_MSK								(0x01UL << USBFS_GRSTCTL_HCSRST_POS)		/** HCLK soft reset */
#define USBFS_GRSTCTL_HFCRST_POS								0x02UL		/** Host frame counter reset */
#define USBFS_GRSTCTL_HFCRST_MSK								(0x01UL << USBFS_GRSTCTL_HFCRST_POS)		/** Host frame counter reset */
#define USBFS_GRSTCTL_RXFF_POS								0x04UL		/** RxFIFO flush */
#define USBFS_GRSTCTL_RXFF_MSK								(0x01UL << USBFS_GRSTCTL_RXFF_POS)		/** RxFIFO flush */
#define USBFS_GRSTCTL_TXFF_POS								0x05UL		/** TxFIFO flush */
#define USBFS_GRSTCTL_TXFF_MSK								(0x01UL << USBFS_GRSTCTL_TXFF_POS)		/** TxFIFO flush */
#define USBFS_GRSTCTL_TXFNUM_POS								0x06UL		/** TxFIFO number */
#define USBFS_GRSTCTL_TXFNUM_MSK								(0x1FUL << USBFS_GRSTCTL_TXFNUM_POS)		/** TxFIFO number */
#define USBFS_GINTF_COPM_POS								0x00UL		/** Current operation mode */
#define USBFS_GINTF_COPM_MSK								(0x01UL << USBFS_GINTF_COPM_POS)		/** Current operation mode */
#define USBFS_GINTF_MFIF_POS								0x01UL		/** Mode fault interrupt flag */
#define USBFS_GINTF_MFIF_MSK								(0x01UL << USBFS_GINTF_MFIF_POS)		/** Mode fault interrupt flag */
#define USBFS_GINTF_OTGIF_POS								0x02UL		/** OTG interrupt flag */
#define USBFS_GINTF_OTGIF_MSK								(0x01UL << USBFS_GINTF_OTGIF_POS)		/** OTG interrupt flag */
#define USBFS_GINTF_SOF_POS								0x03UL		/** Start of frame */
#define USBFS_GINTF_SOF_MSK								(0x01UL << USBFS_GINTF_SOF_POS)		/** Start of frame */
#define USBFS_GINTF_RXFNEIF_POS								0x04UL		/** RxFIFO non-empty interrupt flag */
#define USBFS_GINTF_RXFNEIF_MSK								(0x01UL << USBFS_GINTF_RXFNEIF_POS)		/** RxFIFO non-empty interrupt flag */
#define USBFS_GINTF_NPTXFEIF_POS								0x05UL		/** Non-periodic TxFIFO empty interrupt flag */
#define USBFS_GINTF_NPTXFEIF_MSK								(0x01UL << USBFS_GINTF_NPTXFEIF_POS)		/** Non-periodic TxFIFO empty interrupt flag */
#define USBFS_GINTF_GNPINAK_POS								0x06UL		/** Global Non-Periodic IN NAK effective */
#define USBFS_GINTF_GNPINAK_MSK								(0x01UL << USBFS_GINTF_GNPINAK_POS)		/** Global Non-Periodic IN NAK effective */
#define USBFS_GINTF_GONAK_POS								0x07UL		/** Global OUT NAK effective */
#define USBFS_GINTF_GONAK_MSK								(0x01UL << USBFS_GINTF_GONAK_POS)		/** Global OUT NAK effective */
#define USBFS_GINTF_ESP_POS								0x0AUL		/** Early suspend */
#define USBFS_GINTF_ESP_MSK								(0x01UL << USBFS_GINTF_ESP_POS)		/** Early suspend */
#define USBFS_GINTF_SP_POS								0x0BUL		/** USB suspend */
#define USBFS_GINTF_SP_MSK								(0x01UL << USBFS_GINTF_SP_POS)		/** USB suspend */
#define USBFS_GINTF_RST_POS								0x0CUL		/** USB reset */
#define USBFS_GINTF_RST_MSK								(0x01UL << USBFS_GINTF_RST_POS)		/** USB reset */
#define USBFS_GINTF_ENUMF_POS								0x0DUL		/** Enumeration finished */
#define USBFS_GINTF_ENUMF_MSK								(0x01UL << USBFS_GINTF_ENUMF_POS)		/** Enumeration finished */
#define USBFS_GINTF_ISOOPDIF_POS								0x0EUL		/** Isochronous OUT packet dropped interrupt */
#define USBFS_GINTF_ISOOPDIF_MSK								(0x01UL << USBFS_GINTF_ISOOPDIF_POS)		/** Isochronous OUT packet dropped interrupt */
#define USBFS_GINTF_EOPFIF_POS								0x0FUL		/** End of periodic frame interrupt flag */
#define USBFS_GINTF_EOPFIF_MSK								(0x01UL << USBFS_GINTF_EOPFIF_POS)		/** End of periodic frame interrupt flag */
#define USBFS_GINTF_IEPIF_POS								0x12UL		/** IN endpoint interrupt flag */
#define USBFS_GINTF_IEPIF_MSK								(0x01UL << USBFS_GINTF_IEPIF_POS)		/** IN endpoint interrupt flag */
#define USBFS_GINTF_OEPIF_POS								0x13UL		/** OUT endpoint interrupt flag */
#define USBFS_GINTF_OEPIF_MSK								(0x01UL << USBFS_GINTF_OEPIF_POS)		/** OUT endpoint interrupt flag */
#define USBFS_GINTF_ISOINCIF_POS								0x14UL		/** Isochronous IN transfer Not Complete Interrupt Flag */
#define USBFS_GINTF_ISOINCIF_MSK								(0x01UL << USBFS_GINTF_ISOINCIF_POS)		/** Isochronous IN transfer Not Complete Interrupt Flag */
#define USBFS_GINTF_PXNCIF_ISOONCIF_POS								0x15UL		/** periodic transfer not complete interrupt flag(Host mode)/isochronous OUT transfer not complete interrupt flag(Device mode) */
#define USBFS_GINTF_PXNCIF_ISOONCIF_MSK								(0x01UL << USBFS_GINTF_PXNCIF_ISOONCIF_POS)		/** periodic transfer not complete interrupt flag(Host mode)/isochronous OUT transfer not complete interrupt flag(Device mode) */
#define USBFS_GINTF_HPIF_POS								0x18UL		/** Host port interrupt flag */
#define USBFS_GINTF_HPIF_MSK								(0x01UL << USBFS_GINTF_HPIF_POS)		/** Host port interrupt flag */
#define USBFS_GINTF_HCIF_POS								0x19UL		/** Host channels interrupt flag */
#define USBFS_GINTF_HCIF_MSK								(0x01UL << USBFS_GINTF_HCIF_POS)		/** Host channels interrupt flag */
#define USBFS_GINTF_PTXFEIF_POS								0x1AUL		/** Periodic TxFIFO empty interrupt flag */
#define USBFS_GINTF_PTXFEIF_MSK								(0x01UL << USBFS_GINTF_PTXFEIF_POS)		/** Periodic TxFIFO empty interrupt flag */
#define USBFS_GINTF_IDPSC_POS								0x1CUL		/** ID pin status change */
#define USBFS_GINTF_IDPSC_MSK								(0x01UL << USBFS_GINTF_IDPSC_POS)		/** ID pin status change */
#define USBFS_GINTF_DISCIF_POS								0x1DUL		/** Disconnect interrupt flag */
#define USBFS_GINTF_DISCIF_MSK								(0x01UL << USBFS_GINTF_DISCIF_POS)		/** Disconnect interrupt flag */
#define USBFS_GINTF_SESIF_POS								0x1EUL		/** Session interrupt flag */
#define USBFS_GINTF_SESIF_MSK								(0x01UL << USBFS_GINTF_SESIF_POS)		/** Session interrupt flag */
#define USBFS_GINTF_WKUPIF_POS								0x1FUL		/** Wakeup interrupt flag */
#define USBFS_GINTF_WKUPIF_MSK								(0x01UL << USBFS_GINTF_WKUPIF_POS)		/** Wakeup interrupt flag */
#define USBFS_GINTEN_MFIE_POS								0x01UL		/** Mode fault interrupt enable */
#define USBFS_GINTEN_MFIE_MSK								(0x01UL << USBFS_GINTEN_MFIE_POS)		/** Mode fault interrupt enable */
#define USBFS_GINTEN_OTGIE_POS								0x02UL		/** OTG interrupt enable  */
#define USBFS_GINTEN_OTGIE_MSK								(0x01UL << USBFS_GINTEN_OTGIE_POS)		/** OTG interrupt enable  */
#define USBFS_GINTEN_SOFIE_POS								0x03UL		/** Start of frame interrupt enable */
#define USBFS_GINTEN_SOFIE_MSK								(0x01UL << USBFS_GINTEN_SOFIE_POS)		/** Start of frame interrupt enable */
#define USBFS_GINTEN_RXFNEIE_POS								0x04UL		/** Receive FIFO non-empty interrupt enable */
#define USBFS_GINTEN_RXFNEIE_MSK								(0x01UL << USBFS_GINTEN_RXFNEIE_POS)		/** Receive FIFO non-empty interrupt enable */
#define USBFS_GINTEN_NPTXFEIE_POS								0x05UL		/** Non-periodic TxFIFO empty interrupt enable */
#define USBFS_GINTEN_NPTXFEIE_MSK								(0x01UL << USBFS_GINTEN_NPTXFEIE_POS)		/** Non-periodic TxFIFO empty interrupt enable */
#define USBFS_GINTEN_GNPINAKIE_POS								0x06UL		/** Global non-periodic IN NAK effective interrupt enable */
#define USBFS_GINTEN_GNPINAKIE_MSK								(0x01UL << USBFS_GINTEN_GNPINAKIE_POS)		/** Global non-periodic IN NAK effective interrupt enable */
#define USBFS_GINTEN_GONAKIE_POS								0x07UL		/** Global OUT NAK effective interrupt enable */
#define USBFS_GINTEN_GONAKIE_MSK								(0x01UL << USBFS_GINTEN_GONAKIE_POS)		/** Global OUT NAK effective interrupt enable */
#define USBFS_GINTEN_ESPIE_POS								0x0AUL		/** Early suspend interrupt enable */
#define USBFS_GINTEN_ESPIE_MSK								(0x01UL << USBFS_GINTEN_ESPIE_POS)		/** Early suspend interrupt enable */
#define USBFS_GINTEN_SPIE_POS								0x0BUL		/** USB suspend interrupt enable */
#define USBFS_GINTEN_SPIE_MSK								(0x01UL << USBFS_GINTEN_SPIE_POS)		/** USB suspend interrupt enable */
#define USBFS_GINTEN_RSTIE_POS								0x0CUL		/** USB reset interrupt enable */
#define USBFS_GINTEN_RSTIE_MSK								(0x01UL << USBFS_GINTEN_RSTIE_POS)		/** USB reset interrupt enable */
#define USBFS_GINTEN_ENUMFIE_POS								0x0DUL		/** Enumeration finish interrupt enable */
#define USBFS_GINTEN_ENUMFIE_MSK								(0x01UL << USBFS_GINTEN_ENUMFIE_POS)		/** Enumeration finish interrupt enable */
#define USBFS_GINTEN_ISOOPDIE_POS								0x0EUL		/** Isochronous OUT packet dropped interrupt enable */
#define USBFS_GINTEN_ISOOPDIE_MSK								(0x01UL << USBFS_GINTEN_ISOOPDIE_POS)		/** Isochronous OUT packet dropped interrupt enable */
#define USBFS_GINTEN_EOPFIE_POS								0x0FUL		/** End of periodic frame interrupt enable */
#define USBFS_GINTEN_EOPFIE_MSK								(0x01UL << USBFS_GINTEN_EOPFIE_POS)		/** End of periodic frame interrupt enable */
#define USBFS_GINTEN_IEPIE_POS								0x12UL		/** IN endpoints interrupt enable */
#define USBFS_GINTEN_IEPIE_MSK								(0x01UL << USBFS_GINTEN_IEPIE_POS)		/** IN endpoints interrupt enable */
#define USBFS_GINTEN_OEPIE_POS								0x13UL		/** OUT endpoints interrupt enable */
#define USBFS_GINTEN_OEPIE_MSK								(0x01UL << USBFS_GINTEN_OEPIE_POS)		/** OUT endpoints interrupt enable */
#define USBFS_GINTEN_ISOINCIE_POS								0x14UL		/** isochronous IN transfer not complete interrupt enable */
#define USBFS_GINTEN_ISOINCIE_MSK								(0x01UL << USBFS_GINTEN_ISOINCIE_POS)		/** isochronous IN transfer not complete interrupt enable */
#define USBFS_GINTEN_PXNCIE_ISOONCIE_POS								0x15UL		/** periodic transfer not compelete Interrupt enable(Host mode)/isochronous OUT transfer not complete interrupt enable(Device mode) */
#define USBFS_GINTEN_PXNCIE_ISOONCIE_MSK								(0x01UL << USBFS_GINTEN_PXNCIE_ISOONCIE_POS)		/** periodic transfer not compelete Interrupt enable(Host mode)/isochronous OUT transfer not complete interrupt enable(Device mode) */
#define USBFS_GINTEN_HPIE_POS								0x18UL		/** Host port interrupt enable */
#define USBFS_GINTEN_HPIE_MSK								(0x01UL << USBFS_GINTEN_HPIE_POS)		/** Host port interrupt enable */
#define USBFS_GINTEN_HCIE_POS								0x19UL		/** Host channels interrupt enable */
#define USBFS_GINTEN_HCIE_MSK								(0x01UL << USBFS_GINTEN_HCIE_POS)		/** Host channels interrupt enable */
#define USBFS_GINTEN_PTXFEIE_POS								0x1AUL		/** Periodic TxFIFO empty interrupt enable */
#define USBFS_GINTEN_PTXFEIE_MSK								(0x01UL << USBFS_GINTEN_PTXFEIE_POS)		/** Periodic TxFIFO empty interrupt enable */
#define USBFS_GINTEN_IDPSCIE_POS								0x1CUL		/** ID pin status change interrupt enable */
#define USBFS_GINTEN_IDPSCIE_MSK								(0x01UL << USBFS_GINTEN_IDPSCIE_POS)		/** ID pin status change interrupt enable */
#define USBFS_GINTEN_DISCIE_POS								0x1DUL		/** Disconnect interrupt enable */
#define USBFS_GINTEN_DISCIE_MSK								(0x01UL << USBFS_GINTEN_DISCIE_POS)		/** Disconnect interrupt enable */
#define USBFS_GINTEN_SESIE_POS								0x1EUL		/** Session interrupt enable */
#define USBFS_GINTEN_SESIE_MSK								(0x01UL << USBFS_GINTEN_SESIE_POS)		/** Session interrupt enable */
#define USBFS_GINTEN_WKUPIE_POS								0x1FUL		/** Wakeup interrupt enable */
#define USBFS_GINTEN_WKUPIE_MSK								(0x01UL << USBFS_GINTEN_WKUPIE_POS)		/** Wakeup interrupt enable */
#define USBFS_GRSTATR_Device_EPNUM_POS								0x00UL		/** Endpoint number */
#define USBFS_GRSTATR_Device_EPNUM_MSK								(0x0FUL << USBFS_GRSTATR_Device_EPNUM_POS)		/** Endpoint number */
#define USBFS_GRSTATR_Device_BCOUNT_POS								0x04UL		/** Byte count */
#define USBFS_GRSTATR_Device_BCOUNT_MSK								(0x7FFUL << USBFS_GRSTATR_Device_BCOUNT_POS)		/** Byte count */
#define USBFS_GRSTATR_Device_DPID_POS								0x0FUL		/** Data PID */
#define USBFS_GRSTATR_Device_DPID_MSK								(0x03UL << USBFS_GRSTATR_Device_DPID_POS)		/** Data PID */
#define USBFS_GRSTATR_Device_RPCKST_POS								0x11UL		/** Recieve packet status */
#define USBFS_GRSTATR_Device_RPCKST_MSK								(0x0FUL << USBFS_GRSTATR_Device_RPCKST_POS)		/** Recieve packet status */
#define USBFS_GRSTATR_Host_CNUM_POS								0x00UL		/** Channel number */
#define USBFS_GRSTATR_Host_CNUM_MSK								(0x0FUL << USBFS_GRSTATR_Host_CNUM_POS)		/** Channel number */
#define USBFS_GRSTATR_Host_BCOUNT_POS								0x04UL		/** Byte count */
#define USBFS_GRSTATR_Host_BCOUNT_MSK								(0x7FFUL << USBFS_GRSTATR_Host_BCOUNT_POS)		/** Byte count */
#define USBFS_GRSTATR_Host_DPID_POS								0x0FUL		/** Data PID */
#define USBFS_GRSTATR_Host_DPID_MSK								(0x03UL << USBFS_GRSTATR_Host_DPID_POS)		/** Data PID */
#define USBFS_GRSTATR_Host_RPCKST_POS								0x11UL		/** Reivece packet status */
#define USBFS_GRSTATR_Host_RPCKST_MSK								(0x0FUL << USBFS_GRSTATR_Host_RPCKST_POS)		/** Reivece packet status */
#define USBFS_GRSTATP_Device_EPNUM_POS								0x00UL		/** Endpoint number */
#define USBFS_GRSTATP_Device_EPNUM_MSK								(0x0FUL << USBFS_GRSTATP_Device_EPNUM_POS)		/** Endpoint number */
#define USBFS_GRSTATP_Device_BCOUNT_POS								0x04UL		/** Byte count */
#define USBFS_GRSTATP_Device_BCOUNT_MSK								(0x7FFUL << USBFS_GRSTATP_Device_BCOUNT_POS)		/** Byte count */
#define USBFS_GRSTATP_Device_DPID_POS								0x0FUL		/** Data PID */
#define USBFS_GRSTATP_Device_DPID_MSK								(0x03UL << USBFS_GRSTATP_Device_DPID_POS)		/** Data PID */
#define USBFS_GRSTATP_Device_RPCKST_POS								0x11UL		/** Recieve packet status */
#define USBFS_GRSTATP_Device_RPCKST_MSK								(0x0FUL << USBFS_GRSTATP_Device_RPCKST_POS)		/** Recieve packet status */
#define USBFS_GRSTATP_Host_CNUM_POS								0x00UL		/** Channel number */
#define USBFS_GRSTATP_Host_CNUM_MSK								(0x0FUL << USBFS_GRSTATP_Host_CNUM_POS)		/** Channel number */
#define USBFS_GRSTATP_Host_BCOUNT_POS								0x04UL		/** Byte count */
#define USBFS_GRSTATP_Host_BCOUNT_MSK								(0x7FFUL << USBFS_GRSTATP_Host_BCOUNT_POS)		/** Byte count */
#define USBFS_GRSTATP_Host_DPID_POS								0x0FUL		/** Data PID */
#define USBFS_GRSTATP_Host_DPID_MSK								(0x03UL << USBFS_GRSTATP_Host_DPID_POS)		/** Data PID */
#define USBFS_GRSTATP_Host_RPCKST_POS								0x11UL		/** Reivece packet status */
#define USBFS_GRSTATP_Host_RPCKST_MSK								(0x0FUL << USBFS_GRSTATP_Host_RPCKST_POS)		/** Reivece packet status */
#define USBFS_GRFLEN_RXFD_POS								0x00UL		/** Rx FIFO depth */
#define USBFS_GRFLEN_RXFD_MSK								(0xFFFFUL << USBFS_GRFLEN_RXFD_POS)		/** Rx FIFO depth */
#define USBFS_HNPTFLEN_HNPTXRSAR_POS								0x00UL		/** host non-periodic transmit Tx RAM start address */
#define USBFS_HNPTFLEN_HNPTXRSAR_MSK								(0xFFFFUL << USBFS_HNPTFLEN_HNPTXRSAR_POS)		/** host non-periodic transmit Tx RAM start address */
#define USBFS_HNPTFLEN_HNPTXFD_POS								0x10UL		/** host non-periodic TxFIFO depth */
#define USBFS_HNPTFLEN_HNPTXFD_MSK								(0xFFFFUL << USBFS_HNPTFLEN_HNPTXFD_POS)		/** host non-periodic TxFIFO depth */
#define USBFS_DIEP0TFLEN_IEP0TXFD_POS								0x10UL		/** in endpoint 0 Tx FIFO depth */
#define USBFS_DIEP0TFLEN_IEP0TXFD_MSK								(0xFFFFUL << USBFS_DIEP0TFLEN_IEP0TXFD_POS)		/** in endpoint 0 Tx FIFO depth */
#define USBFS_DIEP0TFLEN_IEP0TXRSAR_POS								0x00UL		/** in endpoint 0 Tx RAM start address */
#define USBFS_DIEP0TFLEN_IEP0TXRSAR_MSK								(0xFFFFUL << USBFS_DIEP0TFLEN_IEP0TXRSAR_POS)		/** in endpoint 0 Tx RAM start address */
#define USBFS_HNPTFQSTAT_NPTXFS_POS								0x00UL		/** Non-periodic TxFIFO space */
#define USBFS_HNPTFQSTAT_NPTXFS_MSK								(0xFFFFUL << USBFS_HNPTFQSTAT_NPTXFS_POS)		/** Non-periodic TxFIFO space */
#define USBFS_HNPTFQSTAT_NPTXRQS_POS								0x10UL		/** Non-periodic transmit request queue space  */
#define USBFS_HNPTFQSTAT_NPTXRQS_MSK								(0xFFUL << USBFS_HNPTFQSTAT_NPTXRQS_POS)		/** Non-periodic transmit request queue space  */
#define USBFS_HNPTFQSTAT_NPTXRQTOP_POS								0x18UL		/** Top of the non-periodic transmit request queue */
#define USBFS_HNPTFQSTAT_NPTXRQTOP_MSK								(0x7FUL << USBFS_HNPTFQSTAT_NPTXRQTOP_POS)		/** Top of the non-periodic transmit request queue */
#define USBFS_GCCFG_PWRON_POS								0x10UL		/** Power on */
#define USBFS_GCCFG_PWRON_MSK								(0x01UL << USBFS_GCCFG_PWRON_POS)		/** Power on */
#define USBFS_GCCFG_VBUSACEN_POS								0x12UL		/** The VBUS A-device Comparer enable */
#define USBFS_GCCFG_VBUSACEN_MSK								(0x01UL << USBFS_GCCFG_VBUSACEN_POS)		/** The VBUS A-device Comparer enable */
#define USBFS_GCCFG_VBUSBCEN_POS								0x13UL		/** The VBUS B-device Comparer enable */
#define USBFS_GCCFG_VBUSBCEN_MSK								(0x01UL << USBFS_GCCFG_VBUSBCEN_POS)		/** The VBUS B-device Comparer enable */
#define USBFS_GCCFG_SOFOEN_POS								0x14UL		/** SOF output enable */
#define USBFS_GCCFG_SOFOEN_MSK								(0x01UL << USBFS_GCCFG_SOFOEN_POS)		/** SOF output enable */
#define USBFS_GCCFG_VBUSIG_POS								0x15UL		/** VBUS ignored */
#define USBFS_GCCFG_VBUSIG_MSK								(0x01UL << USBFS_GCCFG_VBUSIG_POS)		/** VBUS ignored */
#define USBFS_CID_CID_POS								0x00UL		/** Core ID */
#define USBFS_CID_CID_MSK								(0xFFFFFFFFUL << USBFS_CID_CID_POS)		/** Core ID */
#define USBFS_HPTFLEN_HPTXFSAR_POS								0x00UL		/** Host periodic TxFIFO start address */
#define USBFS_HPTFLEN_HPTXFSAR_MSK								(0xFFFFUL << USBFS_HPTFLEN_HPTXFSAR_POS)		/** Host periodic TxFIFO start address */
#define USBFS_HPTFLEN_HPTXFD_POS								0x10UL		/** Host periodic TxFIFO depth */
#define USBFS_HPTFLEN_HPTXFD_MSK								(0xFFFFUL << USBFS_HPTFLEN_HPTXFD_POS)		/** Host periodic TxFIFO depth */
#define USBFS_DIEP1TFLEN_IEPTXRSAR_POS								0x00UL		/** IN endpoint FIFO transmit RAM start address */
#define USBFS_DIEP1TFLEN_IEPTXRSAR_MSK								(0xFFFFUL << USBFS_DIEP1TFLEN_IEPTXRSAR_POS)		/** IN endpoint FIFO transmit RAM start address */
#define USBFS_DIEP1TFLEN_IEPTXFD_POS								0x10UL		/** IN endpoint TxFIFO depth */
#define USBFS_DIEP1TFLEN_IEPTXFD_MSK								(0xFFFFUL << USBFS_DIEP1TFLEN_IEPTXFD_POS)		/** IN endpoint TxFIFO depth */
#define USBFS_DIEP2TFLEN_IEPTXRSAR_POS								0x00UL		/** IN endpoint FIFO transmit RAM start address */
#define USBFS_DIEP2TFLEN_IEPTXRSAR_MSK								(0xFFFFUL << USBFS_DIEP2TFLEN_IEPTXRSAR_POS)		/** IN endpoint FIFO transmit RAM start address */
#define USBFS_DIEP2TFLEN_IEPTXFD_POS								0x10UL		/** IN endpoint TxFIFO depth */
#define USBFS_DIEP2TFLEN_IEPTXFD_MSK								(0xFFFFUL << USBFS_DIEP2TFLEN_IEPTXFD_POS)		/** IN endpoint TxFIFO depth */
#define USBFS_DIEP3TFLEN_IEPTXRSAR_POS								0x00UL		/** IN endpoint FIFO4 transmit RAM start address */
#define USBFS_DIEP3TFLEN_IEPTXRSAR_MSK								(0xFFFFUL << USBFS_DIEP3TFLEN_IEPTXRSAR_POS)		/** IN endpoint FIFO4 transmit RAM start address */
#define USBFS_DIEP3TFLEN_IEPTXFD_POS								0x10UL		/** IN endpoint TxFIFO depth */
#define USBFS_DIEP3TFLEN_IEPTXFD_MSK								(0xFFFFUL << USBFS_DIEP3TFLEN_IEPTXFD_POS)		/** IN endpoint TxFIFO depth */
#define USBFS_HCTL_CLKSEL_POS								0x00UL		/** clock select for USB clock */
#define USBFS_HCTL_CLKSEL_MSK								(0x03UL << USBFS_HCTL_CLKSEL_POS)		/** clock select for USB clock */
#define USBFS_HFT_FRI_POS								0x00UL		/** Frame interval */
#define USBFS_HFT_FRI_MSK								(0xFFFFUL << USBFS_HFT_FRI_POS)		/** Frame interval */
#define USBFS_HFINFR_FRNUM_POS								0x00UL		/** Frame number */
#define USBFS_HFINFR_FRNUM_MSK								(0xFFFFUL << USBFS_HFINFR_FRNUM_POS)		/** Frame number */
#define USBFS_HFINFR_FRT_POS								0x10UL		/** Frame remaining time */
#define USBFS_HFINFR_FRT_MSK								(0xFFFFUL << USBFS_HFINFR_FRT_POS)		/** Frame remaining time */
#define USBFS_HPTFQSTAT_PTXFS_POS								0x00UL		/** Periodic transmit data FIFO space available */
#define USBFS_HPTFQSTAT_PTXFS_MSK								(0xFFFFUL << USBFS_HPTFQSTAT_PTXFS_POS)		/** Periodic transmit data FIFO space available */
#define USBFS_HPTFQSTAT_PTXREQS_POS								0x10UL		/** Periodic transmit request queue space available */
#define USBFS_HPTFQSTAT_PTXREQS_MSK								(0xFFUL << USBFS_HPTFQSTAT_PTXREQS_POS)		/** Periodic transmit request queue space available */
#define USBFS_HPTFQSTAT_PTXREQT_POS								0x18UL		/** Top of the periodic transmit request queue */
#define USBFS_HPTFQSTAT_PTXREQT_MSK								(0xFFUL << USBFS_HPTFQSTAT_PTXREQT_POS)		/** Top of the periodic transmit request queue */
#define USBFS_HACHINT_HACHINT_POS								0x00UL		/** Host all channel interrupts */
#define USBFS_HACHINT_HACHINT_MSK								(0xFFUL << USBFS_HACHINT_HACHINT_POS)		/** Host all channel interrupts */
#define USBFS_HACHINTEN_CINTEN_POS								0x00UL		/** Channel interrupt enable */
#define USBFS_HACHINTEN_CINTEN_MSK								(0xFFUL << USBFS_HACHINTEN_CINTEN_POS)		/** Channel interrupt enable */
#define USBFS_HPCS_PCST_POS								0x00UL		/** Port connect status */
#define USBFS_HPCS_PCST_MSK								(0x01UL << USBFS_HPCS_PCST_POS)		/** Port connect status */
#define USBFS_HPCS_PCD_POS								0x01UL		/** Port connect detected */
#define USBFS_HPCS_PCD_MSK								(0x01UL << USBFS_HPCS_PCD_POS)		/** Port connect detected */
#define USBFS_HPCS_PE_POS								0x02UL		/** Port enable */
#define USBFS_HPCS_PE_MSK								(0x01UL << USBFS_HPCS_PE_POS)		/** Port enable */
#define USBFS_HPCS_PEDC_POS								0x03UL		/** Port enable/disable change */
#define USBFS_HPCS_PEDC_MSK								(0x01UL << USBFS_HPCS_PEDC_POS)		/** Port enable/disable change */
#define USBFS_HPCS_PREM_POS								0x06UL		/** Port resume */
#define USBFS_HPCS_PREM_MSK								(0x01UL << USBFS_HPCS_PREM_POS)		/** Port resume */
#define USBFS_HPCS_PSP_POS								0x07UL		/** Port suspend */
#define USBFS_HPCS_PSP_MSK								(0x01UL << USBFS_HPCS_PSP_POS)		/** Port suspend */
#define USBFS_HPCS_PRST_POS								0x08UL		/** Port reset */
#define USBFS_HPCS_PRST_MSK								(0x01UL << USBFS_HPCS_PRST_POS)		/** Port reset */
#define USBFS_HPCS_PLST_POS								0x0AUL		/** Port line status */
#define USBFS_HPCS_PLST_MSK								(0x03UL << USBFS_HPCS_PLST_POS)		/** Port line status */
#define USBFS_HPCS_PP_POS								0x0CUL		/** Port power */
#define USBFS_HPCS_PP_MSK								(0x01UL << USBFS_HPCS_PP_POS)		/** Port power */
#define USBFS_HPCS_PS_POS								0x11UL		/** Port speed */
#define USBFS_HPCS_PS_MSK								(0x03UL << USBFS_HPCS_PS_POS)		/** Port speed */
#define USBFS_HCH0CTL_MPL_POS								0x00UL		/** Maximum packet size */
#define USBFS_HCH0CTL_MPL_MSK								(0x7FFUL << USBFS_HCH0CTL_MPL_POS)		/** Maximum packet size */
#define USBFS_HCH0CTL_EPNUM_POS								0x0BUL		/** Endpoint number */
#define USBFS_HCH0CTL_EPNUM_MSK								(0x0FUL << USBFS_HCH0CTL_EPNUM_POS)		/** Endpoint number */
#define USBFS_HCH0CTL_EPDIR_POS								0x0FUL		/** Endpoint direction */
#define USBFS_HCH0CTL_EPDIR_MSK								(0x01UL << USBFS_HCH0CTL_EPDIR_POS)		/** Endpoint direction */
#define USBFS_HCH0CTL_LSD_POS								0x11UL		/** Low-speed device */
#define USBFS_HCH0CTL_LSD_MSK								(0x01UL << USBFS_HCH0CTL_LSD_POS)		/** Low-speed device */
#define USBFS_HCH0CTL_EPTYPE_POS								0x12UL		/** Endpoint type */
#define USBFS_HCH0CTL_EPTYPE_MSK								(0x03UL << USBFS_HCH0CTL_EPTYPE_POS)		/** Endpoint type */
#define USBFS_HCH0CTL_DAR_POS								0x16UL		/** Device address */
#define USBFS_HCH0CTL_DAR_MSK								(0x7FUL << USBFS_HCH0CTL_DAR_POS)		/** Device address */
#define USBFS_HCH0CTL_ODDFRM_POS								0x1DUL		/** Odd frame */
#define USBFS_HCH0CTL_ODDFRM_MSK								(0x01UL << USBFS_HCH0CTL_ODDFRM_POS)		/** Odd frame */
#define USBFS_HCH0CTL_CDIS_POS								0x1EUL		/** Channel disable */
#define USBFS_HCH0CTL_CDIS_MSK								(0x01UL << USBFS_HCH0CTL_CDIS_POS)		/** Channel disable */
#define USBFS_HCH0CTL_CEN_POS								0x1FUL		/** Channel enable */
#define USBFS_HCH0CTL_CEN_MSK								(0x01UL << USBFS_HCH0CTL_CEN_POS)		/** Channel enable */
#define USBFS_HCH1CTL_MPL_POS								0x00UL		/** Maximum packet size */
#define USBFS_HCH1CTL_MPL_MSK								(0x7FFUL << USBFS_HCH1CTL_MPL_POS)		/** Maximum packet size */
#define USBFS_HCH1CTL_EPNUM_POS								0x0BUL		/** Endpoint number */
#define USBFS_HCH1CTL_EPNUM_MSK								(0x0FUL << USBFS_HCH1CTL_EPNUM_POS)		/** Endpoint number */
#define USBFS_HCH1CTL_EPDIR_POS								0x0FUL		/** Endpoint direction */
#define USBFS_HCH1CTL_EPDIR_MSK								(0x01UL << USBFS_HCH1CTL_EPDIR_POS)		/** Endpoint direction */
#define USBFS_HCH1CTL_LSD_POS								0x11UL		/** Low-speed device */
#define USBFS_HCH1CTL_LSD_MSK								(0x01UL << USBFS_HCH1CTL_LSD_POS)		/** Low-speed device */
#define USBFS_HCH1CTL_EPTYPE_POS								0x12UL		/** Endpoint type */
#define USBFS_HCH1CTL_EPTYPE_MSK								(0x03UL << USBFS_HCH1CTL_EPTYPE_POS)		/** Endpoint type */
#define USBFS_HCH1CTL_DAR_POS								0x16UL		/** Device address */
#define USBFS_HCH1CTL_DAR_MSK								(0x7FUL << USBFS_HCH1CTL_DAR_POS)		/** Device address */
#define USBFS_HCH1CTL_ODDFRM_POS								0x1DUL		/** Odd frame */
#define USBFS_HCH1CTL_ODDFRM_MSK								(0x01UL << USBFS_HCH1CTL_ODDFRM_POS)		/** Odd frame */
#define USBFS_HCH1CTL_CDIS_POS								0x1EUL		/** Channel disable */
#define USBFS_HCH1CTL_CDIS_MSK								(0x01UL << USBFS_HCH1CTL_CDIS_POS)		/** Channel disable */
#define USBFS_HCH1CTL_CEN_POS								0x1FUL		/** Channel enable */
#define USBFS_HCH1CTL_CEN_MSK								(0x01UL << USBFS_HCH1CTL_CEN_POS)		/** Channel enable */
#define USBFS_HCH2CTL_MPL_POS								0x00UL		/** Maximum packet size */
#define USBFS_HCH2CTL_MPL_MSK								(0x7FFUL << USBFS_HCH2CTL_MPL_POS)		/** Maximum packet size */
#define USBFS_HCH2CTL_EPNUM_POS								0x0BUL		/** Endpoint number */
#define USBFS_HCH2CTL_EPNUM_MSK								(0x0FUL << USBFS_HCH2CTL_EPNUM_POS)		/** Endpoint number */
#define USBFS_HCH2CTL_EPDIR_POS								0x0FUL		/** Endpoint direction */
#define USBFS_HCH2CTL_EPDIR_MSK								(0x01UL << USBFS_HCH2CTL_EPDIR_POS)		/** Endpoint direction */
#define USBFS_HCH2CTL_LSD_POS								0x11UL		/** Low-speed device */
#define USBFS_HCH2CTL_LSD_MSK								(0x01UL << USBFS_HCH2CTL_LSD_POS)		/** Low-speed device */
#define USBFS_HCH2CTL_EPTYPE_POS								0x12UL		/** Endpoint type */
#define USBFS_HCH2CTL_EPTYPE_MSK								(0x03UL << USBFS_HCH2CTL_EPTYPE_POS)		/** Endpoint type */
#define USBFS_HCH2CTL_DAR_POS								0x16UL		/** Device address */
#define USBFS_HCH2CTL_DAR_MSK								(0x7FUL << USBFS_HCH2CTL_DAR_POS)		/** Device address */
#define USBFS_HCH2CTL_ODDFRM_POS								0x1DUL		/** Odd frame */
#define USBFS_HCH2CTL_ODDFRM_MSK								(0x01UL << USBFS_HCH2CTL_ODDFRM_POS)		/** Odd frame */
#define USBFS_HCH2CTL_CDIS_POS								0x1EUL		/** Channel disable */
#define USBFS_HCH2CTL_CDIS_MSK								(0x01UL << USBFS_HCH2CTL_CDIS_POS)		/** Channel disable */
#define USBFS_HCH2CTL_CEN_POS								0x1FUL		/** Channel enable */
#define USBFS_HCH2CTL_CEN_MSK								(0x01UL << USBFS_HCH2CTL_CEN_POS)		/** Channel enable */
#define USBFS_HCH3CTL_MPL_POS								0x00UL		/** Maximum packet size */
#define USBFS_HCH3CTL_MPL_MSK								(0x7FFUL << USBFS_HCH3CTL_MPL_POS)		/** Maximum packet size */
#define USBFS_HCH3CTL_EPNUM_POS								0x0BUL		/** Endpoint number */
#define USBFS_HCH3CTL_EPNUM_MSK								(0x0FUL << USBFS_HCH3CTL_EPNUM_POS)		/** Endpoint number */
#define USBFS_HCH3CTL_EPDIR_POS								0x0FUL		/** Endpoint direction */
#define USBFS_HCH3CTL_EPDIR_MSK								(0x01UL << USBFS_HCH3CTL_EPDIR_POS)		/** Endpoint direction */
#define USBFS_HCH3CTL_LSD_POS								0x11UL		/** Low-speed device */
#define USBFS_HCH3CTL_LSD_MSK								(0x01UL << USBFS_HCH3CTL_LSD_POS)		/** Low-speed device */
#define USBFS_HCH3CTL_EPTYPE_POS								0x12UL		/** Endpoint type */
#define USBFS_HCH3CTL_EPTYPE_MSK								(0x03UL << USBFS_HCH3CTL_EPTYPE_POS)		/** Endpoint type */
#define USBFS_HCH3CTL_DAR_POS								0x16UL		/** Device address */
#define USBFS_HCH3CTL_DAR_MSK								(0x7FUL << USBFS_HCH3CTL_DAR_POS)		/** Device address */
#define USBFS_HCH3CTL_ODDFRM_POS								0x1DUL		/** Odd frame */
#define USBFS_HCH3CTL_ODDFRM_MSK								(0x01UL << USBFS_HCH3CTL_ODDFRM_POS)		/** Odd frame */
#define USBFS_HCH3CTL_CDIS_POS								0x1EUL		/** Channel disable */
#define USBFS_HCH3CTL_CDIS_MSK								(0x01UL << USBFS_HCH3CTL_CDIS_POS)		/** Channel disable */
#define USBFS_HCH3CTL_CEN_POS								0x1FUL		/** Channel enable */
#define USBFS_HCH3CTL_CEN_MSK								(0x01UL << USBFS_HCH3CTL_CEN_POS)		/** Channel enable */
#define USBFS_HCH4CTL_MPL_POS								0x00UL		/** Maximum packet size */
#define USBFS_HCH4CTL_MPL_MSK								(0x7FFUL << USBFS_HCH4CTL_MPL_POS)		/** Maximum packet size */
#define USBFS_HCH4CTL_EPNUM_POS								0x0BUL		/** Endpoint number */
#define USBFS_HCH4CTL_EPNUM_MSK								(0x0FUL << USBFS_HCH4CTL_EPNUM_POS)		/** Endpoint number */
#define USBFS_HCH4CTL_EPDIR_POS								0x0FUL		/** Endpoint direction */
#define USBFS_HCH4CTL_EPDIR_MSK								(0x01UL << USBFS_HCH4CTL_EPDIR_POS)		/** Endpoint direction */
#define USBFS_HCH4CTL_LSD_POS								0x11UL		/** Low-speed device */
#define USBFS_HCH4CTL_LSD_MSK								(0x01UL << USBFS_HCH4CTL_LSD_POS)		/** Low-speed device */
#define USBFS_HCH4CTL_EPTYPE_POS								0x12UL		/** Endpoint type */
#define USBFS_HCH4CTL_EPTYPE_MSK								(0x03UL << USBFS_HCH4CTL_EPTYPE_POS)		/** Endpoint type */
#define USBFS_HCH4CTL_DAR_POS								0x16UL		/** Device address */
#define USBFS_HCH4CTL_DAR_MSK								(0x7FUL << USBFS_HCH4CTL_DAR_POS)		/** Device address */
#define USBFS_HCH4CTL_ODDFRM_POS								0x1DUL		/** Odd frame */
#define USBFS_HCH4CTL_ODDFRM_MSK								(0x01UL << USBFS_HCH4CTL_ODDFRM_POS)		/** Odd frame */
#define USBFS_HCH4CTL_CDIS_POS								0x1EUL		/** Channel disable */
#define USBFS_HCH4CTL_CDIS_MSK								(0x01UL << USBFS_HCH4CTL_CDIS_POS)		/** Channel disable */
#define USBFS_HCH4CTL_CEN_POS								0x1FUL		/** Channel enable */
#define USBFS_HCH4CTL_CEN_MSK								(0x01UL << USBFS_HCH4CTL_CEN_POS)		/** Channel enable */
#define USBFS_HCH5CTL_MPL_POS								0x00UL		/** Maximum packet size */
#define USBFS_HCH5CTL_MPL_MSK								(0x7FFUL << USBFS_HCH5CTL_MPL_POS)		/** Maximum packet size */
#define USBFS_HCH5CTL_EPNUM_POS								0x0BUL		/** Endpoint number */
#define USBFS_HCH5CTL_EPNUM_MSK								(0x0FUL << USBFS_HCH5CTL_EPNUM_POS)		/** Endpoint number */
#define USBFS_HCH5CTL_EPDIR_POS								0x0FUL		/** Endpoint direction */
#define USBFS_HCH5CTL_EPDIR_MSK								(0x01UL << USBFS_HCH5CTL_EPDIR_POS)		/** Endpoint direction */
#define USBFS_HCH5CTL_LSD_POS								0x11UL		/** Low-speed device */
#define USBFS_HCH5CTL_LSD_MSK								(0x01UL << USBFS_HCH5CTL_LSD_POS)		/** Low-speed device */
#define USBFS_HCH5CTL_EPTYPE_POS								0x12UL		/** Endpoint type */
#define USBFS_HCH5CTL_EPTYPE_MSK								(0x03UL << USBFS_HCH5CTL_EPTYPE_POS)		/** Endpoint type */
#define USBFS_HCH5CTL_DAR_POS								0x16UL		/** Device address */
#define USBFS_HCH5CTL_DAR_MSK								(0x7FUL << USBFS_HCH5CTL_DAR_POS)		/** Device address */
#define USBFS_HCH5CTL_ODDFRM_POS								0x1DUL		/** Odd frame */
#define USBFS_HCH5CTL_ODDFRM_MSK								(0x01UL << USBFS_HCH5CTL_ODDFRM_POS)		/** Odd frame */
#define USBFS_HCH5CTL_CDIS_POS								0x1EUL		/** Channel disable */
#define USBFS_HCH5CTL_CDIS_MSK								(0x01UL << USBFS_HCH5CTL_CDIS_POS)		/** Channel disable */
#define USBFS_HCH5CTL_CEN_POS								0x1FUL		/** Channel enable */
#define USBFS_HCH5CTL_CEN_MSK								(0x01UL << USBFS_HCH5CTL_CEN_POS)		/** Channel enable */
#define USBFS_HCH6CTL_MPL_POS								0x00UL		/** Maximum packet size */
#define USBFS_HCH6CTL_MPL_MSK								(0x7FFUL << USBFS_HCH6CTL_MPL_POS)		/** Maximum packet size */
#define USBFS_HCH6CTL_EPNUM_POS								0x0BUL		/** Endpoint number */
#define USBFS_HCH6CTL_EPNUM_MSK								(0x0FUL << USBFS_HCH6CTL_EPNUM_POS)		/** Endpoint number */
#define USBFS_HCH6CTL_EPDIR_POS								0x0FUL		/** Endpoint direction */
#define USBFS_HCH6CTL_EPDIR_MSK								(0x01UL << USBFS_HCH6CTL_EPDIR_POS)		/** Endpoint direction */
#define USBFS_HCH6CTL_LSD_POS								0x11UL		/** Low-speed device */
#define USBFS_HCH6CTL_LSD_MSK								(0x01UL << USBFS_HCH6CTL_LSD_POS)		/** Low-speed device */
#define USBFS_HCH6CTL_EPTYPE_POS								0x12UL		/** Endpoint type */
#define USBFS_HCH6CTL_EPTYPE_MSK								(0x03UL << USBFS_HCH6CTL_EPTYPE_POS)		/** Endpoint type */
#define USBFS_HCH6CTL_DAR_POS								0x16UL		/** Device address */
#define USBFS_HCH6CTL_DAR_MSK								(0x7FUL << USBFS_HCH6CTL_DAR_POS)		/** Device address */
#define USBFS_HCH6CTL_ODDFRM_POS								0x1DUL		/** Odd frame */
#define USBFS_HCH6CTL_ODDFRM_MSK								(0x01UL << USBFS_HCH6CTL_ODDFRM_POS)		/** Odd frame */
#define USBFS_HCH6CTL_CDIS_POS								0x1EUL		/** Channel disable */
#define USBFS_HCH6CTL_CDIS_MSK								(0x01UL << USBFS_HCH6CTL_CDIS_POS)		/** Channel disable */
#define USBFS_HCH6CTL_CEN_POS								0x1FUL		/** Channel enable */
#define USBFS_HCH6CTL_CEN_MSK								(0x01UL << USBFS_HCH6CTL_CEN_POS)		/** Channel enable */
#define USBFS_HCH7CTL_MPL_POS								0x00UL		/** Maximum packet size */
#define USBFS_HCH7CTL_MPL_MSK								(0x7FFUL << USBFS_HCH7CTL_MPL_POS)		/** Maximum packet size */
#define USBFS_HCH7CTL_EPNUM_POS								0x0BUL		/** Endpoint number */
#define USBFS_HCH7CTL_EPNUM_MSK								(0x0FUL << USBFS_HCH7CTL_EPNUM_POS)		/** Endpoint number */
#define USBFS_HCH7CTL_EPDIR_POS								0x0FUL		/** Endpoint direction */
#define USBFS_HCH7CTL_EPDIR_MSK								(0x01UL << USBFS_HCH7CTL_EPDIR_POS)		/** Endpoint direction */
#define USBFS_HCH7CTL_LSD_POS								0x11UL		/** Low-speed device */
#define USBFS_HCH7CTL_LSD_MSK								(0x01UL << USBFS_HCH7CTL_LSD_POS)		/** Low-speed device */
#define USBFS_HCH7CTL_EPTYPE_POS								0x12UL		/** Endpoint type */
#define USBFS_HCH7CTL_EPTYPE_MSK								(0x03UL << USBFS_HCH7CTL_EPTYPE_POS)		/** Endpoint type */
#define USBFS_HCH7CTL_DAR_POS								0x16UL		/** Device address */
#define USBFS_HCH7CTL_DAR_MSK								(0x7FUL << USBFS_HCH7CTL_DAR_POS)		/** Device address */
#define USBFS_HCH7CTL_ODDFRM_POS								0x1DUL		/** Odd frame */
#define USBFS_HCH7CTL_ODDFRM_MSK								(0x01UL << USBFS_HCH7CTL_ODDFRM_POS)		/** Odd frame */
#define USBFS_HCH7CTL_CDIS_POS								0x1EUL		/** Channel disable */
#define USBFS_HCH7CTL_CDIS_MSK								(0x01UL << USBFS_HCH7CTL_CDIS_POS)		/** Channel disable */
#define USBFS_HCH7CTL_CEN_POS								0x1FUL		/** Channel enable */
#define USBFS_HCH7CTL_CEN_MSK								(0x01UL << USBFS_HCH7CTL_CEN_POS)		/** Channel enable */
#define USBFS_HCH0INTF_TF_POS								0x00UL		/** Transfer finished */
#define USBFS_HCH0INTF_TF_MSK								(0x01UL << USBFS_HCH0INTF_TF_POS)		/** Transfer finished */
#define USBFS_HCH0INTF_CH_POS								0x01UL		/** Channel halted */
#define USBFS_HCH0INTF_CH_MSK								(0x01UL << USBFS_HCH0INTF_CH_POS)		/** Channel halted */
#define USBFS_HCH0INTF_STALL_POS								0x03UL		/** STALL response received interrupt */
#define USBFS_HCH0INTF_STALL_MSK								(0x01UL << USBFS_HCH0INTF_STALL_POS)		/** STALL response received interrupt */
#define USBFS_HCH0INTF_NAK_POS								0x04UL		/** NAK response received interrupt */
#define USBFS_HCH0INTF_NAK_MSK								(0x01UL << USBFS_HCH0INTF_NAK_POS)		/** NAK response received interrupt */
#define USBFS_HCH0INTF_ACK_POS								0x05UL		/** ACK response received/transmitted interrupt */
#define USBFS_HCH0INTF_ACK_MSK								(0x01UL << USBFS_HCH0INTF_ACK_POS)		/** ACK response received/transmitted interrupt */
#define USBFS_HCH0INTF_USBER_POS								0x07UL		/** USB bus error */
#define USBFS_HCH0INTF_USBER_MSK								(0x01UL << USBFS_HCH0INTF_USBER_POS)		/** USB bus error */
#define USBFS_HCH0INTF_BBER_POS								0x08UL		/** Babble error */
#define USBFS_HCH0INTF_BBER_MSK								(0x01UL << USBFS_HCH0INTF_BBER_POS)		/** Babble error */
#define USBFS_HCH0INTF_REQOVR_POS								0x09UL		/** Request queue overrun */
#define USBFS_HCH0INTF_REQOVR_MSK								(0x01UL << USBFS_HCH0INTF_REQOVR_POS)		/** Request queue overrun */
#define USBFS_HCH0INTF_DTER_POS								0x0AUL		/** Data toggle error */
#define USBFS_HCH0INTF_DTER_MSK								(0x01UL << USBFS_HCH0INTF_DTER_POS)		/** Data toggle error */
#define USBFS_HCH1INTF_TF_POS								0x00UL		/** Transfer finished */
#define USBFS_HCH1INTF_TF_MSK								(0x01UL << USBFS_HCH1INTF_TF_POS)		/** Transfer finished */
#define USBFS_HCH1INTF_CH_POS								0x01UL		/** Channel halted */
#define USBFS_HCH1INTF_CH_MSK								(0x01UL << USBFS_HCH1INTF_CH_POS)		/** Channel halted */
#define USBFS_HCH1INTF_STALL_POS								0x03UL		/** STALL response received interrupt */
#define USBFS_HCH1INTF_STALL_MSK								(0x01UL << USBFS_HCH1INTF_STALL_POS)		/** STALL response received interrupt */
#define USBFS_HCH1INTF_NAK_POS								0x04UL		/** NAK response received interrupt */
#define USBFS_HCH1INTF_NAK_MSK								(0x01UL << USBFS_HCH1INTF_NAK_POS)		/** NAK response received interrupt */
#define USBFS_HCH1INTF_ACK_POS								0x05UL		/** ACK response received/transmitted interrupt */
#define USBFS_HCH1INTF_ACK_MSK								(0x01UL << USBFS_HCH1INTF_ACK_POS)		/** ACK response received/transmitted interrupt */
#define USBFS_HCH1INTF_USBER_POS								0x07UL		/** USB bus error */
#define USBFS_HCH1INTF_USBER_MSK								(0x01UL << USBFS_HCH1INTF_USBER_POS)		/** USB bus error */
#define USBFS_HCH1INTF_BBER_POS								0x08UL		/** Babble error */
#define USBFS_HCH1INTF_BBER_MSK								(0x01UL << USBFS_HCH1INTF_BBER_POS)		/** Babble error */
#define USBFS_HCH1INTF_REQOVR_POS								0x09UL		/** Request queue overrun */
#define USBFS_HCH1INTF_REQOVR_MSK								(0x01UL << USBFS_HCH1INTF_REQOVR_POS)		/** Request queue overrun */
#define USBFS_HCH1INTF_DTER_POS								0x0AUL		/** Data toggle error */
#define USBFS_HCH1INTF_DTER_MSK								(0x01UL << USBFS_HCH1INTF_DTER_POS)		/** Data toggle error */
#define USBFS_HCH2INTF_TF_POS								0x00UL		/** Transfer finished */
#define USBFS_HCH2INTF_TF_MSK								(0x01UL << USBFS_HCH2INTF_TF_POS)		/** Transfer finished */
#define USBFS_HCH2INTF_CH_POS								0x01UL		/** Channel halted */
#define USBFS_HCH2INTF_CH_MSK								(0x01UL << USBFS_HCH2INTF_CH_POS)		/** Channel halted */
#define USBFS_HCH2INTF_STALL_POS								0x03UL		/** STALL response received interrupt */
#define USBFS_HCH2INTF_STALL_MSK								(0x01UL << USBFS_HCH2INTF_STALL_POS)		/** STALL response received interrupt */
#define USBFS_HCH2INTF_NAK_POS								0x04UL		/** NAK response received interrupt */
#define USBFS_HCH2INTF_NAK_MSK								(0x01UL << USBFS_HCH2INTF_NAK_POS)		/** NAK response received interrupt */
#define USBFS_HCH2INTF_ACK_POS								0x05UL		/** ACK response received/transmitted interrupt */
#define USBFS_HCH2INTF_ACK_MSK								(0x01UL << USBFS_HCH2INTF_ACK_POS)		/** ACK response received/transmitted interrupt */
#define USBFS_HCH2INTF_USBER_POS								0x07UL		/** USB bus error */
#define USBFS_HCH2INTF_USBER_MSK								(0x01UL << USBFS_HCH2INTF_USBER_POS)		/** USB bus error */
#define USBFS_HCH2INTF_BBER_POS								0x08UL		/** Babble error */
#define USBFS_HCH2INTF_BBER_MSK								(0x01UL << USBFS_HCH2INTF_BBER_POS)		/** Babble error */
#define USBFS_HCH2INTF_REQOVR_POS								0x09UL		/** Request queue overrun */
#define USBFS_HCH2INTF_REQOVR_MSK								(0x01UL << USBFS_HCH2INTF_REQOVR_POS)		/** Request queue overrun */
#define USBFS_HCH2INTF_DTER_POS								0x0AUL		/** Data toggle error */
#define USBFS_HCH2INTF_DTER_MSK								(0x01UL << USBFS_HCH2INTF_DTER_POS)		/** Data toggle error */
#define USBFS_HCH3INTF_TF_POS								0x00UL		/** Transfer finished */
#define USBFS_HCH3INTF_TF_MSK								(0x01UL << USBFS_HCH3INTF_TF_POS)		/** Transfer finished */
#define USBFS_HCH3INTF_CH_POS								0x01UL		/** Channel halted */
#define USBFS_HCH3INTF_CH_MSK								(0x01UL << USBFS_HCH3INTF_CH_POS)		/** Channel halted */
#define USBFS_HCH3INTF_STALL_POS								0x03UL		/** STALL response received interrupt */
#define USBFS_HCH3INTF_STALL_MSK								(0x01UL << USBFS_HCH3INTF_STALL_POS)		/** STALL response received interrupt */
#define USBFS_HCH3INTF_NAK_POS								0x04UL		/** NAK response received interrupt */
#define USBFS_HCH3INTF_NAK_MSK								(0x01UL << USBFS_HCH3INTF_NAK_POS)		/** NAK response received interrupt */
#define USBFS_HCH3INTF_ACK_POS								0x05UL		/** ACK response received/transmitted interrupt */
#define USBFS_HCH3INTF_ACK_MSK								(0x01UL << USBFS_HCH3INTF_ACK_POS)		/** ACK response received/transmitted interrupt */
#define USBFS_HCH3INTF_USBER_POS								0x07UL		/** USB bus error */
#define USBFS_HCH3INTF_USBER_MSK								(0x01UL << USBFS_HCH3INTF_USBER_POS)		/** USB bus error */
#define USBFS_HCH3INTF_BBER_POS								0x08UL		/** Babble error */
#define USBFS_HCH3INTF_BBER_MSK								(0x01UL << USBFS_HCH3INTF_BBER_POS)		/** Babble error */
#define USBFS_HCH3INTF_REQOVR_POS								0x09UL		/** Request queue overrun */
#define USBFS_HCH3INTF_REQOVR_MSK								(0x01UL << USBFS_HCH3INTF_REQOVR_POS)		/** Request queue overrun */
#define USBFS_HCH3INTF_DTER_POS								0x0AUL		/** Data toggle error */
#define USBFS_HCH3INTF_DTER_MSK								(0x01UL << USBFS_HCH3INTF_DTER_POS)		/** Data toggle error */
#define USBFS_HCH4INTF_TF_POS								0x00UL		/** Transfer finished */
#define USBFS_HCH4INTF_TF_MSK								(0x01UL << USBFS_HCH4INTF_TF_POS)		/** Transfer finished */
#define USBFS_HCH4INTF_CH_POS								0x01UL		/** Channel halted */
#define USBFS_HCH4INTF_CH_MSK								(0x01UL << USBFS_HCH4INTF_CH_POS)		/** Channel halted */
#define USBFS_HCH4INTF_STALL_POS								0x03UL		/** STALL response received interrupt */
#define USBFS_HCH4INTF_STALL_MSK								(0x01UL << USBFS_HCH4INTF_STALL_POS)		/** STALL response received interrupt */
#define USBFS_HCH4INTF_NAK_POS								0x04UL		/** NAK response received interrupt */
#define USBFS_HCH4INTF_NAK_MSK								(0x01UL << USBFS_HCH4INTF_NAK_POS)		/** NAK response received interrupt */
#define USBFS_HCH4INTF_ACK_POS								0x05UL		/** ACK response received/transmitted interrupt */
#define USBFS_HCH4INTF_ACK_MSK								(0x01UL << USBFS_HCH4INTF_ACK_POS)		/** ACK response received/transmitted interrupt */
#define USBFS_HCH4INTF_USBER_POS								0x07UL		/** USB bus error */
#define USBFS_HCH4INTF_USBER_MSK								(0x01UL << USBFS_HCH4INTF_USBER_POS)		/** USB bus error */
#define USBFS_HCH4INTF_BBER_POS								0x08UL		/** Babble error */
#define USBFS_HCH4INTF_BBER_MSK								(0x01UL << USBFS_HCH4INTF_BBER_POS)		/** Babble error */
#define USBFS_HCH4INTF_REQOVR_POS								0x09UL		/** Request queue overrun */
#define USBFS_HCH4INTF_REQOVR_MSK								(0x01UL << USBFS_HCH4INTF_REQOVR_POS)		/** Request queue overrun */
#define USBFS_HCH4INTF_DTER_POS								0x0AUL		/** Data toggle error */
#define USBFS_HCH4INTF_DTER_MSK								(0x01UL << USBFS_HCH4INTF_DTER_POS)		/** Data toggle error */
#define USBFS_HCH5INTF_TF_POS								0x00UL		/** Transfer finished */
#define USBFS_HCH5INTF_TF_MSK								(0x01UL << USBFS_HCH5INTF_TF_POS)		/** Transfer finished */
#define USBFS_HCH5INTF_CH_POS								0x01UL		/** Channel halted */
#define USBFS_HCH5INTF_CH_MSK								(0x01UL << USBFS_HCH5INTF_CH_POS)		/** Channel halted */
#define USBFS_HCH5INTF_STALL_POS								0x03UL		/** STALL response received interrupt */
#define USBFS_HCH5INTF_STALL_MSK								(0x01UL << USBFS_HCH5INTF_STALL_POS)		/** STALL response received interrupt */
#define USBFS_HCH5INTF_NAK_POS								0x04UL		/** NAK response received interrupt */
#define USBFS_HCH5INTF_NAK_MSK								(0x01UL << USBFS_HCH5INTF_NAK_POS)		/** NAK response received interrupt */
#define USBFS_HCH5INTF_ACK_POS								0x05UL		/** ACK response received/transmitted interrupt */
#define USBFS_HCH5INTF_ACK_MSK								(0x01UL << USBFS_HCH5INTF_ACK_POS)		/** ACK response received/transmitted interrupt */
#define USBFS_HCH5INTF_USBER_POS								0x07UL		/** USB bus error */
#define USBFS_HCH5INTF_USBER_MSK								(0x01UL << USBFS_HCH5INTF_USBER_POS)		/** USB bus error */
#define USBFS_HCH5INTF_BBER_POS								0x08UL		/** Babble error */
#define USBFS_HCH5INTF_BBER_MSK								(0x01UL << USBFS_HCH5INTF_BBER_POS)		/** Babble error */
#define USBFS_HCH5INTF_REQOVR_POS								0x09UL		/** Request queue overrun */
#define USBFS_HCH5INTF_REQOVR_MSK								(0x01UL << USBFS_HCH5INTF_REQOVR_POS)		/** Request queue overrun */
#define USBFS_HCH5INTF_DTER_POS								0x0AUL		/** Data toggle error */
#define USBFS_HCH5INTF_DTER_MSK								(0x01UL << USBFS_HCH5INTF_DTER_POS)		/** Data toggle error */
#define USBFS_HCH6INTF_TF_POS								0x00UL		/** Transfer finished */
#define USBFS_HCH6INTF_TF_MSK								(0x01UL << USBFS_HCH6INTF_TF_POS)		/** Transfer finished */
#define USBFS_HCH6INTF_CH_POS								0x01UL		/** Channel halted */
#define USBFS_HCH6INTF_CH_MSK								(0x01UL << USBFS_HCH6INTF_CH_POS)		/** Channel halted */
#define USBFS_HCH6INTF_STALL_POS								0x03UL		/** STALL response received interrupt */
#define USBFS_HCH6INTF_STALL_MSK								(0x01UL << USBFS_HCH6INTF_STALL_POS)		/** STALL response received interrupt */
#define USBFS_HCH6INTF_NAK_POS								0x04UL		/** NAK response received interrupt */
#define USBFS_HCH6INTF_NAK_MSK								(0x01UL << USBFS_HCH6INTF_NAK_POS)		/** NAK response received interrupt */
#define USBFS_HCH6INTF_ACK_POS								0x05UL		/** ACK response received/transmitted interrupt */
#define USBFS_HCH6INTF_ACK_MSK								(0x01UL << USBFS_HCH6INTF_ACK_POS)		/** ACK response received/transmitted interrupt */
#define USBFS_HCH6INTF_USBER_POS								0x07UL		/** USB bus error */
#define USBFS_HCH6INTF_USBER_MSK								(0x01UL << USBFS_HCH6INTF_USBER_POS)		/** USB bus error */
#define USBFS_HCH6INTF_BBER_POS								0x08UL		/** Babble error */
#define USBFS_HCH6INTF_BBER_MSK								(0x01UL << USBFS_HCH6INTF_BBER_POS)		/** Babble error */
#define USBFS_HCH6INTF_REQOVR_POS								0x09UL		/** Request queue overrun */
#define USBFS_HCH6INTF_REQOVR_MSK								(0x01UL << USBFS_HCH6INTF_REQOVR_POS)		/** Request queue overrun */
#define USBFS_HCH6INTF_DTER_POS								0x0AUL		/** Data toggle error */
#define USBFS_HCH6INTF_DTER_MSK								(0x01UL << USBFS_HCH6INTF_DTER_POS)		/** Data toggle error */
#define USBFS_HCH7INTF_TF_POS								0x00UL		/** Transfer finished */
#define USBFS_HCH7INTF_TF_MSK								(0x01UL << USBFS_HCH7INTF_TF_POS)		/** Transfer finished */
#define USBFS_HCH7INTF_CH_POS								0x01UL		/** Channel halted */
#define USBFS_HCH7INTF_CH_MSK								(0x01UL << USBFS_HCH7INTF_CH_POS)		/** Channel halted */
#define USBFS_HCH7INTF_STALL_POS								0x03UL		/** STALL response received interrupt */
#define USBFS_HCH7INTF_STALL_MSK								(0x01UL << USBFS_HCH7INTF_STALL_POS)		/** STALL response received interrupt */
#define USBFS_HCH7INTF_NAK_POS								0x04UL		/** NAK response received interrupt */
#define USBFS_HCH7INTF_NAK_MSK								(0x01UL << USBFS_HCH7INTF_NAK_POS)		/** NAK response received interrupt */
#define USBFS_HCH7INTF_ACK_POS								0x05UL		/** ACK response received/transmitted interrupt */
#define USBFS_HCH7INTF_ACK_MSK								(0x01UL << USBFS_HCH7INTF_ACK_POS)		/** ACK response received/transmitted interrupt */
#define USBFS_HCH7INTF_USBER_POS								0x07UL		/** USB bus error */
#define USBFS_HCH7INTF_USBER_MSK								(0x01UL << USBFS_HCH7INTF_USBER_POS)		/** USB bus error */
#define USBFS_HCH7INTF_BBER_POS								0x08UL		/** Babble error */
#define USBFS_HCH7INTF_BBER_MSK								(0x01UL << USBFS_HCH7INTF_BBER_POS)		/** Babble error */
#define USBFS_HCH7INTF_REQOVR_POS								0x09UL		/** Request queue overrun */
#define USBFS_HCH7INTF_REQOVR_MSK								(0x01UL << USBFS_HCH7INTF_REQOVR_POS)		/** Request queue overrun */
#define USBFS_HCH7INTF_DTER_POS								0x0AUL		/** Data toggle error */
#define USBFS_HCH7INTF_DTER_MSK								(0x01UL << USBFS_HCH7INTF_DTER_POS)		/** Data toggle error */
#define USBFS_HCH0INTEN_TFIE_POS								0x00UL		/** Transfer completed interrupt enable */
#define USBFS_HCH0INTEN_TFIE_MSK								(0x01UL << USBFS_HCH0INTEN_TFIE_POS)		/** Transfer completed interrupt enable */
#define USBFS_HCH0INTEN_CHIE_POS								0x01UL		/** Channel halted interrupt enable */
#define USBFS_HCH0INTEN_CHIE_MSK								(0x01UL << USBFS_HCH0INTEN_CHIE_POS)		/** Channel halted interrupt enable */
#define USBFS_HCH0INTEN_STALLIE_POS								0x03UL		/** STALL interrupt enable */
#define USBFS_HCH0INTEN_STALLIE_MSK								(0x01UL << USBFS_HCH0INTEN_STALLIE_POS)		/** STALL interrupt enable */
#define USBFS_HCH0INTEN_NAKIE_POS								0x04UL		/** NAK interrupt enable */
#define USBFS_HCH0INTEN_NAKIE_MSK								(0x01UL << USBFS_HCH0INTEN_NAKIE_POS)		/** NAK interrupt enable */
#define USBFS_HCH0INTEN_ACKIE_POS								0x05UL		/** ACK interrupt enable */
#define USBFS_HCH0INTEN_ACKIE_MSK								(0x01UL << USBFS_HCH0INTEN_ACKIE_POS)		/** ACK interrupt enable */
#define USBFS_HCH0INTEN_USBERIE_POS								0x07UL		/** USB bus error interrupt enable */
#define USBFS_HCH0INTEN_USBERIE_MSK								(0x01UL << USBFS_HCH0INTEN_USBERIE_POS)		/** USB bus error interrupt enable */
#define USBFS_HCH0INTEN_BBERIE_POS								0x08UL		/** Babble error interrupt enable */
#define USBFS_HCH0INTEN_BBERIE_MSK								(0x01UL << USBFS_HCH0INTEN_BBERIE_POS)		/** Babble error interrupt enable */
#define USBFS_HCH0INTEN_REQOVRIE_POS								0x09UL		/** request queue overrun interrupt enable */
#define USBFS_HCH0INTEN_REQOVRIE_MSK								(0x01UL << USBFS_HCH0INTEN_REQOVRIE_POS)		/** request queue overrun interrupt enable */
#define USBFS_HCH0INTEN_DTERIE_POS								0x0AUL		/** Data toggle error interrupt enable */
#define USBFS_HCH0INTEN_DTERIE_MSK								(0x01UL << USBFS_HCH0INTEN_DTERIE_POS)		/** Data toggle error interrupt enable */
#define USBFS_HCH1INTEN_TFIE_POS								0x00UL		/** Transfer completed interrupt enable */
#define USBFS_HCH1INTEN_TFIE_MSK								(0x01UL << USBFS_HCH1INTEN_TFIE_POS)		/** Transfer completed interrupt enable */
#define USBFS_HCH1INTEN_CHIE_POS								0x01UL		/** Channel halted interrupt enable */
#define USBFS_HCH1INTEN_CHIE_MSK								(0x01UL << USBFS_HCH1INTEN_CHIE_POS)		/** Channel halted interrupt enable */
#define USBFS_HCH1INTEN_STALLIE_POS								0x03UL		/** STALL interrupt enable */
#define USBFS_HCH1INTEN_STALLIE_MSK								(0x01UL << USBFS_HCH1INTEN_STALLIE_POS)		/** STALL interrupt enable */
#define USBFS_HCH1INTEN_NAKIE_POS								0x04UL		/** NAK interrupt enable */
#define USBFS_HCH1INTEN_NAKIE_MSK								(0x01UL << USBFS_HCH1INTEN_NAKIE_POS)		/** NAK interrupt enable */
#define USBFS_HCH1INTEN_ACKIE_POS								0x05UL		/** ACK interrupt enable */
#define USBFS_HCH1INTEN_ACKIE_MSK								(0x01UL << USBFS_HCH1INTEN_ACKIE_POS)		/** ACK interrupt enable */
#define USBFS_HCH1INTEN_USBERIE_POS								0x07UL		/** USB bus error interrupt enable */
#define USBFS_HCH1INTEN_USBERIE_MSK								(0x01UL << USBFS_HCH1INTEN_USBERIE_POS)		/** USB bus error interrupt enable */
#define USBFS_HCH1INTEN_BBERIE_POS								0x08UL		/** Babble error interrupt enable */
#define USBFS_HCH1INTEN_BBERIE_MSK								(0x01UL << USBFS_HCH1INTEN_BBERIE_POS)		/** Babble error interrupt enable */
#define USBFS_HCH1INTEN_REQOVRIE_POS								0x09UL		/** request queue overrun interrupt enable */
#define USBFS_HCH1INTEN_REQOVRIE_MSK								(0x01UL << USBFS_HCH1INTEN_REQOVRIE_POS)		/** request queue overrun interrupt enable */
#define USBFS_HCH1INTEN_DTERIE_POS								0x0AUL		/** Data toggle error interrupt enable */
#define USBFS_HCH1INTEN_DTERIE_MSK								(0x01UL << USBFS_HCH1INTEN_DTERIE_POS)		/** Data toggle error interrupt enable */
#define USBFS_HCH2INTEN_TFIE_POS								0x00UL		/** Transfer completed interrupt enable */
#define USBFS_HCH2INTEN_TFIE_MSK								(0x01UL << USBFS_HCH2INTEN_TFIE_POS)		/** Transfer completed interrupt enable */
#define USBFS_HCH2INTEN_CHIE_POS								0x01UL		/** Channel halted interrupt enable */
#define USBFS_HCH2INTEN_CHIE_MSK								(0x01UL << USBFS_HCH2INTEN_CHIE_POS)		/** Channel halted interrupt enable */
#define USBFS_HCH2INTEN_STALLIE_POS								0x03UL		/** STALL interrupt enable */
#define USBFS_HCH2INTEN_STALLIE_MSK								(0x01UL << USBFS_HCH2INTEN_STALLIE_POS)		/** STALL interrupt enable */
#define USBFS_HCH2INTEN_NAKIE_POS								0x04UL		/** NAK interrupt enable */
#define USBFS_HCH2INTEN_NAKIE_MSK								(0x01UL << USBFS_HCH2INTEN_NAKIE_POS)		/** NAK interrupt enable */
#define USBFS_HCH2INTEN_ACKIE_POS								0x05UL		/** ACK interrupt enable */
#define USBFS_HCH2INTEN_ACKIE_MSK								(0x01UL << USBFS_HCH2INTEN_ACKIE_POS)		/** ACK interrupt enable */
#define USBFS_HCH2INTEN_USBERIE_POS								0x07UL		/** USB bus error interrupt enable */
#define USBFS_HCH2INTEN_USBERIE_MSK								(0x01UL << USBFS_HCH2INTEN_USBERIE_POS)		/** USB bus error interrupt enable */
#define USBFS_HCH2INTEN_BBERIE_POS								0x08UL		/** Babble error interrupt enable */
#define USBFS_HCH2INTEN_BBERIE_MSK								(0x01UL << USBFS_HCH2INTEN_BBERIE_POS)		/** Babble error interrupt enable */
#define USBFS_HCH2INTEN_REQOVRIE_POS								0x09UL		/** request queue overrun interrupt enable */
#define USBFS_HCH2INTEN_REQOVRIE_MSK								(0x01UL << USBFS_HCH2INTEN_REQOVRIE_POS)		/** request queue overrun interrupt enable */
#define USBFS_HCH2INTEN_DTERIE_POS								0x0AUL		/** Data toggle error interrupt enable */
#define USBFS_HCH2INTEN_DTERIE_MSK								(0x01UL << USBFS_HCH2INTEN_DTERIE_POS)		/** Data toggle error interrupt enable */
#define USBFS_HCH3INTEN_TFIE_POS								0x00UL		/** Transfer completed interrupt enable */
#define USBFS_HCH3INTEN_TFIE_MSK								(0x01UL << USBFS_HCH3INTEN_TFIE_POS)		/** Transfer completed interrupt enable */
#define USBFS_HCH3INTEN_CHIE_POS								0x01UL		/** Channel halted interrupt enable */
#define USBFS_HCH3INTEN_CHIE_MSK								(0x01UL << USBFS_HCH3INTEN_CHIE_POS)		/** Channel halted interrupt enable */
#define USBFS_HCH3INTEN_STALLIE_POS								0x03UL		/** STALL interrupt enable */
#define USBFS_HCH3INTEN_STALLIE_MSK								(0x01UL << USBFS_HCH3INTEN_STALLIE_POS)		/** STALL interrupt enable */
#define USBFS_HCH3INTEN_NAKIE_POS								0x04UL		/** NAK interrupt enable */
#define USBFS_HCH3INTEN_NAKIE_MSK								(0x01UL << USBFS_HCH3INTEN_NAKIE_POS)		/** NAK interrupt enable */
#define USBFS_HCH3INTEN_ACKIE_POS								0x05UL		/** ACK interrupt enable */
#define USBFS_HCH3INTEN_ACKIE_MSK								(0x01UL << USBFS_HCH3INTEN_ACKIE_POS)		/** ACK interrupt enable */
#define USBFS_HCH3INTEN_USBERIE_POS								0x07UL		/** USB bus error interrupt enable */
#define USBFS_HCH3INTEN_USBERIE_MSK								(0x01UL << USBFS_HCH3INTEN_USBERIE_POS)		/** USB bus error interrupt enable */
#define USBFS_HCH3INTEN_BBERIE_POS								0x08UL		/** Babble error interrupt enable */
#define USBFS_HCH3INTEN_BBERIE_MSK								(0x01UL << USBFS_HCH3INTEN_BBERIE_POS)		/** Babble error interrupt enable */
#define USBFS_HCH3INTEN_REQOVRIE_POS								0x09UL		/** request queue overrun interrupt enable */
#define USBFS_HCH3INTEN_REQOVRIE_MSK								(0x01UL << USBFS_HCH3INTEN_REQOVRIE_POS)		/** request queue overrun interrupt enable */
#define USBFS_HCH3INTEN_DTERIE_POS								0x0AUL		/** Data toggle error interrupt enable */
#define USBFS_HCH3INTEN_DTERIE_MSK								(0x01UL << USBFS_HCH3INTEN_DTERIE_POS)		/** Data toggle error interrupt enable */
#define USBFS_HCH4INTEN_TFIE_POS								0x00UL		/** Transfer completed interrupt enable */
#define USBFS_HCH4INTEN_TFIE_MSK								(0x01UL << USBFS_HCH4INTEN_TFIE_POS)		/** Transfer completed interrupt enable */
#define USBFS_HCH4INTEN_CHIE_POS								0x01UL		/** Channel halted interrupt enable */
#define USBFS_HCH4INTEN_CHIE_MSK								(0x01UL << USBFS_HCH4INTEN_CHIE_POS)		/** Channel halted interrupt enable */
#define USBFS_HCH4INTEN_STALLIE_POS								0x03UL		/** STALL interrupt enable */
#define USBFS_HCH4INTEN_STALLIE_MSK								(0x01UL << USBFS_HCH4INTEN_STALLIE_POS)		/** STALL interrupt enable */
#define USBFS_HCH4INTEN_NAKIE_POS								0x04UL		/** NAK interrupt enable */
#define USBFS_HCH4INTEN_NAKIE_MSK								(0x01UL << USBFS_HCH4INTEN_NAKIE_POS)		/** NAK interrupt enable */
#define USBFS_HCH4INTEN_ACKIE_POS								0x05UL		/** ACK interrupt enable */
#define USBFS_HCH4INTEN_ACKIE_MSK								(0x01UL << USBFS_HCH4INTEN_ACKIE_POS)		/** ACK interrupt enable */
#define USBFS_HCH4INTEN_USBERIE_POS								0x07UL		/** USB bus error interrupt enable */
#define USBFS_HCH4INTEN_USBERIE_MSK								(0x01UL << USBFS_HCH4INTEN_USBERIE_POS)		/** USB bus error interrupt enable */
#define USBFS_HCH4INTEN_BBERIE_POS								0x08UL		/** Babble error interrupt enable */
#define USBFS_HCH4INTEN_BBERIE_MSK								(0x01UL << USBFS_HCH4INTEN_BBERIE_POS)		/** Babble error interrupt enable */
#define USBFS_HCH4INTEN_REQOVRIE_POS								0x09UL		/** request queue overrun interrupt enable */
#define USBFS_HCH4INTEN_REQOVRIE_MSK								(0x01UL << USBFS_HCH4INTEN_REQOVRIE_POS)		/** request queue overrun interrupt enable */
#define USBFS_HCH4INTEN_DTERIE_POS								0x0AUL		/** Data toggle error interrupt enable */
#define USBFS_HCH4INTEN_DTERIE_MSK								(0x01UL << USBFS_HCH4INTEN_DTERIE_POS)		/** Data toggle error interrupt enable */
#define USBFS_HCH5INTEN_TFIE_POS								0x00UL		/** Transfer completed interrupt enable */
#define USBFS_HCH5INTEN_TFIE_MSK								(0x01UL << USBFS_HCH5INTEN_TFIE_POS)		/** Transfer completed interrupt enable */
#define USBFS_HCH5INTEN_CHIE_POS								0x01UL		/** Channel halted interrupt enable */
#define USBFS_HCH5INTEN_CHIE_MSK								(0x01UL << USBFS_HCH5INTEN_CHIE_POS)		/** Channel halted interrupt enable */
#define USBFS_HCH5INTEN_STALLIE_POS								0x03UL		/** STALL interrupt enable */
#define USBFS_HCH5INTEN_STALLIE_MSK								(0x01UL << USBFS_HCH5INTEN_STALLIE_POS)		/** STALL interrupt enable */
#define USBFS_HCH5INTEN_NAKIE_POS								0x04UL		/** NAK interrupt enable */
#define USBFS_HCH5INTEN_NAKIE_MSK								(0x01UL << USBFS_HCH5INTEN_NAKIE_POS)		/** NAK interrupt enable */
#define USBFS_HCH5INTEN_ACKIE_POS								0x05UL		/** ACK interrupt enable */
#define USBFS_HCH5INTEN_ACKIE_MSK								(0x01UL << USBFS_HCH5INTEN_ACKIE_POS)		/** ACK interrupt enable */
#define USBFS_HCH5INTEN_USBERIE_POS								0x07UL		/** USB bus error interrupt enable */
#define USBFS_HCH5INTEN_USBERIE_MSK								(0x01UL << USBFS_HCH5INTEN_USBERIE_POS)		/** USB bus error interrupt enable */
#define USBFS_HCH5INTEN_BBERIE_POS								0x08UL		/** Babble error interrupt enable */
#define USBFS_HCH5INTEN_BBERIE_MSK								(0x01UL << USBFS_HCH5INTEN_BBERIE_POS)		/** Babble error interrupt enable */
#define USBFS_HCH5INTEN_REQOVRIE_POS								0x09UL		/** request queue overrun interrupt enable */
#define USBFS_HCH5INTEN_REQOVRIE_MSK								(0x01UL << USBFS_HCH5INTEN_REQOVRIE_POS)		/** request queue overrun interrupt enable */
#define USBFS_HCH5INTEN_DTERIE_POS								0x0AUL		/** Data toggle error interrupt enable */
#define USBFS_HCH5INTEN_DTERIE_MSK								(0x01UL << USBFS_HCH5INTEN_DTERIE_POS)		/** Data toggle error interrupt enable */
#define USBFS_HCH6INTEN_TFIE_POS								0x00UL		/** Transfer completed interrupt enable */
#define USBFS_HCH6INTEN_TFIE_MSK								(0x01UL << USBFS_HCH6INTEN_TFIE_POS)		/** Transfer completed interrupt enable */
#define USBFS_HCH6INTEN_CHIE_POS								0x01UL		/** Channel halted interrupt enable */
#define USBFS_HCH6INTEN_CHIE_MSK								(0x01UL << USBFS_HCH6INTEN_CHIE_POS)		/** Channel halted interrupt enable */
#define USBFS_HCH6INTEN_STALLIE_POS								0x03UL		/** STALL interrupt enable */
#define USBFS_HCH6INTEN_STALLIE_MSK								(0x01UL << USBFS_HCH6INTEN_STALLIE_POS)		/** STALL interrupt enable */
#define USBFS_HCH6INTEN_NAKIE_POS								0x04UL		/** NAK interrupt enable */
#define USBFS_HCH6INTEN_NAKIE_MSK								(0x01UL << USBFS_HCH6INTEN_NAKIE_POS)		/** NAK interrupt enable */
#define USBFS_HCH6INTEN_ACKIE_POS								0x05UL		/** ACK interrupt enable */
#define USBFS_HCH6INTEN_ACKIE_MSK								(0x01UL << USBFS_HCH6INTEN_ACKIE_POS)		/** ACK interrupt enable */
#define USBFS_HCH6INTEN_USBERIE_POS								0x07UL		/** USB bus error interrupt enable */
#define USBFS_HCH6INTEN_USBERIE_MSK								(0x01UL << USBFS_HCH6INTEN_USBERIE_POS)		/** USB bus error interrupt enable */
#define USBFS_HCH6INTEN_BBERIE_POS								0x08UL		/** Babble error interrupt enable */
#define USBFS_HCH6INTEN_BBERIE_MSK								(0x01UL << USBFS_HCH6INTEN_BBERIE_POS)		/** Babble error interrupt enable */
#define USBFS_HCH6INTEN_REQOVRIE_POS								0x09UL		/** request queue overrun interrupt enable */
#define USBFS_HCH6INTEN_REQOVRIE_MSK								(0x01UL << USBFS_HCH6INTEN_REQOVRIE_POS)		/** request queue overrun interrupt enable */
#define USBFS_HCH6INTEN_DTERIE_POS								0x0AUL		/** Data toggle error interrupt enable */
#define USBFS_HCH6INTEN_DTERIE_MSK								(0x01UL << USBFS_HCH6INTEN_DTERIE_POS)		/** Data toggle error interrupt enable */
#define USBFS_HCH7INTEN_TFIE_POS								0x00UL		/** Transfer completed interrupt enable */
#define USBFS_HCH7INTEN_TFIE_MSK								(0x01UL << USBFS_HCH7INTEN_TFIE_POS)		/** Transfer completed interrupt enable */
#define USBFS_HCH7INTEN_CHIE_POS								0x01UL		/** Channel halted interrupt enable */
#define USBFS_HCH7INTEN_CHIE_MSK								(0x01UL << USBFS_HCH7INTEN_CHIE_POS)		/** Channel halted interrupt enable */
#define USBFS_HCH7INTEN_STALLIE_POS								0x03UL		/** STALL interrupt enable */
#define USBFS_HCH7INTEN_STALLIE_MSK								(0x01UL << USBFS_HCH7INTEN_STALLIE_POS)		/** STALL interrupt enable */
#define USBFS_HCH7INTEN_NAKIE_POS								0x04UL		/** NAK interrupt enable */
#define USBFS_HCH7INTEN_NAKIE_MSK								(0x01UL << USBFS_HCH7INTEN_NAKIE_POS)		/** NAK interrupt enable */
#define USBFS_HCH7INTEN_ACKIE_POS								0x05UL		/** ACK interrupt enable */
#define USBFS_HCH7INTEN_ACKIE_MSK								(0x01UL << USBFS_HCH7INTEN_ACKIE_POS)		/** ACK interrupt enable */
#define USBFS_HCH7INTEN_USBERIE_POS								0x07UL		/** USB bus error interrupt enable */
#define USBFS_HCH7INTEN_USBERIE_MSK								(0x01UL << USBFS_HCH7INTEN_USBERIE_POS)		/** USB bus error interrupt enable */
#define USBFS_HCH7INTEN_BBERIE_POS								0x08UL		/** Babble error interrupt enable */
#define USBFS_HCH7INTEN_BBERIE_MSK								(0x01UL << USBFS_HCH7INTEN_BBERIE_POS)		/** Babble error interrupt enable */
#define USBFS_HCH7INTEN_REQOVRIE_POS								0x09UL		/** request queue overrun interrupt enable */
#define USBFS_HCH7INTEN_REQOVRIE_MSK								(0x01UL << USBFS_HCH7INTEN_REQOVRIE_POS)		/** request queue overrun interrupt enable */
#define USBFS_HCH7INTEN_DTERIE_POS								0x0AUL		/** Data toggle error interrupt enable */
#define USBFS_HCH7INTEN_DTERIE_MSK								(0x01UL << USBFS_HCH7INTEN_DTERIE_POS)		/** Data toggle error interrupt enable */
#define USBFS_HCH0LEN_TLEN_POS								0x00UL		/** Transfer length */
#define USBFS_HCH0LEN_TLEN_MSK								(0x7FFFFUL << USBFS_HCH0LEN_TLEN_POS)		/** Transfer length */
#define USBFS_HCH0LEN_PCNT_POS								0x13UL		/** Packet count */
#define USBFS_HCH0LEN_PCNT_MSK								(0x3FFUL << USBFS_HCH0LEN_PCNT_POS)		/** Packet count */
#define USBFS_HCH0LEN_DPID_POS								0x1DUL		/** Data PID */
#define USBFS_HCH0LEN_DPID_MSK								(0x03UL << USBFS_HCH0LEN_DPID_POS)		/** Data PID */
#define USBFS_HCH1LEN_TLEN_POS								0x00UL		/** Transfer length */
#define USBFS_HCH1LEN_TLEN_MSK								(0x7FFFFUL << USBFS_HCH1LEN_TLEN_POS)		/** Transfer length */
#define USBFS_HCH1LEN_PCNT_POS								0x13UL		/** Packet count */
#define USBFS_HCH1LEN_PCNT_MSK								(0x3FFUL << USBFS_HCH1LEN_PCNT_POS)		/** Packet count */
#define USBFS_HCH1LEN_DPID_POS								0x1DUL		/** Data PID */
#define USBFS_HCH1LEN_DPID_MSK								(0x03UL << USBFS_HCH1LEN_DPID_POS)		/** Data PID */
#define USBFS_HCH2LEN_TLEN_POS								0x00UL		/** Transfer length */
#define USBFS_HCH2LEN_TLEN_MSK								(0x7FFFFUL << USBFS_HCH2LEN_TLEN_POS)		/** Transfer length */
#define USBFS_HCH2LEN_PCNT_POS								0x13UL		/** Packet count */
#define USBFS_HCH2LEN_PCNT_MSK								(0x3FFUL << USBFS_HCH2LEN_PCNT_POS)		/** Packet count */
#define USBFS_HCH2LEN_DPID_POS								0x1DUL		/** Data PID */
#define USBFS_HCH2LEN_DPID_MSK								(0x03UL << USBFS_HCH2LEN_DPID_POS)		/** Data PID */
#define USBFS_HCH3LEN_TLEN_POS								0x00UL		/** Transfer length */
#define USBFS_HCH3LEN_TLEN_MSK								(0x7FFFFUL << USBFS_HCH3LEN_TLEN_POS)		/** Transfer length */
#define USBFS_HCH3LEN_PCNT_POS								0x13UL		/** Packet count */
#define USBFS_HCH3LEN_PCNT_MSK								(0x3FFUL << USBFS_HCH3LEN_PCNT_POS)		/** Packet count */
#define USBFS_HCH3LEN_DPID_POS								0x1DUL		/** Data PID */
#define USBFS_HCH3LEN_DPID_MSK								(0x03UL << USBFS_HCH3LEN_DPID_POS)		/** Data PID */
#define USBFS_HCH4LEN_TLEN_POS								0x00UL		/** Transfer length */
#define USBFS_HCH4LEN_TLEN_MSK								(0x7FFFFUL << USBFS_HCH4LEN_TLEN_POS)		/** Transfer length */
#define USBFS_HCH4LEN_PCNT_POS								0x13UL		/** Packet count */
#define USBFS_HCH4LEN_PCNT_MSK								(0x3FFUL << USBFS_HCH4LEN_PCNT_POS)		/** Packet count */
#define USBFS_HCH4LEN_DPID_POS								0x1DUL		/** Data PID */
#define USBFS_HCH4LEN_DPID_MSK								(0x03UL << USBFS_HCH4LEN_DPID_POS)		/** Data PID */
#define USBFS_HCH5LEN_TLEN_POS								0x00UL		/** Transfer length */
#define USBFS_HCH5LEN_TLEN_MSK								(0x7FFFFUL << USBFS_HCH5LEN_TLEN_POS)		/** Transfer length */
#define USBFS_HCH5LEN_PCNT_POS								0x13UL		/** Packet count */
#define USBFS_HCH5LEN_PCNT_MSK								(0x3FFUL << USBFS_HCH5LEN_PCNT_POS)		/** Packet count */
#define USBFS_HCH5LEN_DPID_POS								0x1DUL		/** Data PID */
#define USBFS_HCH5LEN_DPID_MSK								(0x03UL << USBFS_HCH5LEN_DPID_POS)		/** Data PID */
#define USBFS_HCH6LEN_TLEN_POS								0x00UL		/** Transfer length */
#define USBFS_HCH6LEN_TLEN_MSK								(0x7FFFFUL << USBFS_HCH6LEN_TLEN_POS)		/** Transfer length */
#define USBFS_HCH6LEN_PCNT_POS								0x13UL		/** Packet count */
#define USBFS_HCH6LEN_PCNT_MSK								(0x3FFUL << USBFS_HCH6LEN_PCNT_POS)		/** Packet count */
#define USBFS_HCH6LEN_DPID_POS								0x1DUL		/** Data PID */
#define USBFS_HCH6LEN_DPID_MSK								(0x03UL << USBFS_HCH6LEN_DPID_POS)		/** Data PID */
#define USBFS_HCH7LEN_TLEN_POS								0x00UL		/** Transfer length */
#define USBFS_HCH7LEN_TLEN_MSK								(0x7FFFFUL << USBFS_HCH7LEN_TLEN_POS)		/** Transfer length */
#define USBFS_HCH7LEN_PCNT_POS								0x13UL		/** Packet count */
#define USBFS_HCH7LEN_PCNT_MSK								(0x3FFUL << USBFS_HCH7LEN_PCNT_POS)		/** Packet count */
#define USBFS_HCH7LEN_DPID_POS								0x1DUL		/** Data PID */
#define USBFS_HCH7LEN_DPID_MSK								(0x03UL << USBFS_HCH7LEN_DPID_POS)		/** Data PID */
#define USBFS_DCFG_DS_POS								0x00UL		/** Device speed */
#define USBFS_DCFG_DS_MSK								(0x03UL << USBFS_DCFG_DS_POS)		/** Device speed */
#define USBFS_DCFG_NZLSOH_POS								0x02UL		/** Non-zero-length status OUT handshake */
#define USBFS_DCFG_NZLSOH_MSK								(0x01UL << USBFS_DCFG_NZLSOH_POS)		/** Non-zero-length status OUT handshake */
#define USBFS_DCFG_DAR_POS								0x04UL		/** Device address */
#define USBFS_DCFG_DAR_MSK								(0x7FUL << USBFS_DCFG_DAR_POS)		/** Device address */
#define USBFS_DCFG_EOPFT_POS								0x0BUL		/** end of periodic frame time */
#define USBFS_DCFG_EOPFT_MSK								(0x03UL << USBFS_DCFG_EOPFT_POS)		/** end of periodic frame time */
#define USBFS_DCTL_RWKUP_POS								0x00UL		/** Remote wakeup */
#define USBFS_DCTL_RWKUP_MSK								(0x01UL << USBFS_DCTL_RWKUP_POS)		/** Remote wakeup */
#define USBFS_DCTL_SD_POS								0x01UL		/** Soft disconnect */
#define USBFS_DCTL_SD_MSK								(0x01UL << USBFS_DCTL_SD_POS)		/** Soft disconnect */
#define USBFS_DCTL_GINS_POS								0x02UL		/** Global IN NAK status */
#define USBFS_DCTL_GINS_MSK								(0x01UL << USBFS_DCTL_GINS_POS)		/** Global IN NAK status */
#define USBFS_DCTL_GONS_POS								0x03UL		/** Global OUT NAK status */
#define USBFS_DCTL_GONS_MSK								(0x01UL << USBFS_DCTL_GONS_POS)		/** Global OUT NAK status */
#define USBFS_DCTL_SGINAK_POS								0x07UL		/** Set global IN NAK */
#define USBFS_DCTL_SGINAK_MSK								(0x01UL << USBFS_DCTL_SGINAK_POS)		/** Set global IN NAK */
#define USBFS_DCTL_CGINAK_POS								0x08UL		/** Clear global IN NAK */
#define USBFS_DCTL_CGINAK_MSK								(0x01UL << USBFS_DCTL_CGINAK_POS)		/** Clear global IN NAK */
#define USBFS_DCTL_SGONAK_POS								0x09UL		/** Set global OUT NAK */
#define USBFS_DCTL_SGONAK_MSK								(0x01UL << USBFS_DCTL_SGONAK_POS)		/** Set global OUT NAK */
#define USBFS_DCTL_CGONAK_POS								0x0AUL		/** Clear global OUT NAK */
#define USBFS_DCTL_CGONAK_MSK								(0x01UL << USBFS_DCTL_CGONAK_POS)		/** Clear global OUT NAK */
#define USBFS_DCTL_POIF_POS								0x0BUL		/** Power-on initialization flag */
#define USBFS_DCTL_POIF_MSK								(0x01UL << USBFS_DCTL_POIF_POS)		/** Power-on initialization flag */
#define USBFS_DSTAT_SPST_POS								0x00UL		/** Suspend status */
#define USBFS_DSTAT_SPST_MSK								(0x01UL << USBFS_DSTAT_SPST_POS)		/** Suspend status */
#define USBFS_DSTAT_ES_POS								0x01UL		/** Enumerated speed */
#define USBFS_DSTAT_ES_MSK								(0x03UL << USBFS_DSTAT_ES_POS)		/** Enumerated speed */
#define USBFS_DSTAT_FNRSOF_POS								0x08UL		/** Frame number of the received SOF */
#define USBFS_DSTAT_FNRSOF_MSK								(0x3FFFUL << USBFS_DSTAT_FNRSOF_POS)		/** Frame number of the received SOF */
#define USBFS_DIEPINTEN_TFEN_POS								0x00UL		/** Transfer finished interrupt enable */
#define USBFS_DIEPINTEN_TFEN_MSK								(0x01UL << USBFS_DIEPINTEN_TFEN_POS)		/** Transfer finished interrupt enable */
#define USBFS_DIEPINTEN_EPDISEN_POS								0x01UL		/** Endpoint disabled interrupt enable */
#define USBFS_DIEPINTEN_EPDISEN_MSK								(0x01UL << USBFS_DIEPINTEN_EPDISEN_POS)		/** Endpoint disabled interrupt enable */
#define USBFS_DIEPINTEN_CITOEN_POS								0x03UL		/** Control IN timeout condition interrupt enable (Non-isochronous endpoints) */
#define USBFS_DIEPINTEN_CITOEN_MSK								(0x01UL << USBFS_DIEPINTEN_CITOEN_POS)		/** Control IN timeout condition interrupt enable (Non-isochronous endpoints) */
#define USBFS_DIEPINTEN_EPTXFUDEN_POS								0x04UL		/** Endpoint Tx FIFO underrun interrupt enable bit */
#define USBFS_DIEPINTEN_EPTXFUDEN_MSK								(0x01UL << USBFS_DIEPINTEN_EPTXFUDEN_POS)		/** Endpoint Tx FIFO underrun interrupt enable bit */
#define USBFS_DIEPINTEN_IEPNEEN_POS								0x06UL		/** IN endpoint NAK effective interrupt enable */
#define USBFS_DIEPINTEN_IEPNEEN_MSK								(0x01UL << USBFS_DIEPINTEN_IEPNEEN_POS)		/** IN endpoint NAK effective interrupt enable */
#define USBFS_DOEPINTEN_TFEN_POS								0x00UL		/** Transfer finished interrupt enable */
#define USBFS_DOEPINTEN_TFEN_MSK								(0x01UL << USBFS_DOEPINTEN_TFEN_POS)		/** Transfer finished interrupt enable */
#define USBFS_DOEPINTEN_EPDISEN_POS								0x01UL		/** Endpoint disabled interrupt enable */
#define USBFS_DOEPINTEN_EPDISEN_MSK								(0x01UL << USBFS_DOEPINTEN_EPDISEN_POS)		/** Endpoint disabled interrupt enable */
#define USBFS_DOEPINTEN_STPFEN_POS								0x03UL		/** SETUP phase finished interrupt enable */
#define USBFS_DOEPINTEN_STPFEN_MSK								(0x01UL << USBFS_DOEPINTEN_STPFEN_POS)		/** SETUP phase finished interrupt enable */
#define USBFS_DOEPINTEN_EPRXFOVREN_POS								0x04UL		/**  Endpoint Rx FIFO overrun interrupt enable */
#define USBFS_DOEPINTEN_EPRXFOVREN_MSK								(0x01UL << USBFS_DOEPINTEN_EPRXFOVREN_POS)		/**  Endpoint Rx FIFO overrun interrupt enable */
#define USBFS_DOEPINTEN_BTBSTPEN_POS								0x06UL		/**  Back-to-back SETUP packets interrupt enable */
#define USBFS_DOEPINTEN_BTBSTPEN_MSK								(0x01UL << USBFS_DOEPINTEN_BTBSTPEN_POS)		/**  Back-to-back SETUP packets interrupt enable */
#define USBFS_DAEPINT_IEPITB_POS								0x00UL		/** Device all IN endpoint interrupt bits */
#define USBFS_DAEPINT_IEPITB_MSK								(0x0FUL << USBFS_DAEPINT_IEPITB_POS)		/** Device all IN endpoint interrupt bits */
#define USBFS_DAEPINT_OEPITB_POS								0x10UL		/** Device all OUT endpoint interrupt bits */
#define USBFS_DAEPINT_OEPITB_MSK								(0x0FUL << USBFS_DAEPINT_OEPITB_POS)		/** Device all OUT endpoint interrupt bits */
#define USBFS_DAEPINTEN_IEPIE_POS								0x00UL		/** IN EP interrupt interrupt enable bits */
#define USBFS_DAEPINTEN_IEPIE_MSK								(0x0FUL << USBFS_DAEPINTEN_IEPIE_POS)		/** IN EP interrupt interrupt enable bits */
#define USBFS_DAEPINTEN_OEPIE_POS								0x10UL		/** OUT endpoint interrupt enable bits */
#define USBFS_DAEPINTEN_OEPIE_MSK								(0x0FUL << USBFS_DAEPINTEN_OEPIE_POS)		/** OUT endpoint interrupt enable bits */
#define USBFS_DVBUSDT_DVBUSDT_POS								0x00UL		/** Device VBUS discharge time */
#define USBFS_DVBUSDT_DVBUSDT_MSK								(0xFFFFUL << USBFS_DVBUSDT_DVBUSDT_POS)		/** Device VBUS discharge time */
#define USBFS_DVBUSPT_DVBUSPT_POS								0x00UL		/** Device VBUS pulsing time */
#define USBFS_DVBUSPT_DVBUSPT_MSK								(0xFFFUL << USBFS_DVBUSPT_DVBUSPT_POS)		/** Device VBUS pulsing time */
#define USBFS_DIEPFEINTEN_IEPTXFEIE_POS								0x00UL		/** IN EP Tx FIFO empty interrupt enable bits */
#define USBFS_DIEPFEINTEN_IEPTXFEIE_MSK								(0x0FUL << USBFS_DIEPFEINTEN_IEPTXFEIE_POS)		/** IN EP Tx FIFO empty interrupt enable bits */
#define USBFS_DIEP0CTL_MPL_POS								0x00UL		/** Maximum packet length */
#define USBFS_DIEP0CTL_MPL_MSK								(0x03UL << USBFS_DIEP0CTL_MPL_POS)		/** Maximum packet length */
#define USBFS_DIEP0CTL_EPACT_POS								0x0FUL		/** endpoint active */
#define USBFS_DIEP0CTL_EPACT_MSK								(0x01UL << USBFS_DIEP0CTL_EPACT_POS)		/** endpoint active */
#define USBFS_DIEP0CTL_NAKS_POS								0x11UL		/** NAK status */
#define USBFS_DIEP0CTL_NAKS_MSK								(0x01UL << USBFS_DIEP0CTL_NAKS_POS)		/** NAK status */
#define USBFS_DIEP0CTL_EPTYPE_POS								0x12UL		/** Endpoint type */
#define USBFS_DIEP0CTL_EPTYPE_MSK								(0x03UL << USBFS_DIEP0CTL_EPTYPE_POS)		/** Endpoint type */
#define USBFS_DIEP0CTL_STALL_POS								0x15UL		/** STALL handshake */
#define USBFS_DIEP0CTL_STALL_MSK								(0x01UL << USBFS_DIEP0CTL_STALL_POS)		/** STALL handshake */
#define USBFS_DIEP0CTL_TXFNUM_POS								0x16UL		/** TxFIFO number */
#define USBFS_DIEP0CTL_TXFNUM_MSK								(0x0FUL << USBFS_DIEP0CTL_TXFNUM_POS)		/** TxFIFO number */
#define USBFS_DIEP0CTL_CNAK_POS								0x1AUL		/** Clear NAK */
#define USBFS_DIEP0CTL_CNAK_MSK								(0x01UL << USBFS_DIEP0CTL_CNAK_POS)		/** Clear NAK */
#define USBFS_DIEP0CTL_SNAK_POS								0x1BUL		/** Set NAK */
#define USBFS_DIEP0CTL_SNAK_MSK								(0x01UL << USBFS_DIEP0CTL_SNAK_POS)		/** Set NAK */
#define USBFS_DIEP0CTL_EPD_POS								0x1EUL		/** Endpoint disable */
#define USBFS_DIEP0CTL_EPD_MSK								(0x01UL << USBFS_DIEP0CTL_EPD_POS)		/** Endpoint disable */
#define USBFS_DIEP0CTL_EPEN_POS								0x1FUL		/** Endpoint enable */
#define USBFS_DIEP0CTL_EPEN_MSK								(0x01UL << USBFS_DIEP0CTL_EPEN_POS)		/** Endpoint enable */
#define USBFS_DIEP1CTL_EPEN_POS								0x1FUL		/** Endpoint enable */
#define USBFS_DIEP1CTL_EPEN_MSK								(0x01UL << USBFS_DIEP1CTL_EPEN_POS)		/** Endpoint enable */
#define USBFS_DIEP1CTL_EPD_POS								0x1EUL		/** Endpoint disable */
#define USBFS_DIEP1CTL_EPD_MSK								(0x01UL << USBFS_DIEP1CTL_EPD_POS)		/** Endpoint disable */
#define USBFS_DIEP1CTL_SD1PID_SODDFRM_POS								0x1DUL		/** Set DATA1 PID/Set odd frame */
#define USBFS_DIEP1CTL_SD1PID_SODDFRM_MSK								(0x01UL << USBFS_DIEP1CTL_SD1PID_SODDFRM_POS)		/** Set DATA1 PID/Set odd frame */
#define USBFS_DIEP1CTL_SD0PID_SEVENFRM_POS								0x1CUL		/** SD0PID/SEVNFRM */
#define USBFS_DIEP1CTL_SD0PID_SEVENFRM_MSK								(0x01UL << USBFS_DIEP1CTL_SD0PID_SEVENFRM_POS)		/** SD0PID/SEVNFRM */
#define USBFS_DIEP1CTL_SNAK_POS								0x1BUL		/** Set NAK */
#define USBFS_DIEP1CTL_SNAK_MSK								(0x01UL << USBFS_DIEP1CTL_SNAK_POS)		/** Set NAK */
#define USBFS_DIEP1CTL_CNAK_POS								0x1AUL		/** Clear NAK */
#define USBFS_DIEP1CTL_CNAK_MSK								(0x01UL << USBFS_DIEP1CTL_CNAK_POS)		/** Clear NAK */
#define USBFS_DIEP1CTL_TXFNUM_POS								0x16UL		/** Tx FIFO number */
#define USBFS_DIEP1CTL_TXFNUM_MSK								(0x0FUL << USBFS_DIEP1CTL_TXFNUM_POS)		/** Tx FIFO number */
#define USBFS_DIEP1CTL_STALL_POS								0x15UL		/** STALL handshake */
#define USBFS_DIEP1CTL_STALL_MSK								(0x01UL << USBFS_DIEP1CTL_STALL_POS)		/** STALL handshake */
#define USBFS_DIEP1CTL_EPTYPE_POS								0x12UL		/** Endpoint type */
#define USBFS_DIEP1CTL_EPTYPE_MSK								(0x03UL << USBFS_DIEP1CTL_EPTYPE_POS)		/** Endpoint type */
#define USBFS_DIEP1CTL_NAKS_POS								0x11UL		/** NAK status */
#define USBFS_DIEP1CTL_NAKS_MSK								(0x01UL << USBFS_DIEP1CTL_NAKS_POS)		/** NAK status */
#define USBFS_DIEP1CTL_EOFRM_DPID_POS								0x10UL		/** EOFRM/DPID */
#define USBFS_DIEP1CTL_EOFRM_DPID_MSK								(0x01UL << USBFS_DIEP1CTL_EOFRM_DPID_POS)		/** EOFRM/DPID */
#define USBFS_DIEP1CTL_EPACT_POS								0x0FUL		/** Endpoint active */
#define USBFS_DIEP1CTL_EPACT_MSK								(0x01UL << USBFS_DIEP1CTL_EPACT_POS)		/** Endpoint active */
#define USBFS_DIEP1CTL_MPL_POS								0x00UL		/** maximum packet length */
#define USBFS_DIEP1CTL_MPL_MSK								(0x7FFUL << USBFS_DIEP1CTL_MPL_POS)		/** maximum packet length */
#define USBFS_DIEP2CTL_EPEN_POS								0x1FUL		/** Endpoint enable */
#define USBFS_DIEP2CTL_EPEN_MSK								(0x01UL << USBFS_DIEP2CTL_EPEN_POS)		/** Endpoint enable */
#define USBFS_DIEP2CTL_EPD_POS								0x1EUL		/** Endpoint disable */
#define USBFS_DIEP2CTL_EPD_MSK								(0x01UL << USBFS_DIEP2CTL_EPD_POS)		/** Endpoint disable */
#define USBFS_DIEP2CTL_SD1PID_SODDFRM_POS								0x1DUL		/** Set DATA1 PID/Set odd frame */
#define USBFS_DIEP2CTL_SD1PID_SODDFRM_MSK								(0x01UL << USBFS_DIEP2CTL_SD1PID_SODDFRM_POS)		/** Set DATA1 PID/Set odd frame */
#define USBFS_DIEP2CTL_SD0PID_SEVENFRM_POS								0x1CUL		/** SD0PID/SEVNFRM */
#define USBFS_DIEP2CTL_SD0PID_SEVENFRM_MSK								(0x01UL << USBFS_DIEP2CTL_SD0PID_SEVENFRM_POS)		/** SD0PID/SEVNFRM */
#define USBFS_DIEP2CTL_SNAK_POS								0x1BUL		/** Set NAK */
#define USBFS_DIEP2CTL_SNAK_MSK								(0x01UL << USBFS_DIEP2CTL_SNAK_POS)		/** Set NAK */
#define USBFS_DIEP2CTL_CNAK_POS								0x1AUL		/** Clear NAK */
#define USBFS_DIEP2CTL_CNAK_MSK								(0x01UL << USBFS_DIEP2CTL_CNAK_POS)		/** Clear NAK */
#define USBFS_DIEP2CTL_TXFNUM_POS								0x16UL		/** Tx FIFO number */
#define USBFS_DIEP2CTL_TXFNUM_MSK								(0x0FUL << USBFS_DIEP2CTL_TXFNUM_POS)		/** Tx FIFO number */
#define USBFS_DIEP2CTL_STALL_POS								0x15UL		/** STALL handshake */
#define USBFS_DIEP2CTL_STALL_MSK								(0x01UL << USBFS_DIEP2CTL_STALL_POS)		/** STALL handshake */
#define USBFS_DIEP2CTL_EPTYPE_POS								0x12UL		/** Endpoint type */
#define USBFS_DIEP2CTL_EPTYPE_MSK								(0x03UL << USBFS_DIEP2CTL_EPTYPE_POS)		/** Endpoint type */
#define USBFS_DIEP2CTL_NAKS_POS								0x11UL		/** NAK status */
#define USBFS_DIEP2CTL_NAKS_MSK								(0x01UL << USBFS_DIEP2CTL_NAKS_POS)		/** NAK status */
#define USBFS_DIEP2CTL_EOFRM_DPID_POS								0x10UL		/** EOFRM/DPID */
#define USBFS_DIEP2CTL_EOFRM_DPID_MSK								(0x01UL << USBFS_DIEP2CTL_EOFRM_DPID_POS)		/** EOFRM/DPID */
#define USBFS_DIEP2CTL_EPACT_POS								0x0FUL		/** Endpoint active */
#define USBFS_DIEP2CTL_EPACT_MSK								(0x01UL << USBFS_DIEP2CTL_EPACT_POS)		/** Endpoint active */
#define USBFS_DIEP2CTL_MPL_POS								0x00UL		/** maximum packet length */
#define USBFS_DIEP2CTL_MPL_MSK								(0x7FFUL << USBFS_DIEP2CTL_MPL_POS)		/** maximum packet length */
#define USBFS_DIEP3CTL_EPEN_POS								0x1FUL		/** Endpoint enable */
#define USBFS_DIEP3CTL_EPEN_MSK								(0x01UL << USBFS_DIEP3CTL_EPEN_POS)		/** Endpoint enable */
#define USBFS_DIEP3CTL_EPD_POS								0x1EUL		/** Endpoint disable */
#define USBFS_DIEP3CTL_EPD_MSK								(0x01UL << USBFS_DIEP3CTL_EPD_POS)		/** Endpoint disable */
#define USBFS_DIEP3CTL_SD1PID_SODDFRM_POS								0x1DUL		/** Set DATA1 PID/Set odd frame */
#define USBFS_DIEP3CTL_SD1PID_SODDFRM_MSK								(0x01UL << USBFS_DIEP3CTL_SD1PID_SODDFRM_POS)		/** Set DATA1 PID/Set odd frame */
#define USBFS_DIEP3CTL_SD0PID_SEVENFRM_POS								0x1CUL		/** SD0PID/SEVNFRM */
#define USBFS_DIEP3CTL_SD0PID_SEVENFRM_MSK								(0x01UL << USBFS_DIEP3CTL_SD0PID_SEVENFRM_POS)		/** SD0PID/SEVNFRM */
#define USBFS_DIEP3CTL_SNAK_POS								0x1BUL		/** Set NAK */
#define USBFS_DIEP3CTL_SNAK_MSK								(0x01UL << USBFS_DIEP3CTL_SNAK_POS)		/** Set NAK */
#define USBFS_DIEP3CTL_CNAK_POS								0x1AUL		/** Clear NAK */
#define USBFS_DIEP3CTL_CNAK_MSK								(0x01UL << USBFS_DIEP3CTL_CNAK_POS)		/** Clear NAK */
#define USBFS_DIEP3CTL_TXFNUM_POS								0x16UL		/** Tx FIFO number */
#define USBFS_DIEP3CTL_TXFNUM_MSK								(0x0FUL << USBFS_DIEP3CTL_TXFNUM_POS)		/** Tx FIFO number */
#define USBFS_DIEP3CTL_STALL_POS								0x15UL		/** STALL handshake */
#define USBFS_DIEP3CTL_STALL_MSK								(0x01UL << USBFS_DIEP3CTL_STALL_POS)		/** STALL handshake */
#define USBFS_DIEP3CTL_EPTYPE_POS								0x12UL		/** Endpoint type */
#define USBFS_DIEP3CTL_EPTYPE_MSK								(0x03UL << USBFS_DIEP3CTL_EPTYPE_POS)		/** Endpoint type */
#define USBFS_DIEP3CTL_NAKS_POS								0x11UL		/** NAK status */
#define USBFS_DIEP3CTL_NAKS_MSK								(0x01UL << USBFS_DIEP3CTL_NAKS_POS)		/** NAK status */
#define USBFS_DIEP3CTL_EOFRM_DPID_POS								0x10UL		/** EOFRM/DPID */
#define USBFS_DIEP3CTL_EOFRM_DPID_MSK								(0x01UL << USBFS_DIEP3CTL_EOFRM_DPID_POS)		/** EOFRM/DPID */
#define USBFS_DIEP3CTL_EPACT_POS								0x0FUL		/** Endpoint active */
#define USBFS_DIEP3CTL_EPACT_MSK								(0x01UL << USBFS_DIEP3CTL_EPACT_POS)		/** Endpoint active */
#define USBFS_DIEP3CTL_MPL_POS								0x00UL		/** maximum packet length */
#define USBFS_DIEP3CTL_MPL_MSK								(0x7FFUL << USBFS_DIEP3CTL_MPL_POS)		/** maximum packet length */
#define USBFS_DOEP0CTL_EPEN_POS								0x1FUL		/** Endpoint enable */
#define USBFS_DOEP0CTL_EPEN_MSK								(0x01UL << USBFS_DOEP0CTL_EPEN_POS)		/** Endpoint enable */
#define USBFS_DOEP0CTL_EPD_POS								0x1EUL		/** Endpoint disable */
#define USBFS_DOEP0CTL_EPD_MSK								(0x01UL << USBFS_DOEP0CTL_EPD_POS)		/** Endpoint disable */
#define USBFS_DOEP0CTL_SNAK_POS								0x1BUL		/** Set NAK */
#define USBFS_DOEP0CTL_SNAK_MSK								(0x01UL << USBFS_DOEP0CTL_SNAK_POS)		/** Set NAK */
#define USBFS_DOEP0CTL_CNAK_POS								0x1AUL		/** Clear NAK */
#define USBFS_DOEP0CTL_CNAK_MSK								(0x01UL << USBFS_DOEP0CTL_CNAK_POS)		/** Clear NAK */
#define USBFS_DOEP0CTL_STALL_POS								0x15UL		/** STALL handshake */
#define USBFS_DOEP0CTL_STALL_MSK								(0x01UL << USBFS_DOEP0CTL_STALL_POS)		/** STALL handshake */
#define USBFS_DOEP0CTL_SNOOP_POS								0x14UL		/** Snoop mode */
#define USBFS_DOEP0CTL_SNOOP_MSK								(0x01UL << USBFS_DOEP0CTL_SNOOP_POS)		/** Snoop mode */
#define USBFS_DOEP0CTL_EPTYPE_POS								0x12UL		/** Endpoint type */
#define USBFS_DOEP0CTL_EPTYPE_MSK								(0x03UL << USBFS_DOEP0CTL_EPTYPE_POS)		/** Endpoint type */
#define USBFS_DOEP0CTL_NAKS_POS								0x11UL		/** NAK status */
#define USBFS_DOEP0CTL_NAKS_MSK								(0x01UL << USBFS_DOEP0CTL_NAKS_POS)		/** NAK status */
#define USBFS_DOEP0CTL_EPACT_POS								0x0FUL		/** Endpoint active */
#define USBFS_DOEP0CTL_EPACT_MSK								(0x01UL << USBFS_DOEP0CTL_EPACT_POS)		/** Endpoint active */
#define USBFS_DOEP0CTL_MPL_POS								0x00UL		/** Maximum packet length */
#define USBFS_DOEP0CTL_MPL_MSK								(0x03UL << USBFS_DOEP0CTL_MPL_POS)		/** Maximum packet length */
#define USBFS_DOEP1CTL_EPEN_POS								0x1FUL		/** Endpoint enable */
#define USBFS_DOEP1CTL_EPEN_MSK								(0x01UL << USBFS_DOEP1CTL_EPEN_POS)		/** Endpoint enable */
#define USBFS_DOEP1CTL_EPD_POS								0x1EUL		/** Endpoint disable */
#define USBFS_DOEP1CTL_EPD_MSK								(0x01UL << USBFS_DOEP1CTL_EPD_POS)		/** Endpoint disable */
#define USBFS_DOEP1CTL_SD1PID_SODDFRM_POS								0x1DUL		/** SD1PID/SODDFRM */
#define USBFS_DOEP1CTL_SD1PID_SODDFRM_MSK								(0x01UL << USBFS_DOEP1CTL_SD1PID_SODDFRM_POS)		/** SD1PID/SODDFRM */
#define USBFS_DOEP1CTL_SD0PID_SEVENFRM_POS								0x1CUL		/** SD0PID/SEVENFRM */
#define USBFS_DOEP1CTL_SD0PID_SEVENFRM_MSK								(0x01UL << USBFS_DOEP1CTL_SD0PID_SEVENFRM_POS)		/** SD0PID/SEVENFRM */
#define USBFS_DOEP1CTL_SNAK_POS								0x1BUL		/** Set NAK */
#define USBFS_DOEP1CTL_SNAK_MSK								(0x01UL << USBFS_DOEP1CTL_SNAK_POS)		/** Set NAK */
#define USBFS_DOEP1CTL_CNAK_POS								0x1AUL		/** Clear NAK */
#define USBFS_DOEP1CTL_CNAK_MSK								(0x01UL << USBFS_DOEP1CTL_CNAK_POS)		/** Clear NAK */
#define USBFS_DOEP1CTL_STALL_POS								0x15UL		/** STALL handshake */
#define USBFS_DOEP1CTL_STALL_MSK								(0x01UL << USBFS_DOEP1CTL_STALL_POS)		/** STALL handshake */
#define USBFS_DOEP1CTL_SNOOP_POS								0x14UL		/** Snoop mode */
#define USBFS_DOEP1CTL_SNOOP_MSK								(0x01UL << USBFS_DOEP1CTL_SNOOP_POS)		/** Snoop mode */
#define USBFS_DOEP1CTL_EPTYPE_POS								0x12UL		/** Endpoint type */
#define USBFS_DOEP1CTL_EPTYPE_MSK								(0x03UL << USBFS_DOEP1CTL_EPTYPE_POS)		/** Endpoint type */
#define USBFS_DOEP1CTL_NAKS_POS								0x11UL		/** NAK status */
#define USBFS_DOEP1CTL_NAKS_MSK								(0x01UL << USBFS_DOEP1CTL_NAKS_POS)		/** NAK status */
#define USBFS_DOEP1CTL_EOFRM_DPID_POS								0x10UL		/** EOFRM/DPID */
#define USBFS_DOEP1CTL_EOFRM_DPID_MSK								(0x01UL << USBFS_DOEP1CTL_EOFRM_DPID_POS)		/** EOFRM/DPID */
#define USBFS_DOEP1CTL_EPACT_POS								0x0FUL		/** Endpoint active */
#define USBFS_DOEP1CTL_EPACT_MSK								(0x01UL << USBFS_DOEP1CTL_EPACT_POS)		/** Endpoint active */
#define USBFS_DOEP1CTL_MPL_POS								0x00UL		/** maximum packet length */
#define USBFS_DOEP1CTL_MPL_MSK								(0x7FFUL << USBFS_DOEP1CTL_MPL_POS)		/** maximum packet length */
#define USBFS_DOEP2CTL_EPEN_POS								0x1FUL		/** Endpoint enable */
#define USBFS_DOEP2CTL_EPEN_MSK								(0x01UL << USBFS_DOEP2CTL_EPEN_POS)		/** Endpoint enable */
#define USBFS_DOEP2CTL_EPD_POS								0x1EUL		/** Endpoint disable */
#define USBFS_DOEP2CTL_EPD_MSK								(0x01UL << USBFS_DOEP2CTL_EPD_POS)		/** Endpoint disable */
#define USBFS_DOEP2CTL_SD1PID_SODDFRM_POS								0x1DUL		/** SD1PID/SODDFRM */
#define USBFS_DOEP2CTL_SD1PID_SODDFRM_MSK								(0x01UL << USBFS_DOEP2CTL_SD1PID_SODDFRM_POS)		/** SD1PID/SODDFRM */
#define USBFS_DOEP2CTL_SD0PID_SEVENFRM_POS								0x1CUL		/** SD0PID/SEVENFRM */
#define USBFS_DOEP2CTL_SD0PID_SEVENFRM_MSK								(0x01UL << USBFS_DOEP2CTL_SD0PID_SEVENFRM_POS)		/** SD0PID/SEVENFRM */
#define USBFS_DOEP2CTL_SNAK_POS								0x1BUL		/** Set NAK */
#define USBFS_DOEP2CTL_SNAK_MSK								(0x01UL << USBFS_DOEP2CTL_SNAK_POS)		/** Set NAK */
#define USBFS_DOEP2CTL_CNAK_POS								0x1AUL		/** Clear NAK */
#define USBFS_DOEP2CTL_CNAK_MSK								(0x01UL << USBFS_DOEP2CTL_CNAK_POS)		/** Clear NAK */
#define USBFS_DOEP2CTL_STALL_POS								0x15UL		/** STALL handshake */
#define USBFS_DOEP2CTL_STALL_MSK								(0x01UL << USBFS_DOEP2CTL_STALL_POS)		/** STALL handshake */
#define USBFS_DOEP2CTL_SNOOP_POS								0x14UL		/** Snoop mode */
#define USBFS_DOEP2CTL_SNOOP_MSK								(0x01UL << USBFS_DOEP2CTL_SNOOP_POS)		/** Snoop mode */
#define USBFS_DOEP2CTL_EPTYPE_POS								0x12UL		/** Endpoint type */
#define USBFS_DOEP2CTL_EPTYPE_MSK								(0x03UL << USBFS_DOEP2CTL_EPTYPE_POS)		/** Endpoint type */
#define USBFS_DOEP2CTL_NAKS_POS								0x11UL		/** NAK status */
#define USBFS_DOEP2CTL_NAKS_MSK								(0x01UL << USBFS_DOEP2CTL_NAKS_POS)		/** NAK status */
#define USBFS_DOEP2CTL_EOFRM_DPID_POS								0x10UL		/** EOFRM/DPID */
#define USBFS_DOEP2CTL_EOFRM_DPID_MSK								(0x01UL << USBFS_DOEP2CTL_EOFRM_DPID_POS)		/** EOFRM/DPID */
#define USBFS_DOEP2CTL_EPACT_POS								0x0FUL		/** Endpoint active */
#define USBFS_DOEP2CTL_EPACT_MSK								(0x01UL << USBFS_DOEP2CTL_EPACT_POS)		/** Endpoint active */
#define USBFS_DOEP2CTL_MPL_POS								0x00UL		/** maximum packet length */
#define USBFS_DOEP2CTL_MPL_MSK								(0x7FFUL << USBFS_DOEP2CTL_MPL_POS)		/** maximum packet length */
#define USBFS_DOEP3CTL_EPEN_POS								0x1FUL		/** Endpoint enable */
#define USBFS_DOEP3CTL_EPEN_MSK								(0x01UL << USBFS_DOEP3CTL_EPEN_POS)		/** Endpoint enable */
#define USBFS_DOEP3CTL_EPD_POS								0x1EUL		/** Endpoint disable */
#define USBFS_DOEP3CTL_EPD_MSK								(0x01UL << USBFS_DOEP3CTL_EPD_POS)		/** Endpoint disable */
#define USBFS_DOEP3CTL_SD1PID_SODDFRM_POS								0x1DUL		/** SD1PID/SODDFRM */
#define USBFS_DOEP3CTL_SD1PID_SODDFRM_MSK								(0x01UL << USBFS_DOEP3CTL_SD1PID_SODDFRM_POS)		/** SD1PID/SODDFRM */
#define USBFS_DOEP3CTL_SD0PID_SEVENFRM_POS								0x1CUL		/** SD0PID/SEVENFRM */
#define USBFS_DOEP3CTL_SD0PID_SEVENFRM_MSK								(0x01UL << USBFS_DOEP3CTL_SD0PID_SEVENFRM_POS)		/** SD0PID/SEVENFRM */
#define USBFS_DOEP3CTL_SNAK_POS								0x1BUL		/** Set NAK */
#define USBFS_DOEP3CTL_SNAK_MSK								(0x01UL << USBFS_DOEP3CTL_SNAK_POS)		/** Set NAK */
#define USBFS_DOEP3CTL_CNAK_POS								0x1AUL		/** Clear NAK */
#define USBFS_DOEP3CTL_CNAK_MSK								(0x01UL << USBFS_DOEP3CTL_CNAK_POS)		/** Clear NAK */
#define USBFS_DOEP3CTL_STALL_POS								0x15UL		/** STALL handshake */
#define USBFS_DOEP3CTL_STALL_MSK								(0x01UL << USBFS_DOEP3CTL_STALL_POS)		/** STALL handshake */
#define USBFS_DOEP3CTL_SNOOP_POS								0x14UL		/** Snoop mode */
#define USBFS_DOEP3CTL_SNOOP_MSK								(0x01UL << USBFS_DOEP3CTL_SNOOP_POS)		/** Snoop mode */
#define USBFS_DOEP3CTL_EPTYPE_POS								0x12UL		/** Endpoint type */
#define USBFS_DOEP3CTL_EPTYPE_MSK								(0x03UL << USBFS_DOEP3CTL_EPTYPE_POS)		/** Endpoint type */
#define USBFS_DOEP3CTL_NAKS_POS								0x11UL		/** NAK status */
#define USBFS_DOEP3CTL_NAKS_MSK								(0x01UL << USBFS_DOEP3CTL_NAKS_POS)		/** NAK status */
#define USBFS_DOEP3CTL_EOFRM_DPID_POS								0x10UL		/** EOFRM/DPID */
#define USBFS_DOEP3CTL_EOFRM_DPID_MSK								(0x01UL << USBFS_DOEP3CTL_EOFRM_DPID_POS)		/** EOFRM/DPID */
#define USBFS_DOEP3CTL_EPACT_POS								0x0FUL		/** Endpoint active */
#define USBFS_DOEP3CTL_EPACT_MSK								(0x01UL << USBFS_DOEP3CTL_EPACT_POS)		/** Endpoint active */
#define USBFS_DOEP3CTL_MPL_POS								0x00UL		/** maximum packet length */
#define USBFS_DOEP3CTL_MPL_MSK								(0x7FFUL << USBFS_DOEP3CTL_MPL_POS)		/** maximum packet length */
#define USBFS_DIEP0INTF_TXFE_POS								0x07UL		/** Transmit FIFO empty */
#define USBFS_DIEP0INTF_TXFE_MSK								(0x01UL << USBFS_DIEP0INTF_TXFE_POS)		/** Transmit FIFO empty */
#define USBFS_DIEP0INTF_IEPNE_POS								0x06UL		/** IN endpoint NAK effective */
#define USBFS_DIEP0INTF_IEPNE_MSK								(0x01UL << USBFS_DIEP0INTF_IEPNE_POS)		/** IN endpoint NAK effective */
#define USBFS_DIEP0INTF_EPTXFUD_POS								0x04UL		/** Endpoint Tx FIFO underrun */
#define USBFS_DIEP0INTF_EPTXFUD_MSK								(0x01UL << USBFS_DIEP0INTF_EPTXFUD_POS)		/** Endpoint Tx FIFO underrun */
#define USBFS_DIEP0INTF_CITO_POS								0x03UL		/** Control in timeout interrupt */
#define USBFS_DIEP0INTF_CITO_MSK								(0x01UL << USBFS_DIEP0INTF_CITO_POS)		/** Control in timeout interrupt */
#define USBFS_DIEP0INTF_EPDIS_POS								0x01UL		/** Endpoint finished */
#define USBFS_DIEP0INTF_EPDIS_MSK								(0x01UL << USBFS_DIEP0INTF_EPDIS_POS)		/** Endpoint finished */
#define USBFS_DIEP0INTF_TF_POS								0x00UL		/** Transfer finished */
#define USBFS_DIEP0INTF_TF_MSK								(0x01UL << USBFS_DIEP0INTF_TF_POS)		/** Transfer finished */
#define USBFS_DIEP1INTF_TXFE_POS								0x07UL		/** Transmit FIFO empty */
#define USBFS_DIEP1INTF_TXFE_MSK								(0x01UL << USBFS_DIEP1INTF_TXFE_POS)		/** Transmit FIFO empty */
#define USBFS_DIEP1INTF_IEPNE_POS								0x06UL		/** IN endpoint NAK effective */
#define USBFS_DIEP1INTF_IEPNE_MSK								(0x01UL << USBFS_DIEP1INTF_IEPNE_POS)		/** IN endpoint NAK effective */
#define USBFS_DIEP1INTF_EPTXFUD_POS								0x04UL		/** Endpoint Tx FIFO underrun */
#define USBFS_DIEP1INTF_EPTXFUD_MSK								(0x01UL << USBFS_DIEP1INTF_EPTXFUD_POS)		/** Endpoint Tx FIFO underrun */
#define USBFS_DIEP1INTF_CITO_POS								0x03UL		/** Control in timeout interrupt */
#define USBFS_DIEP1INTF_CITO_MSK								(0x01UL << USBFS_DIEP1INTF_CITO_POS)		/** Control in timeout interrupt */
#define USBFS_DIEP1INTF_EPDIS_POS								0x01UL		/** Endpoint finished */
#define USBFS_DIEP1INTF_EPDIS_MSK								(0x01UL << USBFS_DIEP1INTF_EPDIS_POS)		/** Endpoint finished */
#define USBFS_DIEP1INTF_TF_POS								0x00UL		/** Transfer finished */
#define USBFS_DIEP1INTF_TF_MSK								(0x01UL << USBFS_DIEP1INTF_TF_POS)		/** Transfer finished */
#define USBFS_DIEP2INTF_TXFE_POS								0x07UL		/** Transmit FIFO empty */
#define USBFS_DIEP2INTF_TXFE_MSK								(0x01UL << USBFS_DIEP2INTF_TXFE_POS)		/** Transmit FIFO empty */
#define USBFS_DIEP2INTF_IEPNE_POS								0x06UL		/** IN endpoint NAK effective */
#define USBFS_DIEP2INTF_IEPNE_MSK								(0x01UL << USBFS_DIEP2INTF_IEPNE_POS)		/** IN endpoint NAK effective */
#define USBFS_DIEP2INTF_EPTXFUD_POS								0x04UL		/** Endpoint Tx FIFO underrun */
#define USBFS_DIEP2INTF_EPTXFUD_MSK								(0x01UL << USBFS_DIEP2INTF_EPTXFUD_POS)		/** Endpoint Tx FIFO underrun */
#define USBFS_DIEP2INTF_CITO_POS								0x03UL		/** Control in timeout interrupt */
#define USBFS_DIEP2INTF_CITO_MSK								(0x01UL << USBFS_DIEP2INTF_CITO_POS)		/** Control in timeout interrupt */
#define USBFS_DIEP2INTF_EPDIS_POS								0x01UL		/** Endpoint finished */
#define USBFS_DIEP2INTF_EPDIS_MSK								(0x01UL << USBFS_DIEP2INTF_EPDIS_POS)		/** Endpoint finished */
#define USBFS_DIEP2INTF_TF_POS								0x00UL		/** Transfer finished */
#define USBFS_DIEP2INTF_TF_MSK								(0x01UL << USBFS_DIEP2INTF_TF_POS)		/** Transfer finished */
#define USBFS_DIEP3INTF_TXFE_POS								0x07UL		/** Transmit FIFO empty */
#define USBFS_DIEP3INTF_TXFE_MSK								(0x01UL << USBFS_DIEP3INTF_TXFE_POS)		/** Transmit FIFO empty */
#define USBFS_DIEP3INTF_IEPNE_POS								0x06UL		/** IN endpoint NAK effective */
#define USBFS_DIEP3INTF_IEPNE_MSK								(0x01UL << USBFS_DIEP3INTF_IEPNE_POS)		/** IN endpoint NAK effective */
#define USBFS_DIEP3INTF_EPTXFUD_POS								0x04UL		/** Endpoint Tx FIFO underrun */
#define USBFS_DIEP3INTF_EPTXFUD_MSK								(0x01UL << USBFS_DIEP3INTF_EPTXFUD_POS)		/** Endpoint Tx FIFO underrun */
#define USBFS_DIEP3INTF_CITO_POS								0x03UL		/** Control in timeout interrupt */
#define USBFS_DIEP3INTF_CITO_MSK								(0x01UL << USBFS_DIEP3INTF_CITO_POS)		/** Control in timeout interrupt */
#define USBFS_DIEP3INTF_EPDIS_POS								0x01UL		/** Endpoint finished */
#define USBFS_DIEP3INTF_EPDIS_MSK								(0x01UL << USBFS_DIEP3INTF_EPDIS_POS)		/** Endpoint finished */
#define USBFS_DIEP3INTF_TF_POS								0x00UL		/** Transfer finished */
#define USBFS_DIEP3INTF_TF_MSK								(0x01UL << USBFS_DIEP3INTF_TF_POS)		/** Transfer finished */
#define USBFS_DOEP0INTF_BTBSTP_POS								0x06UL		/** Back-to-back SETUP packets */
#define USBFS_DOEP0INTF_BTBSTP_MSK								(0x01UL << USBFS_DOEP0INTF_BTBSTP_POS)		/** Back-to-back SETUP packets */
#define USBFS_DOEP0INTF_EPRXFOVR_POS								0x04UL		/** Endpoint Rx FIFO overrun */
#define USBFS_DOEP0INTF_EPRXFOVR_MSK								(0x01UL << USBFS_DOEP0INTF_EPRXFOVR_POS)		/** Endpoint Rx FIFO overrun */
#define USBFS_DOEP0INTF_STPF_POS								0x03UL		/** Setup phase finished */
#define USBFS_DOEP0INTF_STPF_MSK								(0x01UL << USBFS_DOEP0INTF_STPF_POS)		/** Setup phase finished */
#define USBFS_DOEP0INTF_EPDIS_POS								0x01UL		/** Endpoint disabled */
#define USBFS_DOEP0INTF_EPDIS_MSK								(0x01UL << USBFS_DOEP0INTF_EPDIS_POS)		/** Endpoint disabled */
#define USBFS_DOEP0INTF_TF_POS								0x00UL		/** Transfer finished */
#define USBFS_DOEP0INTF_TF_MSK								(0x01UL << USBFS_DOEP0INTF_TF_POS)		/** Transfer finished */
#define USBFS_DOEP1INTF_BTBSTP_POS								0x06UL		/** Back-to-back SETUP packets */
#define USBFS_DOEP1INTF_BTBSTP_MSK								(0x01UL << USBFS_DOEP1INTF_BTBSTP_POS)		/** Back-to-back SETUP packets */
#define USBFS_DOEP1INTF_EPRXFOVR_POS								0x04UL		/** Endpoint Rx FIFO overrun */
#define USBFS_DOEP1INTF_EPRXFOVR_MSK								(0x01UL << USBFS_DOEP1INTF_EPRXFOVR_POS)		/** Endpoint Rx FIFO overrun */
#define USBFS_DOEP1INTF_STPF_POS								0x03UL		/** Setup phase finished */
#define USBFS_DOEP1INTF_STPF_MSK								(0x01UL << USBFS_DOEP1INTF_STPF_POS)		/** Setup phase finished */
#define USBFS_DOEP1INTF_EPDIS_POS								0x01UL		/** Endpoint disabled */
#define USBFS_DOEP1INTF_EPDIS_MSK								(0x01UL << USBFS_DOEP1INTF_EPDIS_POS)		/** Endpoint disabled */
#define USBFS_DOEP1INTF_TF_POS								0x00UL		/** Transfer finished */
#define USBFS_DOEP1INTF_TF_MSK								(0x01UL << USBFS_DOEP1INTF_TF_POS)		/** Transfer finished */
#define USBFS_DOEP2INTF_BTBSTP_POS								0x06UL		/** Back-to-back SETUP packets */
#define USBFS_DOEP2INTF_BTBSTP_MSK								(0x01UL << USBFS_DOEP2INTF_BTBSTP_POS)		/** Back-to-back SETUP packets */
#define USBFS_DOEP2INTF_EPRXFOVR_POS								0x04UL		/** Endpoint Rx FIFO overrun */
#define USBFS_DOEP2INTF_EPRXFOVR_MSK								(0x01UL << USBFS_DOEP2INTF_EPRXFOVR_POS)		/** Endpoint Rx FIFO overrun */
#define USBFS_DOEP2INTF_STPF_POS								0x03UL		/** Setup phase finished */
#define USBFS_DOEP2INTF_STPF_MSK								(0x01UL << USBFS_DOEP2INTF_STPF_POS)		/** Setup phase finished */
#define USBFS_DOEP2INTF_EPDIS_POS								0x01UL		/** Endpoint disabled */
#define USBFS_DOEP2INTF_EPDIS_MSK								(0x01UL << USBFS_DOEP2INTF_EPDIS_POS)		/** Endpoint disabled */
#define USBFS_DOEP2INTF_TF_POS								0x00UL		/** Transfer finished */
#define USBFS_DOEP2INTF_TF_MSK								(0x01UL << USBFS_DOEP2INTF_TF_POS)		/** Transfer finished */
#define USBFS_DOEP3INTF_BTBSTP_POS								0x06UL		/** Back-to-back SETUP packets */
#define USBFS_DOEP3INTF_BTBSTP_MSK								(0x01UL << USBFS_DOEP3INTF_BTBSTP_POS)		/** Back-to-back SETUP packets */
#define USBFS_DOEP3INTF_EPRXFOVR_POS								0x04UL		/** Endpoint Rx FIFO overrun */
#define USBFS_DOEP3INTF_EPRXFOVR_MSK								(0x01UL << USBFS_DOEP3INTF_EPRXFOVR_POS)		/** Endpoint Rx FIFO overrun */
#define USBFS_DOEP3INTF_STPF_POS								0x03UL		/** Setup phase finished */
#define USBFS_DOEP3INTF_STPF_MSK								(0x01UL << USBFS_DOEP3INTF_STPF_POS)		/** Setup phase finished */
#define USBFS_DOEP3INTF_EPDIS_POS								0x01UL		/** Endpoint disabled */
#define USBFS_DOEP3INTF_EPDIS_MSK								(0x01UL << USBFS_DOEP3INTF_EPDIS_POS)		/** Endpoint disabled */
#define USBFS_DOEP3INTF_TF_POS								0x00UL		/** Transfer finished */
#define USBFS_DOEP3INTF_TF_MSK								(0x01UL << USBFS_DOEP3INTF_TF_POS)		/** Transfer finished */
#define USBFS_DIEP0LEN_PCNT_POS								0x13UL		/** Packet count */
#define USBFS_DIEP0LEN_PCNT_MSK								(0x03UL << USBFS_DIEP0LEN_PCNT_POS)		/** Packet count */
#define USBFS_DIEP0LEN_TLEN_POS								0x00UL		/** Transfer length */
#define USBFS_DIEP0LEN_TLEN_MSK								(0x7FUL << USBFS_DIEP0LEN_TLEN_POS)		/** Transfer length */
#define USBFS_DOEP0LEN_STPCNT_POS								0x1DUL		/** SETUP packet count */
#define USBFS_DOEP0LEN_STPCNT_MSK								(0x03UL << USBFS_DOEP0LEN_STPCNT_POS)		/** SETUP packet count */
#define USBFS_DOEP0LEN_PCNT_POS								0x13UL		/** Packet count */
#define USBFS_DOEP0LEN_PCNT_MSK								(0x01UL << USBFS_DOEP0LEN_PCNT_POS)		/** Packet count */
#define USBFS_DOEP0LEN_TLEN_POS								0x00UL		/** Transfer length */
#define USBFS_DOEP0LEN_TLEN_MSK								(0x7FUL << USBFS_DOEP0LEN_TLEN_POS)		/** Transfer length */
#define USBFS_DIEP1LEN_MCPF_POS								0x1DUL		/** Multi packet count per frame */
#define USBFS_DIEP1LEN_MCPF_MSK								(0x03UL << USBFS_DIEP1LEN_MCPF_POS)		/** Multi packet count per frame */
#define USBFS_DIEP1LEN_PCNT_POS								0x13UL		/** Packet count */
#define USBFS_DIEP1LEN_PCNT_MSK								(0x3FFUL << USBFS_DIEP1LEN_PCNT_POS)		/** Packet count */
#define USBFS_DIEP1LEN_TLEN_POS								0x00UL		/** Transfer length */
#define USBFS_DIEP1LEN_TLEN_MSK								(0x7FFFFUL << USBFS_DIEP1LEN_TLEN_POS)		/** Transfer length */
#define USBFS_DIEP2LEN_MCPF_POS								0x1DUL		/** Multi packet count per frame */
#define USBFS_DIEP2LEN_MCPF_MSK								(0x03UL << USBFS_DIEP2LEN_MCPF_POS)		/** Multi packet count per frame */
#define USBFS_DIEP2LEN_PCNT_POS								0x13UL		/** Packet count */
#define USBFS_DIEP2LEN_PCNT_MSK								(0x3FFUL << USBFS_DIEP2LEN_PCNT_POS)		/** Packet count */
#define USBFS_DIEP2LEN_TLEN_POS								0x00UL		/** Transfer length */
#define USBFS_DIEP2LEN_TLEN_MSK								(0x7FFFFUL << USBFS_DIEP2LEN_TLEN_POS)		/** Transfer length */
#define USBFS_DIEP3LEN_MCPF_POS								0x1DUL		/** Multi packet count per frame */
#define USBFS_DIEP3LEN_MCPF_MSK								(0x03UL << USBFS_DIEP3LEN_MCPF_POS)		/** Multi packet count per frame */
#define USBFS_DIEP3LEN_PCNT_POS								0x13UL		/** Packet count */
#define USBFS_DIEP3LEN_PCNT_MSK								(0x3FFUL << USBFS_DIEP3LEN_PCNT_POS)		/** Packet count */
#define USBFS_DIEP3LEN_TLEN_POS								0x00UL		/** Transfer length */
#define USBFS_DIEP3LEN_TLEN_MSK								(0x7FFFFUL << USBFS_DIEP3LEN_TLEN_POS)		/** Transfer length */
#define USBFS_DOEP1LEN_STPCNT_RXDPID_POS								0x1DUL		/** SETUP packet count/Received data PID */
#define USBFS_DOEP1LEN_STPCNT_RXDPID_MSK								(0x03UL << USBFS_DOEP1LEN_STPCNT_RXDPID_POS)		/** SETUP packet count/Received data PID */
#define USBFS_DOEP1LEN_PCNT_POS								0x13UL		/** Packet count */
#define USBFS_DOEP1LEN_PCNT_MSK								(0x3FFUL << USBFS_DOEP1LEN_PCNT_POS)		/** Packet count */
#define USBFS_DOEP1LEN_TLEN_POS								0x00UL		/** Transfer length */
#define USBFS_DOEP1LEN_TLEN_MSK								(0x7FFFFUL << USBFS_DOEP1LEN_TLEN_POS)		/** Transfer length */
#define USBFS_DOEP2LEN_STPCNT_RXDPID_POS								0x1DUL		/** SETUP packet count/Received data PID */
#define USBFS_DOEP2LEN_STPCNT_RXDPID_MSK								(0x03UL << USBFS_DOEP2LEN_STPCNT_RXDPID_POS)		/** SETUP packet count/Received data PID */
#define USBFS_DOEP2LEN_PCNT_POS								0x13UL		/** Packet count */
#define USBFS_DOEP2LEN_PCNT_MSK								(0x3FFUL << USBFS_DOEP2LEN_PCNT_POS)		/** Packet count */
#define USBFS_DOEP2LEN_TLEN_POS								0x00UL		/** Transfer length */
#define USBFS_DOEP2LEN_TLEN_MSK								(0x7FFFFUL << USBFS_DOEP2LEN_TLEN_POS)		/** Transfer length */
#define USBFS_DOEP3LEN_STPCNT_RXDPID_POS								0x1DUL		/** SETUP packet count/Received data PID */
#define USBFS_DOEP3LEN_STPCNT_RXDPID_MSK								(0x03UL << USBFS_DOEP3LEN_STPCNT_RXDPID_POS)		/** SETUP packet count/Received data PID */
#define USBFS_DOEP3LEN_PCNT_POS								0x13UL		/** Packet count */
#define USBFS_DOEP3LEN_PCNT_MSK								(0x3FFUL << USBFS_DOEP3LEN_PCNT_POS)		/** Packet count */
#define USBFS_DOEP3LEN_TLEN_POS								0x00UL		/** Transfer length */
#define USBFS_DOEP3LEN_TLEN_MSK								(0x7FFFFUL << USBFS_DOEP3LEN_TLEN_POS)		/** Transfer length */
#define USBFS_DIEP0TFSTAT_IEPTFS_POS								0x00UL		/** IN endpoint TxFIFO space remaining */
#define USBFS_DIEP0TFSTAT_IEPTFS_MSK								(0xFFFFUL << USBFS_DIEP0TFSTAT_IEPTFS_POS)		/** IN endpoint TxFIFO space remaining */
#define USBFS_DIEP1TFSTAT_IEPTFS_POS								0x00UL		/** IN endpoint TxFIFO space remaining */
#define USBFS_DIEP1TFSTAT_IEPTFS_MSK								(0xFFFFUL << USBFS_DIEP1TFSTAT_IEPTFS_POS)		/** IN endpoint TxFIFO space remaining */
#define USBFS_DIEP2TFSTAT_IEPTFS_POS								0x00UL		/** IN endpoint TxFIFO space remaining */
#define USBFS_DIEP2TFSTAT_IEPTFS_MSK								(0xFFFFUL << USBFS_DIEP2TFSTAT_IEPTFS_POS)		/** IN endpoint TxFIFO space remaining */
#define USBFS_DIEP3TFSTAT_IEPTFS_POS								0x00UL		/** IN endpoint TxFIFO space remaining */
#define USBFS_DIEP3TFSTAT_IEPTFS_MSK								(0xFFFFUL << USBFS_DIEP3TFSTAT_IEPTFS_POS)		/** IN endpoint TxFIFO space remaining */
#define USBFS_PWRCLKCTL_SUCLK_POS								0x00UL		/** Stop the USB clock */
#define USBFS_PWRCLKCTL_SUCLK_MSK								(0x01UL << USBFS_PWRCLKCTL_SUCLK_POS)		/** Stop the USB clock */
#define USBFS_PWRCLKCTL_SHCLK_POS								0x01UL		/** Stop HCLK */
#define USBFS_PWRCLKCTL_SHCLK_MSK								(0x01UL << USBFS_PWRCLKCTL_SHCLK_POS)		/** Stop HCLK */
#define WWDGT_CTL_WDGTEN_POS								0x07UL		/** Activation bit */
#define WWDGT_CTL_WDGTEN_MSK								(0x01UL << WWDGT_CTL_WDGTEN_POS)		/** Activation bit */
#define WWDGT_CTL_CNT_POS								0x00UL		/** 7-bit counter */
#define WWDGT_CTL_CNT_MSK								(0x7FUL << WWDGT_CTL_CNT_POS)		/** 7-bit counter */
#define WWDGT_CFG_EWIE_POS								0x09UL		/** Early wakeup interrupt */
#define WWDGT_CFG_EWIE_MSK								(0x01UL << WWDGT_CFG_EWIE_POS)		/** Early wakeup interrupt */
#define WWDGT_CFG_PSC_POS								0x07UL		/** Prescaler */
#define WWDGT_CFG_PSC_MSK								(0x03UL << WWDGT_CFG_PSC_POS)		/** Prescaler */
#define WWDGT_CFG_WIN_POS								0x00UL		/** 7-bit window value */
#define WWDGT_CFG_WIN_MSK								(0x7FUL << WWDGT_CFG_WIN_POS)		/** 7-bit window value */
#define WWDGT_STAT_EWIF_POS								0x00UL		/** Early wakeup interrupt flag */
#define WWDGT_STAT_EWIF_MSK								(0x01UL << WWDGT_STAT_EWIF_POS)		/** Early wakeup interrupt flag */


#ifdef __cplusplus
}
#endif

#endif // __GD32VF103XB_H
