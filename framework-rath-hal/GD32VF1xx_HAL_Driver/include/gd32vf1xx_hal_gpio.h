/* include guard */
#ifndef __GD32VF1XX_HAL_GPIO_H
#define __GD32VF1XX_HAL_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gd32vf1xx_hal_def.h"



/**
 * @brief GPIO init structure definition
 */
typedef struct {
  /** Specifies the GPIO pins to be configured. 
   * Values can be GPIO_PIN_x (x = 0..15) */
  uint16_t pin;
  /** Specifies the operating mode for the selected pins. 
   * Values can be
   * - GPIO_MODE_INPUT_ANALOG
   * - GPIO_MODE_INPUT
   * - GPIO_MODE_OUTPUT_PUSHPULL
   * - GPIO_MODE_OUTPUT_OPENDRAIN
   * - GPIO_MODE_AF_PUSHPULL
   * - GPIO_MODE_AF_OPENDRAIN
   */
  uint8_t mode;
  /** Specifies the Pull-up or Pull-Down activation for the selected pins. Only needed for INPUT modes.
   * Values can be
   * - GPIO_PULL_DOWN
   * - GPIO_PULL_UP
   * - GPIO_PULL_NONE
   */
  uint8_t pull;
  /** Specifies the speed for the selected pins. Only needed for OUTPUT modes.
   * Values can be
   * - GPIO_SPEED_10MHZ
   * - GPIO_SPEED_20MHZ
   * - GPIO_SPEED_50MHZ
   */
  uint8_t speed;
} GPIO_InitTypeDef;


#define GPIO_PIN_0                 ((uint16_t)0x0001)  /** Pin 0 selected    */
#define GPIO_PIN_1                 ((uint16_t)0x0002)  /** Pin 1 selected    */
#define GPIO_PIN_2                 ((uint16_t)0x0004)  /** Pin 2 selected    */
#define GPIO_PIN_3                 ((uint16_t)0x0008)  /** Pin 3 selected    */
#define GPIO_PIN_4                 ((uint16_t)0x0010)  /** Pin 4 selected    */
#define GPIO_PIN_5                 ((uint16_t)0x0020)  /** Pin 5 selected    */
#define GPIO_PIN_6                 ((uint16_t)0x0040)  /** Pin 6 selected    */
#define GPIO_PIN_7                 ((uint16_t)0x0080)  /** Pin 7 selected    */
#define GPIO_PIN_8                 ((uint16_t)0x0100)  /** Pin 8 selected    */
#define GPIO_PIN_9                 ((uint16_t)0x0200)  /** Pin 9 selected    */
#define GPIO_PIN_10                ((uint16_t)0x0400)  /** Pin 10 selected   */
#define GPIO_PIN_11                ((uint16_t)0x0800)  /** Pin 11 selected   */
#define GPIO_PIN_12                ((uint16_t)0x1000)  /** Pin 12 selected   */
#define GPIO_PIN_13                ((uint16_t)0x2000)  /** Pin 13 selected   */
#define GPIO_PIN_14                ((uint16_t)0x4000)  /** Pin 14 selected   */
#define GPIO_PIN_15                ((uint16_t)0x8000)  /** Pin 15 selected   */
#define GPIO_PIN_All               ((uint16_t)0xFFFF)  /** all pins selected */

                                     // (CTLx << 2 | MDx)
#define GPIO_MODE_INPUT_ANALOG          (0b00UL << 2UL | 0b00UL)    /** Analog Input or Alternate Function Analog Input mode */
#define GPIO_MODE_INPUT                 (0b10UL << 2UL | 0b00UL)    /** Digital Input or Alternate Function Digital Input mode */
#define GPIO_MODE_OUTPUT_PP             (0b00UL << 2UL | 0b11UL)    /** Output Push-Pull mode */
#define GPIO_MODE_OUTPUT_OD             (0b01UL << 2UL | 0b11UL)    /** Output Open-Drain mode */
#define GPIO_MODE_AF_PP           (0b10UL << 2UL | 0b11UL)    /** Alternate Function Push-Pull mode */
#define GPIO_MODE_AF_OD          (0b11UL << 2UL | 0b11UL)    /** Alternate Function Open-Drain mode */

#define GPIO_PULL_DOWN                  0b00UL    /** enable builtin Pull-Down resistor, input modes only */
#define GPIO_PULL_UP                    0b01UL    /** enable builtin Pull-Up resistor, input modes only */
#define GPIO_PULL_NONE                  0b11UL    /** disable builtin Pull-Down and Pull-Up resistors */

#define GPIO_SPEED_10MHZ                0b01UL    /** maximum output speed 10MHz */
#define GPIO_SPEED_20MHZ                0b10UL    /** maximum output speed 20MHz */
#define GPIO_SPEED_50MHZ                0b11UL    /** maximum output speed 50MHz */

#define GPIO_OUTPUT_PORT_OFFSET         0x04UL           /*!< GPIO event output port offset*/


#define LL_GPIO_setPin(GPIOx, pin)                SET_BITS(GPIOx->BOP, pin)
#define LL_GPIO_clearPin(GPIOx, pin)              SET_BITS(GPIOx->BC, pin)
#define LL_GPIO_readPin(GPIOx, pin)               (READ_BITS(GPIOx->ISTAT, pin) ? SET : RESET)
#define LL_GPIO_readOutputPin(GPIOx, pin)         (READ_BITS(GPIOx->OCTL, pin) ? SET : RESET)

#define LL_GPIO_enableEventOutput()               SET_BITS(AFIO->EC, AFIO_EC_EOE_MSK)
#define LL_GPIO_disableEventOutput()              CLEAR_BITS(AFIO->EC, AFIO_EC_EOE_MSK);

uint32_t LL_GPIO_getPortAddress(GPIO_TypeDef *GPIOx);


void HAL_GPIO_deinit(GPIO_TypeDef *GPIOx);


/**
 * @brief initialize GPIO pin(s)
 * 
 * @param GPIOx GPIO port, (x = A..E)
 * @param GPIO_init GPIO init struct
 */
void HAL_GPIO_init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_init);

/**
 * @brief initialize GPIO pin(s) in one line
 * 
 * @param GPIOx GPIO port, (x = A..E)
 * @param GPIO_pin the GPIO pins to be configured.
 * @param mode pin Mode
 * @param speed pin Speed, only valid for Output modes
 * @param pull pin Pull, only valid for Input modes
 * @see GPIO_InitTypeDef
 */
void HAL_GPIO_setup(GPIO_TypeDef *GPIOx, uint16_t GPIO_pin, uint32_t mode, uint32_t speed, uint32_t pull);

/**
 * @brief change the pin state of an output pin
 * 
 * @param GPIOx 
 * @param GPIO_pin 
 * @param state 
 */
void HAL_GPIO_writePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_pin, uint32_t state);

/**
 * @brief read the pin state of an input pin
 * 
 * @param GPIOx 
 * @param GPIO_pin 
 * @return GPIO_PinState 
 */
uint8_t HAL_GPIO_readPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_pin);

/**
 * @brief read the pin state of an output pin
 * 
 * @param GPIOx 
 * @param GPIO_pin 
 * @return GPIO_PinState 
 */
uint8_t HAL_GPIO_readOutputPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_pin);

void HAL_GPIO_configPinRemap(uint32_t remap, FunctionalState newvalue);


void HAL_GPIO_selectEXTISource(GPIO_TypeDef *GPIOx, uint8_t output_pin);

/*!
    \brief      configure GPIO pin event output
    \param[in]  output_port: gpio event output port
                only one parameter can be selected which are shown as below:
      \arg        GPIO_EVENT_PORT_GPIOA: event output port A
      \arg        GPIO_EVENT_PORT_GPIOB: event output port B
      \arg        GPIO_EVENT_PORT_GPIOC: event output port C
      \arg        GPIO_EVENT_PORT_GPIOD: event output port D
      \arg        GPIO_EVENT_PORT_GPIOE: event output port E
    \param[in]  output_pin:
                only one parameter can be selected which are shown as below:
      \arg        GPIO_EVENT_PIN_x(x=0..15)
    \param[out] none
    \retval     none
*/
void HAL_GPIO_configEventOutput(GPIO_TypeDef *GPIOx, uint8_t output_pin);

/**
 * @brief lock the pin configuration. the configuration cannot be changed until the next reset. 
 * 
 * @param GPIOx 
 * @param GPIO_pin 
 */
void HAL_GPIO_lockPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_pin);

#ifdef __cplusplus
}
#endif

#endif  // __GD32VF1XX_HAL_GPIO_H
