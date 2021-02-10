/**
 * @file gd32vf1xx_hal_gpio.c
 * @author -T.K.- (t_k_233@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2021-01-23
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "gd32vf1xx_hal_gpio.h"




void HAL_GPIO_deinit(GPIO_TypeDef *GPIOx) {
  // if (GPIOx == GPIOA) {
  //   /* reset GPIOA */
  //   rcu_periph_reset_enable(RCU_GPIOARST);
  //   rcu_periph_reset_disable(RCU_GPIOARST);
  // }
  // else if (GPIOx == GPIOB) {
  //   /* reset GPIOB */
  //   rcu_periph_reset_enable(RCU_GPIOBRST);
  //   rcu_periph_reset_disable(RCU_GPIOBRST);
  // }
  // else if (GPIOx == GPIOC) {
  //   /* reset GPIOC */
  //   rcu_periph_reset_enable(RCU_GPIOCRST);
  //   rcu_periph_reset_disable(RCU_GPIOCRST);
  // }
  // else if (GPIOx == GPIOD) {
  //   /* reset GPIOD */
  //   rcu_periph_reset_enable(RCU_GPIODRST);
  //   rcu_periph_reset_disable(RCU_GPIODRST);
  // }
  // else if (GPIOx == GPIOE) {
  //   /* reset GPIOE */
  //   rcu_periph_reset_enable(RCU_GPIOERST);
  //   rcu_periph_reset_disable(RCU_GPIOERST);
  // }
  // else if (GPIOx == AFIO) {
  //   rcu_periph_reset_enable(RCU_AFRST);
  //   rcu_periph_reset_disable(RCU_AFRST);
  // }
}


/**
 * @brief initialize GPIO pin(s)
 * 
 * @param GPIOx GPIO port, (x = A..E)
 * @param GPIO_init GPIO init struct
 */
void HAL_GPIO_init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_init) {
  uint32_t position = 0;
  uint32_t current_pin;

  /* iterate over all the pins */
  while (((GPIO_init->pin) >> position != 0x00UL)) {
    current_pin = READ_BITS(GPIO_init->pin, 0x1UL << position);

    /* if current pin is need to be initalized */
    if (current_pin) {
      uint32_t config = GPIO_init->mode;

      /* clearing OCTL, prepare for INPUT pull mode write or reset pin state for OUTPUT */
      CLEAR_BITS(GPIOx->OCTL, current_pin);

      /* if INPUT, then set the OCTL bit to corresponding pull state */
      if (GPIO_init->mode == GPIO_MODE_INPUT) {
        SET_BITS(GPIOx->OCTL, (GPIO_init->pull & 0x1UL) << current_pin);

        /* if no pull, change CTL[1:0] to 0b01 */
        if (GPIO_init->pull == GPIO_PULL_NONE) {
          config = 0b0100UL;
        }
      }
      /* if OUTPUT, set the speed */
      else if (GPIO_init->mode == GPIO_MODE_OUTPUT_PP || GPIO_init->mode == GPIO_MODE_OUTPUT_OD
        || GPIO_init->mode == GPIO_MODE_AF_PP || GPIO_init->mode == GPIO_MODE_AF_OD) {
        config |= GPIO_init->speed;
      }

      /* write the config settings in CTL register */
      /* write to lower 8 pins */
      if (position <= 7) {
        CLEAR_BITS(GPIOx->CTL0, (GPIO_CTL0_CTL0_MSK | GPIO_CTL0_MD0_MSK) << (position * 4));
        SET_BITS(GPIOx->CTL0, (config & (GPIO_CTL0_CTL0_MSK | GPIO_CTL0_MD0_MSK)) << (position * 4));
      }
      /* write to upper 8 pins */
      else {
        CLEAR_BITS(GPIOx->CTL1, (GPIO_CTL1_CTL8_MSK | GPIO_CTL1_MD8_MSK) << (position * 4));
        SET_BITS(GPIOx->CTL1, (config & (GPIO_CTL1_CTL8_MSK | GPIO_CTL1_MD8_MSK)) << (position * 4));
      }
    }
    position += 1;
  }
}

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
void HAL_GPIO_setup(GPIO_TypeDef *GPIOx, uint16_t GPIO_pin, uint32_t mode, uint32_t speed, uint32_t pull) {
  GPIO_InitTypeDef GPIO_init_struct;
  GPIO_init_struct.pin = GPIO_pin;
  GPIO_init_struct.mode = mode;
  GPIO_init_struct.speed = speed;
  GPIO_init_struct.speed = pull;
  HAL_GPIO_init(GPIOx, &GPIO_init_struct);
}

/**
 * @brief change the pin state of an output pin
 * 
 * @param GPIOx 
 * @param GPIO_pin 
 * @param state 
 */
void HAL_GPIO_writePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_pin, uint32_t state) {
  if (state) {
    LL_GPIO_setPin(GPIOx, GPIO_pin);
  }
  else {
    LL_GPIO_clearPin(GPIOx, GPIO_pin);
  }
}

/**
 * @brief read the pin state of an input pin
 * 
 * @param GPIOx 
 * @param GPIO_pin 
 * @return GPIO_PinState 
 */
uint8_t HAL_GPIO_readPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_pin) {
  return LL_GPIO_readPin(GPIOx, GPIO_pin);
}

/**
 * @brief read the pin state of an output pin
 * 
 * @param GPIOx 
 * @param GPIO_pin 
 * @return GPIO_PinState 
 */
uint8_t HAL_GPIO_readOutputPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_pin) {
  return LL_GPIO_readOutputPin(GPIOx, GPIO_pin);
}

void HAL_GPIO_configPinRemap(uint32_t remap, FunctionalState newvalue) {
  // uint32_t remap1 = 0U, remap2 = 0U, temp_reg = 0U, temp_mask = 0U;

//   if (AFIO_PCF1_FIELDS == (remap & AFIO_PCF1_FIELDS)) {
//     /* get AFIO_PCF1 regiter value */
//     temp_reg = AFIO_PCF1;
//   } else {
//     /* get AFIO_PCF0 regiter value */
//     temp_reg = AFIO_PCF0;
//   }

//   temp_mask = (remap & PCF_POSITION_MASK) >> 0x10U;
//   remap1 = remap & LSB_16BIT_MASK;

//   /* judge pin remap type */
//   if ((PCF_LOCATION1_MASK | PCF_LOCATION2_MASK)
//       == (remap & (PCF_LOCATION1_MASK | PCF_LOCATION2_MASK))) {
//     temp_reg &= PCF_SWJCFG_MASK;
//     AFIO_PCF0 &= PCF_SWJCFG_MASK;
//   } else if (PCF_LOCATION2_MASK == (remap & PCF_LOCATION2_MASK)) {
//     remap2 = ((uint32_t) 0x03U) << temp_mask;
//     temp_reg &= ~remap2;
//     temp_reg |= ~PCF_SWJCFG_MASK;
//   }  else {
//     temp_reg &= ~(remap1 << ((remap >> 0x15U) * 0x10U));
//     temp_reg |= ~PCF_SWJCFG_MASK;
//   }

//   /* set pin remap value */
//   if (DISABLE != newvalue) {
//     temp_reg |= (remap1 << ((remap >> 0x15U) * 0x10U));
//   }

//   if (AFIO_PCF1_FIELDS == (remap & AFIO_PCF1_FIELDS)) {
//     /* set AFIO_PCF1 regiter value */
//     AFIO->PCF1 = temp_reg;
//   } else {
//     /* set AFIO_PCF0 regiter value */
//     AFIO->PCF0 = temp_reg;
//   }
}

#define AFIO_EXTI_SOURCE_MASK              ((uint8_t)0x03U)         /*!< AFIO exti source selection mask*/  
#define AFIO_EXTI_SOURCE_FIELDS            ((uint8_t)0x04U)         /*!< select AFIO exti source registers */
#define LSB_16BIT_MASK                     ((uint16_t)0xFFFFU)      /*!< LSB 16-bit mask */
#define PCF_POSITION_MASK                  ((uint32_t)0x000F0000U)  /*!< AFIO_PCF register position mask */
#define PCF_SWJCFG_MASK                    ((uint32_t)0xF0FFFFFFU)  /*!< AFIO_PCF register SWJCFG mask */
#define PCF_LOCATION1_MASK                 ((uint32_t)0x00200000U)  /*!< AFIO_PCF register location1 mask */
#define PCF_LOCATION2_MASK                 ((uint32_t)0x00100000U)  /*!< AFIO_PCF register location2 mask */
#define AFIO_PCF1_FIELDS                   ((uint32_t)0x80000000U)  /*!< select AFIO_PCF1 register */
#define GPIO_OUTPUT_PORT_OFFSET            ((uint32_t)4U)           /*!< GPIO event output port offset*/


/* GPIO output pin source definitions */
#define GPIO_PIN_SOURCE_0                ((uint8_t)0x00U)          /*!< GPIO pin source 0 */
#define GPIO_PIN_SOURCE_1                ((uint8_t)0x01U)          /*!< GPIO pin source 1 */
#define GPIO_PIN_SOURCE_2                ((uint8_t)0x02U)          /*!< GPIO pin source 2 */
#define GPIO_PIN_SOURCE_3                ((uint8_t)0x03U)          /*!< GPIO pin source 3 */
#define GPIO_PIN_SOURCE_4                ((uint8_t)0x04U)          /*!< GPIO pin source 4 */
#define GPIO_PIN_SOURCE_5                ((uint8_t)0x05U)          /*!< GPIO pin source 5 */
#define GPIO_PIN_SOURCE_6                ((uint8_t)0x06U)          /*!< GPIO pin source 6 */
#define GPIO_PIN_SOURCE_7                ((uint8_t)0x07U)          /*!< GPIO pin source 7 */
#define GPIO_PIN_SOURCE_8                ((uint8_t)0x08U)          /*!< GPIO pin source 8 */
#define GPIO_PIN_SOURCE_9                ((uint8_t)0x09U)          /*!< GPIO pin source 9 */
#define GPIO_PIN_SOURCE_10               ((uint8_t)0x0AU)          /*!< GPIO pin source 10 */
#define GPIO_PIN_SOURCE_11               ((uint8_t)0x0BU)          /*!< GPIO pin source 11 */
#define GPIO_PIN_SOURCE_12               ((uint8_t)0x0CU)          /*!< GPIO pin source 12 */
#define GPIO_PIN_SOURCE_13               ((uint8_t)0x0DU)          /*!< GPIO pin source 13 */
#define GPIO_PIN_SOURCE_14               ((uint8_t)0x0EU)          /*!< GPIO pin source 14 */
#define GPIO_PIN_SOURCE_15               ((uint8_t)0x0FU)          /*!< GPIO pin source 15 */



void HAL_GPIO_selectEXTISource(GPIO_TypeDef *GPIOx, uint8_t output_pin) {
    uint32_t source = 0U;
    source = ((uint32_t) 0x0FU)
            << (AFIO_EXTI_SOURCE_FIELDS * (output_pin & AFIO_EXTI_SOURCE_MASK));

    /* select EXTI sources */
    if (GPIO_PIN_SOURCE_4 > output_pin) {
        /* select EXTI0/EXTI1/EXTI2/EXTI3 */
        AFIO->EXTISS0 &= ~source;
        AFIO->EXTISS0 |= (((uint32_t) LL_GPIO_getPortAddress(GPIOx))
                << (AFIO_EXTI_SOURCE_FIELDS
                        * (output_pin & AFIO_EXTI_SOURCE_MASK)));
    } else if (GPIO_PIN_SOURCE_8 > output_pin) {
        /* select EXTI4/EXTI5/EXTI6/EXTI7 */
        AFIO->EXTISS1 &= ~source;
        AFIO->EXTISS1 |= (((uint32_t) LL_GPIO_getPortAddress(GPIOx))
                << (AFIO_EXTI_SOURCE_FIELDS
                        * (output_pin & AFIO_EXTI_SOURCE_MASK)));
    } else if (GPIO_PIN_SOURCE_12 > output_pin) {
        /* select EXTI8/EXTI9/EXTI10/EXTI11 */
        AFIO->EXTISS2 &= ~source;
        AFIO->EXTISS2 |= (((uint32_t) LL_GPIO_getPortAddress(GPIOx))
                << (AFIO_EXTI_SOURCE_FIELDS
                        * (output_pin & AFIO_EXTI_SOURCE_MASK)));
    } else {
        /* select EXTI12/EXTI13/EXTI14/EXTI15 */
        AFIO->EXTISS3 &= ~source;
        AFIO->EXTISS3 |= (((uint32_t) LL_GPIO_getPortAddress(GPIOx))
                << (AFIO_EXTI_SOURCE_FIELDS
                        * (output_pin & AFIO_EXTI_SOURCE_MASK)));
    }
}

uint32_t LL_GPIO_getPortAddress(GPIO_TypeDef *GPIOx) {
  if (GPIOx == GPIOA) {
    return GPIOA_BASE;
  }
  if (GPIOx == GPIOB) {
    return GPIOB_BASE;
  }
  if (GPIOx == GPIOC) {
    return GPIOC_BASE;
  }
  if (GPIOx == GPIOD) {
    return GPIOD_BASE;
  }
  if (GPIOx == GPIOE) {
    return GPIOE_BASE;
  }
}

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
void HAL_GPIO_configEventOutput(GPIO_TypeDef *GPIOx, uint8_t output_pin) {
  /* clear AFIO_EC_PORT and AFIO_EC_PIN bits */
  CLEAR_BITS(AFIO->EC, AFIO_EC_PORT_MSK | AFIO_EC_PIN_MSK);

  SET_BITS(AFIO->EC, (LL_GPIO_getPortAddress(GPIOx) << GPIO_OUTPUT_PORT_OFFSET) | output_pin);
}


/**
 * @brief lock the pin configuration. the configuration cannot be changed until the next reset. 
 * 
 * @param GPIOx 
 * @param GPIO_pin 
 */
void HAL_GPIO_lockPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_pin) {
  uint32_t lock = 0x00010000UL;
  lock |= GPIO_pin;

  /* lock key writing sequence: write 1 -> write 0 -> write 1 -> read 0 -> read 1 */
  GPIOx->LOCK = (uint32_t) lock;
  GPIOx->LOCK = (uint32_t) GPIO_pin;
  GPIOx->LOCK = (uint32_t) lock;
  lock = GPIOx->LOCK;
  lock = GPIOx->LOCK;
}
