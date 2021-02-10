

#include "gd32vf1xx_hal_rcu.h"



void HAL_RCU_enablePeriphClock(RCU_Periph periph) {
  SET_BITS(*(uint32_t *)(RCU_BASE + (periph >> FLAG_OFFSET)), 1UL << (periph & FLAG_MASK));
}


void HAL_RCU_initDefaultSystemClock() {
  /* Configure the System clock source, PLL Multiplier, AHB/APBx prescalers and Flash settings */

  /* the code below will try to use HXTAL with PLL to achieve 108 MHz system clock. if HXTAL fails to initialize,
   * it will fall back to using IRC8M with PLL to achieve 108 MHz system clock. if IRC8M also fails to
   * initialize, it will bring the program to a dead loop. */
  uint32_t timeout = 0;

  /* enable HXTAL */
  SET_BITS(RCU->CTL, 0b1UL << RCU_CTL_HXTALEN_POS);

  /* wait HXTAL to stabilize */
  while (!READ_BITS(RCU->CTL, RCU_CTL_HXTALSTB_MSK)) {
    if (timeout >= HXTAL_STARTUP_TIMEOUT) {
      /* HXTAL is not available, fall back to IRC8M */

      /* enable IRC8M */
      SET_BITS(RCU->CTL, 0b1UL << RCU_CTL_IRC8MEN_POS);

      /* wait IRC8M to stabilize */
      while (!READ_BITS(RCU->CTL, RCU_CTL_IRC8MSTB_MSK)) {
        if (timeout >= IRC8M_STARTUP_TIMEOUT) {
          while (1) {
            /* all clock initialization failed. dead loop. */
          }
        }
        timeout += 1;
      }

      /* select PLLMF source as IRC8M */
      CLEAR_BITS(RCU->CFG0, RCU_CFG0_PLLSEL_MSK);

      /* set PLLMF factor to 27 */
      CLEAR_BITS(RCU->CFG0, RCU_CFG0_PLLMF_4_MSK | RCU_CFG0_PLLMF_3_0_MSK);
      SET_BITS(RCU->CFG0, (0b1UL << RCU_CFG0_PLLMF_4_POS) | (0b1010UL << RCU_CFG0_PLLMF_3_0_POS));

      /* enable PLL */
      SET_BITS(RCU->CTL, 0b1UL << RCU_CTL_PLLEN_POS);

      /* wait until PLL is stable */
      while (!READ_BITS(RCU->CTL, RCU_CTL_PLLSTB_MSK)) {}

      /* set CK_SYS source as CK_PLL */
      SET_BITS(RCU->CFG0, 0b10UL << RCU_CFG0_SCS_POS);

      /* wait CK_SYS source to be changed to PLL */
      while (READ_BITS(RCU->CFG0, RCU_CFG0_SCSS_MSK) != 0b10UL << RCU_CFG0_SCSS_POS) {}

      /* set bus clocks */
      CLEAR_BITS(RCU->CFG0, RCU_CFG0_APB2PSC_MSK | RCU_CFG0_APB1PSC_MSK | RCU_CFG0_AHBPSC_MSK);
      /* AHB = SYSCLK */
      SET_BITS(RCU->CFG0, 0b0000UL << RCU_CFG0_AHBPSC_POS);
      /* APB1 = AHB/2 */
      SET_BITS(RCU->CFG0, 0b100UL << RCU_CFG0_APB1PSC_POS);
      /* APB2 = AHB/1 */
      SET_BITS(RCU->CFG0, 0b000UL << RCU_CFG0_APB2PSC_POS);

      return ;
    }
    timeout += 1;
  }

  /* set PREDV1 factor to 8 */
  CLEAR_BITS(RCU->CFG1, RCU_CFG1_PREDV1_MSK);
  SET_BITS(RCU->CFG1, 0b0111UL << RCU_CFG1_PREDV1_POS);

  /* set PLLMF1 & PLLMF2 factor to 8 */
  CLEAR_BITS(RCU->CFG1, RCU_CFG1_PLL1MF_MSK | RCU_CFG1_PLL2MF_MSK);
  SET_BITS(RCU->CFG1, 0b0110UL << RCU_CFG1_PLL1MF_POS);
  SET_BITS(RCU->CFG1, 0b0110UL << RCU_CFG1_PLL2MF_POS);

  /* enable PLL1 & PLL2 */
  SET_BITS(RCU->CTL, 0b1UL << RCU_CTL_PLL1EN_POS);
  SET_BITS(RCU->CTL, 0b1UL << RCU_CTL_PLL2EN_POS);

  /* wait until PLL1 & PLL2 is stable */
  while (!READ_BITS(RCU->CTL, RCU_CTL_PLL1STB_MSK)) {}
  while (!READ_BITS(RCU->CTL, RCU_CTL_PLL2STB_MSK)) {}

  /* set PREDV0 source as PLLMF1 */
  SET_BITS(RCU->CFG1, 0b1UL << RCU_CFG1_PREDV0SEL_POS);

  /* set PREDV0 factor to 2 */
  CLEAR_BITS(RCU->CFG1, RCU_CFG1_PREDV0_MSK);
  SET_BITS(RCU->CFG1, 0b0001UL << RCU_CFG1_PREDV0_POS);

  /* select PLLMF source as PREDV0 */
  SET_BITS(RCU->CFG0, 0b1UL << RCU_CFG0_PLLSEL_POS);

  /* set PLLMF factor to 27 */
  CLEAR_BITS(RCU->CFG0, RCU_CFG0_PLLMF_4_MSK | RCU_CFG0_PLLMF_3_0_MSK);
  SET_BITS(RCU->CFG0, (0b1UL << RCU_CFG0_PLLMF_4_POS) | (0b1010UL << RCU_CFG0_PLLMF_3_0_POS));

  /* enable PLL */
  SET_BITS(RCU->CTL, 0b1UL << RCU_CTL_PLLEN_POS);

  /* wait until PLL is stable */
  while (!READ_BITS(RCU->CTL, RCU_CTL_PLLSTB_MSK)) {}

  /* set CK_SYS source as CK_PLL */
  SET_BITS(RCU->CFG0, 0b10UL << RCU_CFG0_SCS_POS);

  /* wait CK_SYS source to be changed to PLL */
  while (READ_BITS(RCU->CFG0, RCU_CFG0_SCSS_MSK) != 0b10UL << RCU_CFG0_SCSS_POS) {}

  /* set bus clocks */
  CLEAR_BITS(RCU->CFG0, RCU_CFG0_APB2PSC_MSK | RCU_CFG0_APB1PSC_MSK | RCU_CFG0_AHBPSC_MSK);
  /* AHB = SYSCLK */
  SET_BITS(RCU->CFG0, 0b0000UL << RCU_CFG0_AHBPSC_POS);
  /* APB1 = AHB/2 */
  SET_BITS(RCU->CFG0, 0b100UL << RCU_CFG0_APB1PSC_POS);
  /* APB2 = AHB/1 */
  SET_BITS(RCU->CFG0, 0b000UL << RCU_CFG0_APB2PSC_POS);

  return ;
}

uint32_t HAL_RCU_getSysClockFreq() {
  /* get CK_SYS selection status */
  uint32_t scss = READ_BITS(RCU->CFG0, RCU_CFG0_SCSS_MSK);

  if (scss == RCU_USE_IRC8M) {
    return IRC8M_VALUE;
  }
  if (scss == RCU_USE_HXTAL) {
    return HXTAL_VALUE;
  }

  /* else PLL will be used, need to calculate factor */
  uint32_t pllmf_4 = READ_BITS(RCU->CFG0, RCU_CFG0_PLLMF_4_MSK) >> RCU_CFG0_PLLMF_4_POS;
  uint32_t pllmf_3_0 = READ_BITS(RCU->CFG0, RCU_CFG0_PLLMF_3_0_MSK) >> RCU_CFG0_PLLMF_3_0_POS;
  uint32_t pllmf;
  uint8_t div_factor = 1;

  if (!pllmf_4) {
    if (pllmf_3_0 == 0b1101UL) {
      /* PLL factor is 6.5 */
      pllmf = 13;
      div_factor = 2;
    }
    else if (pllmf_3_0 == 0b1110UL || pllmf_3_0 == 0b1111UL) {
      /* PLL factor is 16 */
      pllmf = 16;
    }
    else {
      /* pllmf are offset by 2 */
      pllmf = ((pllmf_4 << 4) | pllmf_3_0) + 2;
    }
  }
  else {
    /* pllmf are offset by 1 */
    pllmf = ((pllmf_4 << 4) | pllmf_3_0) + 1;
  }
  
  /* if using IRC8M as source */
  if (READ_BITS(RCU->CFG0, RCU_CFG0_PLLSEL_MSK) == RCU_PLL_USE_IRC8M) {
    return (IRC8M_VALUE / 2) * pllmf / div_factor;
  }

  /* else uses PREDV0 */
  uint32_t predv0 = (READ_BITS(RCU->CFG1, RCU_CFG1_PREDV0_MSK) >> RCU_CFG1_PREDV0_POS) + 1;

  /* if using HXTAL as source */
  if (READ_BITS(RCU->CFG1, RCU_CFG1_PREDV0SEL_MSK) == RCU_PREDV0_USE_HXTAL) {
    return HXTAL_VALUE * pllmf / predv0 / div_factor;
  }

  /* else uses PLL1MF */
  uint32_t pll1mf = READ_BITS(RCU->CFG1, RCU_CFG1_PLL1MF_MSK) >> RCU_CFG1_PLL1MF_POS;
  if (pll1mf != 0b1111UL) {
    /* pll1mf are offset by 2 */
    pll1mf += 2;
  }
  else {
    /* PLL1 factor is 20 */
    pll1mf = 20;
  }

  uint32_t predv1 = (READ_BITS(RCU->CFG1, RCU_CFG1_PREDV1_MSK) >> RCU_CFG1_PREDV1_POS) + 1;

  return HXTAL_VALUE * pll1mf * pllmf / predv0 / predv1 / div_factor;
}

uint32_t HAL_RCU_getAHBClockFreq() {
  uint32_t ahbpsc = READ_BITS(RCU->CFG0, RCU_CFG0_AHBPSC_MSK) >> RCU_CFG0_AHBPSC_POS;
  uint32_t ck_sys = HAL_RCU_getSysClockFreq();
  switch (ahbpsc) {
    case 0b1000UL:
      return ck_sys / 2;
    case 0b1001UL:
      return ck_sys / 4;
    case 0b1010UL:
      return ck_sys / 8;
    case 0b1011UL:
      return ck_sys / 16;
    case 0b1100UL:
      return ck_sys / 64;
    case 0b1101UL:
      return ck_sys / 128;
    case 0b1110UL:
      return ck_sys / 256;
    case 0b1111UL:
      return ck_sys / 512;
  }
  return ck_sys;
}

uint32_t HAL_RCU_getAPB1ClockFreq() {
  uint32_t apb1psc = READ_BITS(RCU->CFG0, RCU_CFG0_APB1PSC_MSK) >> RCU_CFG0_APB1PSC_POS;
  uint32_t ck_ahb = HAL_RCU_getAHBClockFreq();
  switch (apb1psc) {
    case 0b100UL:
      return ck_ahb / 2;
    case 0b101UL:
      return ck_ahb / 4;
    case 0b110UL:
      return ck_ahb / 8;
    case 0b111UL:
      return ck_ahb / 16;
  }
  return ck_ahb;
}

uint32_t HAL_RCU_getAPB2ClockFreq() {
  uint32_t apb2psc = READ_BITS(RCU->CFG0, RCU_CFG0_APB2PSC_MSK) >> RCU_CFG0_APB2PSC_POS;
  uint32_t ck_ahb = HAL_RCU_getAHBClockFreq();
  switch (apb2psc) {
    case 0b100UL:
      return ck_ahb / 2;
    case 0b101UL:
      return ck_ahb / 4;
    case 0b110UL:
      return ck_ahb / 8;
    case 0b111UL:
      return ck_ahb / 16;
  }
  return ck_ahb;
}

void HAL_RCU_configADCClock(uint32_t value) {
  /* set the ADC prescaler factor */
  CLEAR_BITS(RCU->CFG0, RCU_CFG0_ADCPSC_1_0_MSK | RCU_CFG0_ADCPSC_2_MSK);
  SET_BITS(RCU->CFG0, value);
}
