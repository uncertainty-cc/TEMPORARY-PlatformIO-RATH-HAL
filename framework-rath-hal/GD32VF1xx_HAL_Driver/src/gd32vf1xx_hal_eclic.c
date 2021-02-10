
#include "gd32vf1xx_hal_eclic.h"

void HAL_ECLIC_enableGlobalInterrupt(void) {
  /* set machine interrupt enable bit */
  set_csr(mstatus, MSTATUS_MIE);
}

void HAL_ECLIC_setPriorityGroup(uint32_t prigroup) {
  eclic_set_nlbits(prigroup);
}

void HAL_ECLIC_enableIRQ(uint32_t source, uint8_t lvl_abs, uint8_t priority) {
  eclic_enable_interrupt(source);
  eclic_set_irq_lvl_abs(source, lvl_abs);
  eclic_set_irq_priority(source, priority);
}