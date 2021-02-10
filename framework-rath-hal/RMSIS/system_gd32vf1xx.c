/**
 * @file system_gd32vf1xx.c
 * @author -T.K.- (t_k_233@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2021-01-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */


/* This file refers the RISC-V standard, some adjustments are made according to GigaDevice chips */

#include "system_gd32vf1xx.h"


/*handlers.c*/


//See LICENSE for license details.

__attribute__((weak)) uintptr_t handle_nmi() {
  write(1, "nmi\n", 5);
  _exit(1);
  return 0;
}


__attribute__((weak)) uintptr_t handle_trap(uintptr_t mcause, uintptr_t sp) {
  if((mcause & 0xFFF) == 0xFFF) {
    handle_nmi();
  }
  write(1, "trap\n", 5);
  //printf("In trap handler, the mcause is %d\n", mcause);
  //printf("In trap handler, the mepc is 0x%x\n", read_csr(mepc));
  //printf("In trap handler, the mtval is 0x%x\n", read_csr(mbadaddr));
  _exit(mcause);
  return 0;
}

void _init() {

	/* initialize ECLIC */
	eclic_init(ECLIC_NUM_INTERRUPTS);
	eclic_mode_enable();

	/* It must be NOTED:
	 * In the RISC-V arch, if user mode and PMP supported, then by default if PMP is not configured
	 *   with valid entries, then user mode cannot access any memory, and cannot execute any instructions.
	 * So if switch to user-mode and still want to continue, then you must configure PMP first
  */
	// pmp_open_all_space();
	// switch_m2u_mode();

  /* Before enter into main, add the cycle/instret disable by default to save power,
    only use them when needed to measure the cycle/instret */
	disable_mcycle_minstret();
}

void _fini() {}
