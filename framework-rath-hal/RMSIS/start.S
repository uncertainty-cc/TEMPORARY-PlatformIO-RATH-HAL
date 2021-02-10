// See LICENSE for license details.

// style follows the OpenTitan style guide https://docs.opentitan.org/doc/rm/asm_coding_style/

#include "riscv_encoding.h"
#include "riscv_bits.h"
#include "n200_eclic.h"

/* define the startup function */
.globl _start
.type _start, @function

_start:
  /* clear bit in csr MSTATUS_MIE field to turn off global interrupt. */
  csrc CSR_BUMBLEBEE_MSTATUS, MSTATUS_MIE

  /* Jump to logical address first to ensure correct operation of RAM region  */
  la a0, _start
  li a1, 0x20000000

  /* test if this is boot on SRAM */
  bleu a1, a0, _start0800

  /* test if this is boot on main Flash */
  srli a1, a1, 2
  bleu a1, a0, _start0800

  /* else must be boot on bootloader */
  la a0, _start0800
  add a0, a0, a1
  jr a0

_start0800:
  /* set the the NMI base to share with mtvec */
  /* mnvec will be same with mtvec; mcause.EXCCODE of NMI will be 0xFFF. */
  li t0, 0x200
  csrs CSR_BUMBLEBEE_MMISC_CTL, t0

  /* initialize the mtvt */
  la t0, vector_base
  csrw CSR_BUMBLEBEE_MTVT, t0

  /* initialize and enable the mtvt2 */
  la t0, irq_entry
  csrw CSR_BUMBLEBEE_MTVT2, t0
  csrs CSR_BUMBLEBEE_MTVT2, 0x1

  /* initialize the CSR MTVEC for the Trap ane NMI base addr */
  la t0, trap_entry
  csrw CSR_BUMBLEBEE_MTVEC, t0

  #ifdef __riscv_flen
  /* enable FPU */
  li t0, MSTATUS_FS
  csrs mstatus, t0
  csrw fcsr, x0
  #endif

  /* for the code below, inline assembler options are used to produce
    ```
    auipc gp, %pcrel_hi(__global_pointer$)
    addi gp, gp, %pcrel_lo(__global_pointer$)
    ```
    instead of
    ```
    addi gp, gp, 0
    ```
    (https://sourceware.org/binutils/docs-2.31/as/RISC_002dV_002dDirectives.html#RISC_002dV_002dDirectives)
  */
  .option push
  .option norelax
  la gp, __global_pointer$

  .option pop
  la sp, _sp

  /* Load data section */
  la a0, _data_lma
  la a1, _data
  la a2, _edata
  bgeu a1, a2, LoopCopyDataInit

CopyDataInit:
  lw t0, (a0)
  sw t0, (a1)
  addi a0, a0, 4
  addi a1, a1, 4
  bltu a1, a2, CopyDataInit

LoopCopyDataInit:
  /* Clear bss section */
  la a0, __bss_start
  la a1, _end
  bgeu a0, a1, LoopFillZerobss

FillZerobss:
  sw x0, (a0)
  addi a0, a0, 4
  bltu a0, a1, FillZerobss

LoopFillZerobss:
  /* enable mcycle_minstret */
    csrci CSR_BUMBLEBEE_MCOUNTINHIBIT, 0x5

  /* intialize C environment */
  /* call global constructors */
  la a0, __libc_fini_array
  call atexit
  call __libc_init_array

  /* argc = argv = 0 */
  li a0, 0
  li a1, 0
  call main
  tail exit

/* fini: Code to execute after main */
infinite_loop:
  j infinite_loop

.global disable_mcycle_minstret
disable_mcycle_minstret:
  csrsi CSR_BUMBLEBEE_MCOUNTINHIBIT, 0x5
  ret

.global enable_mcycle_minstret
enable_mcycle_minstret:
  csrci CSR_BUMBLEBEE_MCOUNTINHIBIT, 0x5
  ret


.section .init
/******************************************************************************
*
* The minimal vector table for a Cortex M3.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
vector_base:
  j _start
  .align    2
  .word     0
  .word     0
  .word     eclic_msip_handler
  .word     0
  .word     0
  .word  	  0
  .word  	  eclic_mtip_handler
  .word  	  0
  .word  	  0
  .word  	  0
  .word  	  0
  .word  	  0
  .word  	  0
  .word  	  0
  .word 	  0
  .word  	  0
  .word  	  eclic_bwei_handler
  .word  	  eclic_pmovi_handler
  .word  	  WWDGT_IRQHandler
  .word     LVD_IRQHandler
  .word     TAMPER_IRQHandler
  .word     RTC_IRQHandler
  .word     FMC_IRQHandler
  .word     RCU_IRQHandler
  .word     EXTI0_IRQHandler
  .word     EXTI1_IRQHandler
  .word     EXTI2_IRQHandler
  .word     EXTI3_IRQHandler
  .word     EXTI4_IRQHandler
  .word     DMA0_Channel0_IRQHandler
  .word     DMA0_Channel1_IRQHandler
  .word     DMA0_Channel2_IRQHandler
  .word     DMA0_Channel3_IRQHandler
  .word     DMA0_Channel4_IRQHandler
  .word     DMA0_Channel5_IRQHandler
  .word     DMA0_Channel6_IRQHandler
  .word     ADC0_1_IRQHandler
  .word     CAN0_TX_IRQHandler
  .word     CAN0_RX0_IRQHandler
  .word     CAN0_RX1_IRQHandler
  .word     CAN0_EWMC_IRQHandler
  .word     EXTI5_9_IRQHandler
  .word     TIMER0_BRK_IRQHandler
  .word     TIMER0_UP_IRQHandler
  .word     TIMER0_TRG_CMT_IRQHandler
  .word     TIMER0_Channel_IRQHandler
  .word     TIMER1_IRQHandler
  .word     TIMER2_IRQHandler
  .word     TIMER3_IRQHandler
  .word     I2C0_EV_IRQHandler
  .word     I2C0_ER_IRQHandler
  .word     I2C1_EV_IRQHandler
  .word     I2C1_ER_IRQHandler
  .word     SPI0_IRQHandler
  .word     SPI1_IRQHandler
  .word     USART0_IRQHandler
  .word     USART1_IRQHandler
  .word     USART2_IRQHandler
  .word     EXTI10_15_IRQHandler
  .word     RTC_Alarm_IRQHandler
  .word     USBFS_WKUP_IRQHandler
  .word     0
  .word     0
  .word     0
  .word     0
  .word     0
  .word     EXMC_IRQHandler
  .word     0
  .word     TIMER4_IRQHandler
  .word     SPI2_IRQHandler
  .word     UART3_IRQHandler
  .word     UART4_IRQHandler
  .word     TIMER5_IRQHandler
  .word     TIMER6_IRQHandler
  .word     DMA1_Channel0_IRQHandler
  .word     DMA1_Channel1_IRQHandler
  .word     DMA1_Channel2_IRQHandler
  .word     DMA1_Channel3_IRQHandler
  .word     DMA1_Channel4_IRQHandler
  .word     0
  .word     0
  .word     CAN1_TX_IRQHandler
  .word     CAN1_RX0_IRQHandler
  .word     CAN1_RX1_IRQHandler
  .word     CAN1_EWMC_IRQHandler
  .word     USBFS_IRQHandler

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/
  .weak  eclic_msip_handler
  .weak  eclic_mtip_handler
  .weak  eclic_bwei_handler
  .weak  eclic_pmovi_handler
  .weak  WWDGT_IRQHandler
  .weak  LVD_IRQHandler
  .weak  TAMPER_IRQHandler
  .weak  RTC_IRQHandler
  .weak  FMC_IRQHandler
  .weak  RCU_IRQHandler
  .weak  EXTI0_IRQHandler
  .weak  EXTI1_IRQHandler
  .weak  EXTI2_IRQHandler
  .weak  EXTI3_IRQHandler
  .weak  EXTI4_IRQHandler
  .weak  DMA0_Channel0_IRQHandler
  .weak  DMA0_Channel1_IRQHandler
  .weak  DMA0_Channel2_IRQHandler
  .weak  DMA0_Channel3_IRQHandler
  .weak  DMA0_Channel4_IRQHandler
  .weak  DMA0_Channel5_IRQHandler
  .weak  DMA0_Channel6_IRQHandler
  .weak  ADC0_1_IRQHandler
  .weak  CAN0_TX_IRQHandler
  .weak  CAN0_RX0_IRQHandler
  .weak  CAN0_RX1_IRQHandler
  .weak  CAN0_EWMC_IRQHandler
  .weak  EXTI5_9_IRQHandler
  .weak  TIMER0_BRK_IRQHandler
  .weak  TIMER0_UP_IRQHandler
  .weak  TIMER0_TRG_CMT_IRQHandler
  .weak  TIMER0_Channel_IRQHandler
  .weak  TIMER1_IRQHandler
  .weak  TIMER2_IRQHandler
  .weak  TIMER3_IRQHandler
  .weak  I2C0_EV_IRQHandler
  .weak  I2C0_ER_IRQHandler
  .weak  I2C1_EV_IRQHandler
  .weak  I2C1_ER_IRQHandler
  .weak  SPI0_IRQHandler
  .weak  SPI1_IRQHandler
  .weak  USART0_IRQHandler
  .weak  USART1_IRQHandler
  .weak  USART2_IRQHandler
  .weak  EXTI10_15_IRQHandler
  .weak  RTC_Alarm_IRQHandler
  .weak  USBFS_WKUP_IRQHandler
  .weak  EXMC_IRQHandler
  .weak  TIMER4_IRQHandler
  .weak  SPI2_IRQHandler
  .weak  UART3_IRQHandler
  .weak  UART4_IRQHandler
  .weak  TIMER5_IRQHandler
  .weak  TIMER6_IRQHandler
  .weak  DMA1_Channel0_IRQHandler
  .weak  DMA1_Channel1_IRQHandler
  .weak  DMA1_Channel2_IRQHandler
  .weak  DMA1_Channel3_IRQHandler
  .weak  DMA1_Channel4_IRQHandler
  .weak  CAN1_TX_IRQHandler
  .weak  CAN1_RX0_IRQHandler
  .weak  CAN1_RX1_IRQHandler
  .weak  CAN1_EWMC_IRQHandler
  .weak  USBFS_IRQHandler



###############################################
###############################################
#Save caller registers
.macro SAVE_CONTEXT

#ifdef __riscv_flen
  #if (__riscv_flen==64 )
    addi sp, sp, -20*REGBYTES - 20*FPREGBYTES
  #else
    addi sp, sp, -20*REGBYTES
  #endif
#else
  addi sp, sp, -20*REGBYTES
#endif
  STORE x1, 0*REGBYTES(sp)
  STORE x4, 1*REGBYTES(sp)
  STORE x5, 2*REGBYTES(sp)
  STORE x6, 3*REGBYTES(sp)
  STORE x7, 4*REGBYTES(sp)
  STORE x10, 5*REGBYTES(sp)
  STORE x11, 6*REGBYTES(sp)
  STORE x12, 7*REGBYTES(sp)
  STORE x13, 8*REGBYTES(sp)
  STORE x14, 9*REGBYTES(sp)
  STORE x15, 10*REGBYTES(sp)
#ifndef __riscv_32e
  STORE x16, 11*REGBYTES(sp)
  STORE x17, 12*REGBYTES(sp)
  STORE x28, 13*REGBYTES(sp)
  STORE x29, 14*REGBYTES(sp)
  STORE x30, 15*REGBYTES(sp)
  STORE x31, 16*REGBYTES(sp)
#endif

#ifdef __riscv_flen
  #if (__riscv_flen == 64)
  FPSTORE f0, (20*REGBYTES + 0*FPREGBYTES)(sp)
  FPSTORE f1, (20*REGBYTES + 1*FPREGBYTES)(sp)
  FPSTORE f2, (20*REGBYTES + 2*FPREGBYTES)(sp)
  FPSTORE f3, (20*REGBYTES + 3*FPREGBYTES)(sp)
  FPSTORE f4, (20*REGBYTES + 4*FPREGBYTES)(sp)
  FPSTORE f5, (20*REGBYTES + 5*FPREGBYTES)(sp)
  FPSTORE f6, (20*REGBYTES + 6*FPREGBYTES)(sp)
  FPSTORE f7, (20*REGBYTES + 7*FPREGBYTES)(sp)
  FPSTORE f10, (20*REGBYTES + 8*FPREGBYTES)(sp)
  FPSTORE f11, (20*REGBYTES + 9*FPREGBYTES)(sp)
  FPSTORE f12, (20*REGBYTES + 10*FPREGBYTES)(sp)
  FPSTORE f13, (20*REGBYTES + 11*FPREGBYTES)(sp)
  FPSTORE f14, (20*REGBYTES + 12*FPREGBYTES)(sp)
  FPSTORE f15, (20*REGBYTES + 13*FPREGBYTES)(sp)
  FPSTORE f16, (20*REGBYTES + 14*FPREGBYTES)(sp)
  FPSTORE f17, (20*REGBYTES + 15*FPREGBYTES)(sp)
  FPSTORE f28, (20*REGBYTES + 16*FPREGBYTES)(sp)
  FPSTORE f29, (20*REGBYTES + 17*FPREGBYTES)(sp)
  FPSTORE f30, (20*REGBYTES + 18*FPREGBYTES)(sp)
  FPSTORE f31, (20*REGBYTES + 19*FPREGBYTES)(sp)
  #endif
#endif


.endm


###############################################
###############################################
#restore caller registers
.macro RESTORE_CONTEXT
  LOAD x1, 0*REGBYTES(sp)
  LOAD x4, 1*REGBYTES(sp)
  LOAD x5, 2*REGBYTES(sp)
  LOAD x6, 3*REGBYTES(sp)
  LOAD x7, 4*REGBYTES(sp)
  LOAD x10, 5*REGBYTES(sp)
  LOAD x11, 6*REGBYTES(sp)
  LOAD x12, 7*REGBYTES(sp)
  LOAD x13, 8*REGBYTES(sp)
  LOAD x14, 9*REGBYTES(sp)
  LOAD x15, 10*REGBYTES(sp)
#ifndef __riscv_32e
  LOAD x16, 11*REGBYTES(sp)
  LOAD x17, 12*REGBYTES(sp)
  LOAD x28, 13*REGBYTES(sp)
  LOAD x29, 14*REGBYTES(sp)
  LOAD x30, 15*REGBYTES(sp)
  LOAD x31, 16*REGBYTES(sp)
#endif


#ifdef __riscv_flen
  #if (__riscv_flen==64)
/* Restore fp caller registers */
  FPLOAD f0, (20*REGBYTES + 0*FPREGBYTES)(sp)
  FPLOAD f1, (20*REGBYTES + 1*FPREGBYTES)(sp)
  FPLOAD f2, (20*REGBYTES + 2*FPREGBYTES)(sp)
  FPLOAD f3, (20*REGBYTES + 3*FPREGBYTES)(sp)
  FPLOAD f4, (20*REGBYTES + 4*FPREGBYTES)(sp)
  FPLOAD f5, (20*REGBYTES + 5*FPREGBYTES)(sp)
  FPLOAD f6, (20*REGBYTES + 6*FPREGBYTES)(sp)
  FPLOAD f7, (20*REGBYTES + 7*FPREGBYTES)(sp)
  FPLOAD f10, (20*REGBYTES + 8*FPREGBYTES)(sp)
  FPLOAD f11, (20*REGBYTES + 9*FPREGBYTES)(sp)
  FPLOAD f12, (20*REGBYTES + 10*FPREGBYTES)(sp)
  FPLOAD f13, (20*REGBYTES + 11*FPREGBYTES)(sp)
  FPLOAD f14, (20*REGBYTES + 12*FPREGBYTES)(sp)
  FPLOAD f15, (20*REGBYTES + 13*FPREGBYTES)(sp)
  FPLOAD f16, (20*REGBYTES + 14*FPREGBYTES)(sp)
  FPLOAD f17, (20*REGBYTES + 15*FPREGBYTES)(sp)
  FPLOAD f28, (20*REGBYTES + 16*FPREGBYTES)(sp)
  FPLOAD f29, (20*REGBYTES + 17*FPREGBYTES)(sp)
  FPLOAD f30, (20*REGBYTES + 18*FPREGBYTES)(sp)
  FPLOAD f31, (20*REGBYTES + 19*FPREGBYTES)(sp)
  #endif
#endif


#ifdef __riscv_flen
  #if(__riscv_flen == 64 )
    addi sp, sp, 20*REGBYTES  + 20*FPREGBYTES
  #else
    addi sp, sp, 20*REGBYTES
  #endif
#else
// De-allocate the stack space
 addi sp, sp, 20*REGBYTES
#endif
.endm

###############################################
###############################################
#restore caller registers
.macro RESTORE_CONTEXT_EXCPT_X5
  LOAD x1,  0*REGBYTES(sp)
  LOAD x6,  2*REGBYTES(sp)
  LOAD x7,  3*REGBYTES(sp)
  LOAD x10, 4*REGBYTES(sp)
  LOAD x11, 5*REGBYTES(sp)
  LOAD x12, 6*REGBYTES(sp)
  LOAD x13, 7*REGBYTES(sp)
  LOAD x14, 8*REGBYTES(sp)
  LOAD x15, 9*REGBYTES(sp)
#ifndef __riscv_32e
  LOAD x16, 10*REGBYTES(sp)
  LOAD x17, 11*REGBYTES(sp)
  LOAD x28, 12*REGBYTES(sp)
  LOAD x29, 13*REGBYTES(sp)
  LOAD x30, 14*REGBYTES(sp)
  LOAD x31, 15*REGBYTES(sp)
#endif
.endm

###############################################
###############################################
#restore caller registers
.macro RESTORE_CONTEXT_ONLY_X5
  LOAD x5,  1*REGBYTES(sp)
.endm

###############################################
###############################################
# Save the mepc and mstatus
#
.macro SAVE_EPC_STATUS
  csrr x5, CSR_BUMBLEBEE_MEPC
  STORE x5,  16*REGBYTES(sp)
  csrr x5, CSR_BUMBLEBEE_MSTATUS
  STORE x5,  17*REGBYTES(sp)
  csrr x5, CSR_BUMBLEBEE_MSUBM
  STORE x5,  18*REGBYTES(sp)
.endm

###############################################
###############################################
# Restore the mepc and mstatus
#
.macro RESTORE_EPC_STATUS
  LOAD x5,  16*REGBYTES(sp)
  csrw CSR_BUMBLEBEE_MEPC, x5
  LOAD x5,  17*REGBYTES(sp)
  csrw CSR_BUMBLEBEE_MSTATUS, x5
  LOAD x5,  18*REGBYTES(sp)
  csrw CSR_BUMBLEBEE_MSUBM, x5
.endm



###############################################
###############################################
// Trap entry point
//
  .section      .text.trap
  .align 6// In CLIC mode, the trap entry must be 64bytes aligned
  .global trap_entry
.weak trap_entry
trap_entry:
  // Allocate the stack space
 // addi sp, sp, -19*REGBYTES

  // Save the caller saving registers (context)
  SAVE_CONTEXT
  // Save the MEPC/Mstatus/Msubm reg
  SAVE_EPC_STATUS

     // Set the function argument
  csrr a0, mcause
  mv a1, sp
     // Call the function
  call handle_trap

  // Restore the MEPC/Mstatus/Msubm reg
  RESTORE_EPC_STATUS
  // Restore the caller saving registers (context)
  RESTORE_CONTEXT

  // De-allocate the stack space
 // addi sp, sp, 19*REGBYTES
  // Return to regular code
  mret


###############################################
###############################################
// IRQ entry point
//
  .section      .text.irq
  .align 2
  .global irq_entry
.weak irq_entry
irq_entry: // -------------> This label will be set to MTVT2 register
  // Allocate the stack space


  SAVE_CONTEXT// Save 16 regs

  //------This special CSR read operation, which is actually use mcause as operand to directly store it to memory
  csrrwi  x0, CSR_BUMBLEBEE_PUSHMCAUSE, 17
  //------This special CSR read operation, which is actually use mepc as operand to directly store it to memory
  csrrwi  x0, CSR_BUMBLEBEE_PUSHMEPC, 18
  //------This special CSR read operation, which is actually use Msubm as operand to directly store it to memory
  csrrwi  x0, CSR_BUMBLEBEE_PUSHMSUBM, 19

service_loop:
  //------This special CSR read/write operation, which is actually Claim the CLIC to find its pending highest
  // ID, if the ID is not 0, then automatically enable the mstatus.MIE, and jump to its vector-entry-label, and
  // update the link register
  csrrw ra, CSR_BUMBLEBEE_JALMNXTI, ra

  //RESTORE_CONTEXT_EXCPT_X5

  // ---- Critical section with interrupts disabled -----------------------
  csrc CSR_BUMBLEBEE_MSTATUS, MSTATUS_MIE   # Disable interrupts

  LOAD x5,  19*REGBYTES(sp)
  csrw CSR_BUMBLEBEE_MSUBM, x5
  LOAD x5,  18*REGBYTES(sp)
  csrw CSR_BUMBLEBEE_MEPC, x5
  LOAD x5,  17*REGBYTES(sp)
  csrw CSR_BUMBLEBEE_MCAUSE, x5


  RESTORE_CONTEXT

  // Return to regular code
  mret

