/*
 * Copyright (c) 2016, Sequitur Labs Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <arch.h>
#include <asm_macros.S>
#include <bl_common.h>
#include <cortex_a53.h>
#include <cpu_macros.S>
#include <platform_def.h>
#include "../rpi3_def.h"


	.globl	plat_crash_console_init
	.globl	plat_crash_console_putc
	.globl	plat_report_exception
	.globl	plat_reset_handler
	.globl	platform_get_core_pos
	.globl	platform_mem_init
	.globl  platform_set_el0_stack
	.globl  platform_spin_core
	.globl  plat_print_gic_regs
	.globl  plat_print_interconnect_regs
	.globl  plat_interrupt_type_to_line
	.globl  plat_ic_get_pending_interrupt_type
	.globl  plat_report_smc64

	/* Define a crash console for the plaform */
#define RPI3_CRASH_CONSOLE_BASE		UART0_BASE

	/* ---------------------------------------------
	 * int plat_crash_console_init(void)
	 * Function to initialize the crash console
	 * without a C Runtime to print crash report.
	 * Clobber list : x0, x1, x2
	 * ---------------------------------------------
	 */
func plat_crash_console_init
	mov x0, #1
	ret
	b dummy_console_init

	/* ---------------------------------------------
	 * int plat_crash_console_putc(int c)
	 * Function to print a character on the crash
	 * console without a C Runtime.
	 * Clobber list : x1, x2
	 * ---------------------------------------------
	 */
func _wait_for_xmitr
nr:	
	mov     x1, #0x5054                     // #20564
	movk    x1, #0x3f21, lsl #16
	ldr     w1, [x1]
	tbz     w1, #5, nr
	ret

func _console_putc
	mov	x2, x30 // lr
	bl	_wait_for_xmitr
	mov     x1, #0x5040                    // #20544
	movk    x1, #0x3f21, lsl #16
	str	w0, [x1]
	mov	w1, w0
	and     w1, w1, #0xff
	cmp     w1, #0xa
	b.ne	xi
	bl	_wait_for_xmitr
	mov     x1, #0x5040                    // #20544
	movk    x1, #0x3f21, lsl #16
	mov     w0, #0xd
	str	w0, [x1]
xi:	mov	x30, x2
	ret
	
func plat_crash_console_putc
	b	_console_putc

smc64_msg:
	.asciz "SMC_64 entered\n"
	
func plat_report_smc64
	mov	x9, x30
	adr	x3, smc64_msg
1:
	ldrb	w0, [x3], #0x1
	cbz	x0, 2f
	bl	plat_crash_console_putc
	b	1b
2:	
	ret	x9
	
	/* ---------------------------------------------
	 * void plat_report_exception(unsigned int type)
	 * Function to report an unhandled exception
	 * with platform-specific means.
	 * ---------------------------------------------
	 */
func plat_report_exception
	mov	x8, x30

	adr	x4, plat_err_str
	bl	asm_print_str

	adr	x4, esr_el3_str
	bl	asm_print_str

	mrs	x4, esr_el3
	bl	asm_print_hex

	adr	x4, elr_el3_str
	bl	asm_print_str

	mrs	x4, elr_el3
	bl	asm_print_hex

	mov	x30, x8
	ret

	/* -----------------------------------------------------
	 * void plat_reset_handler(void);
	 *
	 * Implement workaround for defect id 831273 by enabling
	 * an event stream every 65536 cycles and set the L2 RAM
	 * latencies for Cortex-A57.
	 * -----------------------------------------------------
	 */
func plat_reset_handler
	/* do nothing */
	ret

	/*
	 * Return 0 to 3
	 */
func platform_get_core_pos
	and	x1, x0, #MPIDR_CPU_MASK
	and	x0, x0, #MPIDR_CLUSTER_MASK
	add	x0, x1, x0, LSR #6
	ret


	/*
	  No GIC on PI
	*/
func plat_print_gic_regs
	ret

func plat_print_interconnect_regs
	ret

func plat_interrupt_type_to_line
	ret

func plat_ic_get_pending_interrupt_type
	ret
	
	/* -----------------------------------------------------
	 * void platform_mem_init(void);
	 *
	 * We don't need to carry out any memory initialization
	 * on LS1043. The Secure RAM is accessible straight away.
	 * -----------------------------------------------------
	 */
func platform_mem_init
	ret

func platform_spin_core
	/*
	 * MPIDR_EL1 Fields:
	 * MPIDR[1:0] = AFF0_CPUID <- Core ID (0,1)
	 * MPIDR[7:2] = AFF0_RES
	 * MPIDR[15:8] = AFF1_CLUSTERID <- Cluster ID (0,1,2,3)
	 * MPIDR[23:16] = AFF2_CLUSTERID
	 * MPIDR[24] = MT
	 * MPIDR[29:25] = RES0
	 * MPIDR[30] = U
	 * MPIDR[31] = ME
	 * MPIDR[39:32] = AFF3
	 *
	 * Linear Processor ID (LPID) calculation from MPIDR_EL1:
	 * (We only use AFF0_CPUID and AFF1_CLUSTERID for now
	 * until AFF2_CLUSTERID and AFF3 have non-zero values)
	 *
	 * LPID = MPIDR[15:8] | MPIDR[1:0]
	 */
	mrs	x0, mpidr_el1
	ubfm	x1, x0, #8, #15
	ubfm	x2, x0, #0, #1
	orr	x10, x2, x1, lsl #2	/* x10 has LPID */
	ubfm    x9, x0, #0, #15         /* x9 contains MPIDR[15:0] */
	/*
	 * offset of the spin table element for this core from start of spin
	 * table (each elem is padded to 64 bytes)
	 */
	lsl	x1, x10, #6
	ldr	x0, =__spin_table
	/* address of this cpus spin table element */
	add	x11, x1, x0

slave_cpu:
	ldr	x0, [x11]
	cbz	x0, slave_cpu
	mov	x1, #1
	cbz	x1, slave_exit
slave_exit:
	br	x0			/* branch to the given address */

.section .rodata.rev_err_str, "aS"
plat_err_str:
	.asciz "\nPlatform exception reporting:"
esr_el3_str:
	.asciz "\nESR_EL3: "
elr_el3_str:
	.asciz "\nELR_EL3: "
