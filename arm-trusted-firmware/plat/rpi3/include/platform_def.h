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

#ifndef __PLATFORM_DEF_H__
#define __PLATFORM_DEF_H__

# if 0
# define RPI3_CONFIG
# endif

# define CFG_USE_UBOOT_HOOKS

 #define PLATFORM_MAX_AFFLVL      MPIDR_AFFLVL1

#include <arch.h>
#include "../rpi3_def.h"

/*******************************************************************************
 * Platform binary types for linking
 ******************************************************************************/
#define PLATFORM_LINKER_FORMAT          "elf64-littleaarch64"
#define PLATFORM_LINKER_ARCH            aarch64

/*******************************************************************************
 * Generic platform constants
 ******************************************************************************/

/* Size of cacheable stacks */
#define PLATFORM_STACK_SIZE		0x800

/* EL3 Runtime Firmware BL3-1 */
#define BL31_IMAGE_NAME			"bl31.bin"

/* Secure Payload BL3-2 (Trusted OS) */
#define BL32_IMAGE_NAME			"bl32.bin"

/* Non-Trusted Firmware BL3-3 */
#define BL33_IMAGE_NAME			"bl33.bin" /* e.g. uImage */

/* Firmware Image Package */
#define FIP_IMAGE_NAME			"fip.bin"

#define PLATFORM_PER_CORE_EL0_STACK     PLATFORM_STACK_SIZE /* 4096 */
#define PLATFORM_CACHE_LINE_SIZE	64
#define PLATFORM_CLUSTER_COUNT		1
#define PLATFORM_CORE_COUNT             4
#define PLATFORM_NUM_AFFS		(PLATFORM_CLUSTER_COUNT + \
					 PLATFORM_CORE_COUNT)
#define MAX_IO_DEVICES			3
#define MAX_IO_HANDLES			4

/*******************************************************************************
 * Platform memory map related constants
 ******************************************************************************/

#define DDR_BASE			0x00000000

/*
  Make the linker happy ... where to link bl31
*/
# define BL2_LIMIT                       (DRAM_NS_BASE + DRAM_NS_SIZE)	   /* BL31 load address */

/*******************************************************************************
 * BL3-1 specific defines.
 ******************************************************************************/
#define BL31_BASE			(BL2_LIMIT)
#define BL31_LIMIT			(BL31_BASE + 0x40000)


/*******************************************************************************
 * BL3-2 specific defines.
 ******************************************************************************/

/*
 * The TSP can execute from DRAM.
 */

#define BL32_SRAM_BASE                  0x10010000 /* 64k chunk */
#define BL32_SRAM_LIMIT                 (64 * 1024) /* not very big */

#define BL32_DRAM_BASE                  DRAM_SEC_BASE
#define BL32_DRAM_LIMIT                 (DRAM_SEC_BASE+DRAM_SEC_SIZE)

#if (PLAT_TSP_LOCATION_ID == PLAT_TRUSTED_DRAM_ID)
#define TSP_SEC_MEM_BASE		BL32_DRAM_BASE
#define TSP_SEC_MEM_SIZE		(BL32_DRAM_LIMIT - BL32_DRAM_BASE)
#define BL32_BASE			BL32_DRAM_BASE
#define BL32_LIMIT			BL32_DRAM_LIMIT
#else
#error "Unsupported PLAT_TSP_LOCATION_ID value"
#endif

/*******************************************************************************
 * BL3-0 specific defines:
 *
 * BL3-0 is loaded for mcu firmware, firstly load it into temperary buffer
 * into 0x0100_0000; then BL2 will parse the sections and load then into
 * seperated buffers as needed.
 *
 ******************************************************************************/
#define BL30_BASE			(DRAM_NS_BASE + 0x01000000)
#define BL30_LIMIT			(DRAM_NS_BASE + 0x01100000)
#define BL30_SIZE			(BL30_LIMIT - BL30_BASE)

/*******************************************************************************
 * Load address of BL3-3 for RPi
 ******************************************************************************/
#define NS_IMAGE_OFFSET			(0x00080000)  /* Linux load address */

/*******************************************************************************
 * Platform specific page table and MMU setup constants
 ******************************************************************************/
#define ADDR_SPACE_SIZE			(1ull << 32)

# define DEBUG_XLAT_TABLE 0 /* enables page table dumping/debug in xlat_tables.c */

#if IMAGE_BL1 || IMAGE_BL31 || IMAGE_BL32
# define MAX_XLAT_TABLES		4
#endif

#if IMAGE_BL2
# define MAX_XLAT_TABLES		4
#endif

#define MAX_MMAP_REGIONS		16

/*******************************************************************************
 * Declarations and constants to access the mailboxes safely. Each mailbox is
 * aligned on the biggest cache line size in the platform. This is known only
 * to the platform as it might have a combination of integrated and external
 * caches. Such alignment ensures that two maiboxes do not sit on the same cache
 * line at any cache level. They could belong to different cpus/clusters &
 * get written while being protected by different locks causing corruption of
 * a valid mailbox address.
 ******************************************************************************/
#define CACHE_WRITEBACK_SHIFT   6
#define CACHE_WRITEBACK_GRANULE (1 << CACHE_WRITEBACK_SHIFT)

#endif /* __PLATFORM_DEF_H__ */
