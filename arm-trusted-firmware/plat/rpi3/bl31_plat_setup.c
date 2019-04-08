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
#include <arm_gic.h>
#include <assert.h>
#include <bl31.h>
#include <bl_common.h>
#include <console.h>
#include <mmio.h>
#include <platform.h>
#include <stddef.h>
#include <string.h>
#include <platform_def.h>
#include "rpi3_def.h"
#include "rpi3_private.h"
#include "debug.h"

/*******************************************************************************
 * Declarations of linker defined symbols which will help us find the layout
 * of trusted RAM
 ******************************************************************************/
extern unsigned long __RO_START__;
extern unsigned long __RO_END__;

extern unsigned long __COHERENT_RAM_START__;
extern unsigned long __COHERENT_RAM_END__;

/*
 * The next 2 constants identify the extents of the code & RO data region.
 * These addresses are used by the MMU setup code and therefore they must be
 * page-aligned.  It is the responsibility of the linker script to ensure that
 * __RO_START__ and __RO_END__ linker symbols refer to page-aligned addresses.
 */
#define BL31_RO_BASE (unsigned long)(&__RO_START__)
#define BL31_RO_LIMIT (unsigned long)(&__RO_END__)

/*
 * The next 2 constants identify the extents of the coherent memory region.
 * These addresses are used by the MMU setup code and therefore they must be
 * page-aligned.  It is the responsibility of the linker script to ensure that
 * __COHERENT_RAM_START__ and __COHERENT_RAM_END__ linker symbols
 * refer to page-aligned addresses.
 */
#define BL31_COHERENT_RAM_BASE (unsigned long)(&__COHERENT_RAM_START__)
#define BL31_COHERENT_RAM_LIMIT (unsigned long)(&__COHERENT_RAM_END__)

/******************************************************************************
 * Placeholder variables for copying the arguments that have been passed to
 * BL3-1 from BL2.
 ******************************************************************************/
static entry_point_info_t bl32_ep_info;
static entry_point_info_t bl33_ep_info;

/*
  From opteed_main.c
*/
struct optee_header {
  uint32_t magic;
  uint8_t version;
  uint8_t arch;
  uint16_t flags;
  uint32_t init_size;
  uint32_t init_load_addr_hi;
  uint32_t init_load_addr_lo;
  uint32_t init_mem_usage;
  uint32_t paged_size;
};

#define OPTEE_MAGIC		0x4554504f
#define OPTEE_VERSION		1
#define OPTEE_ARCH_ARM32	0
#define OPTEE_ARCH_ARM64	1

static struct optee_header optee_header;



uint32_t __spin_table[64]
# if USE_COHERENT_MEM
  __attribute__ ((section("tzfw_coherent_mem")));
# endif

uint64_t *bl31_get_spin_tbl_addr(void)
{
  return(uint64_t *)(&__spin_table[0]);
}

uint64_t bl31_getspin(void)
{
  memset(&__spin_table[0], 0, sizeof(__spin_table));
  return(uint64_t)(platform_spin_core);
}

struct uboot_data {
  unsigned int kernel_load;
  unsigned int dtb_load;
  unsigned int el;
  unsigned int optee_load;
};

extern unsigned int __uboot_data;
void bl31_uboot_set_entries(unsigned int kernel, unsigned int dtb,
			    unsigned int el, unsigned int optee_load)
{
  struct uboot_data *data = (struct uboot_data *)(&__uboot_data);
  data->kernel_load = kernel;
  data->dtb_load = dtb;
  data->el = el;
  data->optee_load = optee_load;
}

unsigned int bl31_get_uboot_optee_load(void)
{
  struct uboot_data *data = (struct uboot_data *)(&__uboot_data);

  return(data->optee_load);
}

unsigned int bl31_get_uboot_kernel_load(void)
{
  struct uboot_data *data = (struct uboot_data *)(&__uboot_data);

  return(data->kernel_load);
}

unsigned int bl31_get_uboot_dtb_load(void)
{
  struct uboot_data *data = (struct uboot_data *)(&__uboot_data);

  return(data->dtb_load);
}

unsigned int bl31_get_uboot_el(void)
{
  struct uboot_data *data = (struct uboot_data *)(&__uboot_data);

  /* Running AARCH64 */
  if (data->el) {
    /* EL2 */
    return(0x3c9);
  }
  else {
    /* EL1 */
    return(0x3c5);
  }
}

/*******************************************************************************
 * Return a pointer to the 'entry_point_info' structure of the next image for
 * the security state specified. BL3-3 corresponds to the non-secure image type
 * while BL3-2 corresponds to the secure image type. A NULL pointer is returned
 * if the image does not exist.
 ******************************************************************************/
entry_point_info_t *bl31_plat_get_next_image_ep_info(uint32_t type)
{
	entry_point_info_t *next_image_info;

	next_image_info = (type == NON_SECURE) ? &bl33_ep_info : &bl32_ep_info;

	/* None of the images on this platform can have 0x0 as the entrypoint */
	if (next_image_info->pc)
		return next_image_info;
	else
		return NULL;
}

/*******************************************************************************
 * Perform any BL3-1 specific platform actions. Here is an opportunity to copy
 * parameters passed by the calling EL (S-EL1 in BL2 & S-EL3 in BL1) before they
 * are lost (potentially). This needs to be done before the MMU is initialized
 * so that the memory layout can be used while creating page tables. Also, BL2
 * has flushed this information to memory, so we are guaranteed to pick up good
 * data
 ******************************************************************************/
void bl31_early_platform_setup(bl31_params_t *from_bl2,
			       void *plat_params_from_bl2)
{
  /*
   * Copy BL3-2 and BL3-3 entry point information.
   * They are stored in Secure RAM, in BL2's address space.
   */
# if 0 /* old ... not being passed in */
  bl32_ep_info = *from_bl2->bl32_ep_info;
  bl33_ep_info = *from_bl2->bl33_ep_info;
# else
  bl31_set_next_image_type(NON_SECURE);

  SET_PARAM_HEAD(&bl32_ep_info,
		 PARAM_EP,
		 VERSION_1,
		 0);
  SET_SECURITY_STATE(bl32_ep_info.h.attr, SECURE);

  optee_header.magic = OPTEE_MAGIC;
  optee_header.version = OPTEE_VERSION;
  optee_header.arch = OPTEE_ARCH_ARM64;
  optee_header.init_load_addr_hi = 0;

  optee_header.init_load_addr_lo = bl31_get_uboot_optee_load();

  bl32_ep_info.pc = (uintptr_t)&optee_header;
  bl32_ep_info.spsr = 0;
  bl32_ep_info.args.arg0 = 0;
  bl32_ep_info.args.arg1 = 0;
  bl32_ep_info.args.arg2 = bl31_get_uboot_dtb_load();


  SET_PARAM_HEAD(&bl33_ep_info,
		 PARAM_EP,
		 VERSION_1,
		 0);
  SET_SECURITY_STATE(bl33_ep_info.h.attr, NON_SECURE);

  bl33_ep_info.pc = bl31_get_uboot_kernel_load();

  /*
    0x3c9: AA64, EL2H
    0x3c5: AA64, EL1h
  */
  bl33_ep_info.spsr = bl31_get_uboot_el();
  bl33_ep_info.args.arg0 = bl31_get_uboot_dtb_load();
  bl33_ep_info.args.arg1 = 0;
  bl33_ep_info.args.arg2 = 0;
  bl33_ep_info.args.arg3 = 0;
  bl33_ep_info.args.arg4 = 0;

# endif

}

/*******************************************************************************
 * Initialize the GIC.
 ******************************************************************************/
void bl31_platform_setup(void)
{
	/* Initialize the gic cpu and distributor interfaces */
	plat_gic_init();
}

/*******************************************************************************
 * Perform the very early platform specific architectural setup here. At the
 * moment this is only intializes the mmu in a quick and dirty way.
 ******************************************************************************/
void bl31_plat_arch_setup()
{
	configure_mmu_el3(BL31_RO_BASE,
			  BL31_COHERENT_RAM_LIMIT - BL31_RO_BASE,
			  BL31_RO_BASE,
			  BL31_RO_LIMIT,
			  BL31_COHERENT_RAM_BASE,
                          BL31_COHERENT_RAM_LIMIT);
}
