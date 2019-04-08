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

#include <assert.h>
#include <arch_helpers.h>
#include <arm_gic.h>
#include <debug.h>
#include <cci400.h>
#include <errno.h>
#include <gic_v2.h>

#include <mmio.h>
#include <platform.h>
#include <platform_def.h>
#include <psci.h>
# include "rpi3_private.h"


# define BOOT_ENTRY_ADDR 0
# define NUM_BOOT_ENTRY	 8

static int cpu_release(int nr, uint64_t boot_addr)
{
  /*
    Each core is spinning at its respective address
  */
  uint64_t *table = (uint64_t *)bl31_get_spin_tbl_addr();

  if (nr <= 0) {
    return PSCI_E_NOT_PRESENT;
  }

  table = table + (nr * NUM_BOOT_ENTRY);

  table[BOOT_ENTRY_ADDR] = boot_addr;

# if 0
  INFO("%s(%d): nr: %d boot_addr: 0x%llx table: 0x%llx\n",
	 __func__, __LINE__,
	 nr, boot_addr, table);
# endif

  return PSCI_E_SUCCESS;
}

/*******************************************************************************
 * RPI3 handler called when an affinity instance is about to be turned on. The
 * level and mpidr determine the affinity instance.
 ******************************************************************************/
//extern int psci_0_2_cpu_on_64(unsigned long cpuid, uint64_t entry_point); /* in bl31_plat_setup.c */
int32_t rpi3_affinst_on(uint64_t mpidr,
			  uint64_t sec_entrypoint,
			  uint32_t afflvl,
			  uint32_t state)
{
	int cpu, cluster;
	unsigned long linear_id;
	unsigned int reg;
	int status = PSCI_E_SUCCESS;

	linear_id = platform_get_core_pos(mpidr);
	cluster = (mpidr & MPIDR_CLUSTER_MASK) >> MPIDR_AFF1_SHIFT;
	cpu = mpidr & MPIDR_CPU_MASK;

	(void)linear_id;
	(void)reg;
	(void)cluster;
	(void)cpu;

	VERBOSE("#%s(%d): mpidr:%llx, afflvl:%x, state:%x linear_id: %d\n",
		  __func__, __LINE__,
		  mpidr, afflvl, state, linear_id);

	/* directly return for power on */
	if (state == PSCI_STATE_ON)
		return PSCI_E_SUCCESS;

	switch (afflvl) {
	case MPIDR_AFFLVL0:
          status = cpu_release(linear_id, sec_entrypoint);
          break;

	case MPIDR_AFFLVL1:
          status = PSCI_E_SUCCESS; /* do nothing at cluster level */
	  break;
	}
	return status;
}

/*******************************************************************************
 * Handler called when an affinity instance has just been powered on after
 * being turned off earlier. The level and mpidr determine the affinity
 * instance. The 'state' arg. allows the platform to decide whether the cluster
 * was turned off prior to wakeup and do what's necessary to setup it up
 * correctly.
 ******************************************************************************/
void rpi3_affinst_on_finish(uint32_t afflvl, uint32_t state)
{
	unsigned long mpidr;
	unsigned long linear_id;

	(void)mpidr;
	(void)linear_id;
}


static void __dead2 rpi3_system_off(void)
{
   INFO("Pull the plug.\n");
   for (;;);
}

#define RESET_TIMEOUT 10

struct bcm2835_wdog_regs {
	uint32_t unknown0[7];
	uint32_t rstc;
	uint32_t unknown1;
	uint32_t wdog;
};

#define BCM2835_WDOG_PASSWORD			0x5a000000
#define BCM2835_WDOG_PHYSADDR			0x3f100000
#define BCM2835_WDOG_RSTC_WRCFG_MASK		0x00000030
#define BCM2835_WDOG_RSTC_WRCFG_FULL_RESET	0x00000020

static void reset_cpu(void)
{
	struct bcm2835_wdog_regs *regs =
		(struct bcm2835_wdog_regs *)BCM2835_WDOG_PHYSADDR;
	uint32_t rstc;

	rstc = regs->rstc;
	rstc &= ~BCM2835_WDOG_RSTC_WRCFG_MASK;
	rstc |= BCM2835_WDOG_RSTC_WRCFG_FULL_RESET;

	regs->wdog = BCM2835_WDOG_PASSWORD | RESET_TIMEOUT;
	regs->rstc = BCM2835_WDOG_PASSWORD | rstc;

}

#define DCFG_RSTCR_RESET_REQ	0x2
#define CONFIG_SYS_IMMR				0x01000000
#define CONFIG_SYS_FSL_GUTS_ADDR		(CONFIG_SYS_IMMR + 0x00ee0000)
#define TGT 0x01ee00b0
static void __dead2 rpi3_system_reset(void)
{

  reset_cpu();

  for (;;);

}

/*******************************************************************************
 * Export the platform handlers to enable psci to invoke them
 ******************************************************************************/
static const plat_pm_ops_t rpi3_ops = {
  .affinst_on		= rpi3_affinst_on,
  .affinst_on_finish	= rpi3_affinst_on_finish,
  .affinst_off		= NULL,
  .affinst_standby	= NULL,
  .affinst_suspend	= NULL,
  .affinst_suspend_finish	= NULL,
  .system_off		= rpi3_system_off,
  .system_reset		= rpi3_system_reset,
};

/*******************************************************************************
 * Export the platform specific power ops.
 ******************************************************************************/
int32_t platform_setup_pm(const plat_pm_ops_t **plat_ops)
{
	*plat_ops = &rpi3_ops;
	return 0;
}
