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


#ifndef __RPI3_PRIVATE_H__
#define __RPI3_PRIVATE_H__

#include <bl_common.h>

/*******************************************************************************
 * This structure represents the superset of information that is passed to
 * BL3-1 e.g. while passing control to it from BL2 which is bl31_params
 * and other platform specific params
 ******************************************************************************/
typedef struct bl2_to_bl31_params_mem {
	struct bl31_params bl31_params;
	struct image_info bl31_image_info;
	struct image_info bl32_image_info;
	struct image_info bl33_image_info;
	struct entry_point_info bl33_ep_info;
	struct entry_point_info bl32_ep_info;
	struct entry_point_info bl31_ep_info;
} bl2_to_bl31_params_mem_t;

/*******************************************************************************
 * Function and variable prototypes
 ******************************************************************************/

void configure_mmu_el3(unsigned long total_base,
		       unsigned long total_size,
		       unsigned long ro_start,
		       unsigned long ro_limit,
		       unsigned long coh_start,
		       unsigned long coh_limit);

extern void plat_gic_init(void);

extern uint64_t *bl31_get_spin_tbl_addr(void);
extern void platform_spin_core(void);

#endif /* __RPI3_PRIVATE_H__ */
