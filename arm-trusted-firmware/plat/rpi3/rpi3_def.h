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

#ifndef __RPI3_DEF_H__
#define __RPI3_DEF_H__

# define UART0_BASE                     0x3f215040

# define DEVICE_BASE			0x3f000000
# define DEVICE_SIZE			0x01000000

/* The size of DDR RAM is 1GB. */
# define DRAM_BASE			0x00000000
# define DRAM_SIZE			0x40000000


# define PLAT_TRUSTED_DRAM_ID	1

/*
 *   - Non-Secure DRAM (remaining DRAM starting at DRAM_BASE)
 TEE-shared nonsec starts at 0x20000 ; with 4MB shared
 */

# define DRAM_NS_BASE			(0x08000000) 
# define DRAM_NS_SIZE			(4 * 1024 * 1024) 

# define DRAM_SEC_BASE		        (DRAM_NS_BASE + DRAM_NS_SIZE)	   /* ATF load address */
# define DRAM_SEC_SIZE			(28 * 1024 * 1024)
# define OPTEE_BASE                     (DRAM_SEC_BASE + 0x20000) /* set above ATF by 64k... */

# endif /* __RPI3_DEF_H__ */
