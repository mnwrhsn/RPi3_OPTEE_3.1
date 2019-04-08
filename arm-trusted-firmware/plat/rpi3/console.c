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

# include <console.h>

# include <stdio.h>

# include "rpi3_def.h"

static unsigned char *base;

void set_serial(unsigned long base_addr)
{
  base = (unsigned char *)base_addr;
}

void fix_serial(void)
{
  base = (unsigned char *)UART0_BASE;
}

int console_init(unsigned long base_addr,
		 unsigned int uart_clk, unsigned int baud_rate)
{
  base = (unsigned char *)base_addr;

  (void)uart_clk;
  (void)baud_rate;

  return(1);
}

int dummy_console_init(unsigned long base_addr,
		       unsigned int uart_clk, unsigned int baud_rate)
{
  base = (unsigned char *)UART0_BASE;
  (void)base_addr;
  (void)uart_clk;
  (void)baud_rate;

  return(1);
}

static unsigned int serial8250_in(int offset)
{
  unsigned char *base = (unsigned char *)(UART0_BASE);
  volatile unsigned int *reg = (unsigned int *)(base + offset);
  volatile unsigned int data;

  data = *reg;

  return(unsigned int)(data);
}

static void serial8250_out(int offset, int value)
{
  unsigned char *base = (unsigned char *)(UART0_BASE);
  volatile unsigned int *reg = (unsigned int *)(base + offset);

  *reg = (unsigned char)value;
}

# define UART_TX       0
# define UART_RX       0
# define UART_LSR      20
# define UART_LSR_TEMT 0x40
# define UART_LSR_THRE 0x20
# define UART_LSR_DR   0x01

# define TX_READY (1 << 5)

#define BOTH_EMPTY (TX_READY)

static void wait_for_xmitr(void)
{
  unsigned int status;
  
  for (;;) {
    status = serial8250_in(UART_LSR);
    if ((status & BOTH_EMPTY) == BOTH_EMPTY)
      return;
  }
}

static void serial_putc(int c)
{
  wait_for_xmitr();
  serial8250_out(UART_TX, c);
}

int console_putc(int c)
{
  serial_putc(c);
  if (c == '\n') {
    serial_putc('\r');
  }
  return(c);
}

int console_getc(void)
{
  while ((serial8250_in(UART_LSR) & UART_LSR_DR) == 0) {
    ;
  }
  return serial8250_in(UART_RX);
}
