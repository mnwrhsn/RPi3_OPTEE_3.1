/*
 * Copyright (C) 2013 Synopsys, Inc. (www.synopsys.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <common.h>
#include <dm.h>
#include <serial.h>

DECLARE_GLOBAL_DATA_PTR;

struct arc_serial_regs {
	unsigned int id0;
	unsigned int id1;
	unsigned int id2;
	unsigned int id3;
	unsigned int data;
	unsigned int status;
	unsigned int baudl;
	unsigned int baudh;
};


struct arc_serial_platdata {
	struct arc_serial_regs *reg;
	unsigned int uartclk;
};

/* Bit definitions of STATUS register */
#define UART_RXEMPTY		(1 << 5)
#define UART_OVERFLOW_ERR	(1 << 1)
#define UART_TXEMPTY		(1 << 7)

static int arc_serial_setbrg(struct udevice *dev, int baudrate)
{
	struct arc_serial_platdata *plat = dev->platdata;
	struct arc_serial_regs *const regs = plat->reg;
	int arc_console_baud = gd->cpu_clk / (baudrate * 4) - 1;

	writeb(arc_console_baud & 0xff, &regs->baudl);

#ifdef CONFIG_ARC
	/*
	 * UART ISS(Instruction Set simulator) emulation has a subtle bug:
	 * A existing value of Baudh = 0 is used as a indication to startup
	 * it's internal state machine.
	 * Thus if baudh is set to 0, 2 times, it chokes.
	 * This happens with BAUD=115200 and the formaula above
	 * Until that is fixed, when running on ISS, we will set baudh to !0
	 */
	if (gd->arch.running_on_hw)
		writeb((arc_console_baud & 0xff00) >> 8, &regs->baudh);
	else
		writeb(1, &regs->baudh);
#else
	writeb((arc_console_baud & 0xff00) >> 8, &regs->baudh);
#endif

	return 0;
}

static int arc_serial_putc(struct udevice *dev, const char c)
{
	struct arc_serial_platdata *plat = dev->platdata;
	struct arc_serial_regs *const regs = plat->reg;

	while (!(readb(&regs->status) & UART_TXEMPTY))
		;

	writeb(c, &regs->data);

	return 0;
}

static int arc_serial_tstc(struct arc_serial_regs *const regs)
{
	return !(readb(&regs->status) & UART_RXEMPTY);
}

static int arc_serial_pending(struct udevice *dev, bool input)
{
	struct arc_serial_platdata *plat = dev->platdata;
	struct arc_serial_regs *const regs = plat->reg;
	uint32_t status = readb(&regs->status);

	if (input)
		return status & UART_RXEMPTY ? 0 : 1;
	else
		return status & UART_TXEMPTY ? 0 : 1;
}

static int arc_serial_getc(struct udevice *dev)
{
	struct arc_serial_platdata *plat = dev->platdata;
	struct arc_serial_regs *const regs = plat->reg;

	while (!arc_serial_tstc(regs))
		;

	/* Check for overflow errors */
	if (readb(&regs->status) & UART_OVERFLOW_ERR)
		return 0;

	return readb(&regs->data) & 0xFF;
}

static int arc_serial_probe(struct udevice *dev)
{
	return 0;
}

static const struct dm_serial_ops arc_serial_ops = {
	.putc = arc_serial_putc,
	.pending = arc_serial_pending,
	.getc = arc_serial_getc,
	.setbrg = arc_serial_setbrg,
};

static const struct udevice_id arc_serial_ids[] = {
	{ .compatible = "snps,arc-uart" },
	{ }
};

static int arc_serial_ofdata_to_platdata(struct udevice *dev)
{
	struct arc_serial_platdata *plat = dev_get_platdata(dev);
	DECLARE_GLOBAL_DATA_PTR;

	plat->reg = (struct arc_serial_regs *)dev_get_addr(dev);
	plat->uartclk = fdtdec_get_int(gd->fdt_blob, dev->of_offset,
				       "clock-frequency", 0);

	return 0;
}

U_BOOT_DRIVER(serial_arc) = {
	.name	= "serial_arc",
	.id	= UCLASS_SERIAL,
	.of_match = arc_serial_ids,
	.ofdata_to_platdata = arc_serial_ofdata_to_platdata,
	.probe = arc_serial_probe,
	.ops	= &arc_serial_ops,
	.flags = DM_FLAG_PRE_RELOC,
};
