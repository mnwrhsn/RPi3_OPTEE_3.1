/*
 * BCM2835 SD host driver.
 *
 * Author:      Phil Elwell <phil@raspberrypi.org>
 *              Copyright (C) 2015-2016 Raspberry Pi (Trading) Ltd.
 *
 * Based on
 *  mmc-bcm2835.c by Gellert Weisz
 * which is, in turn, based on
 *  sdhci-bcm2708.c by Broadcom
 *  sdhci-bcm2835.c by Stephen Warren and Oleksandr Tymoshenko
 *  sdhci.c and sdhci-pci.c by Pierre Ossman
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define FIFO_READ_THRESHOLD     4
#define FIFO_WRITE_THRESHOLD    4
#define ALLOW_CMD23_READ        1
#define ALLOW_CMD23_WRITE       0
#define ENABLE_LOG              1
#define SDDATA_FIFO_PIO_BURST   8
#define CMD_DALLY_US            1

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/scatterlist.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/blkdev.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/of_dma.h>
#include <linux/time.h>
#include <linux/workqueue.h>
#include <soc/bcm2835/raspberrypi-firmware.h>

#define DRIVER_NAME "sdhost-bcm2835"

#define SDCMD  0x00 /* Command to SD card              - 16 R/W */
#define SDARG  0x04 /* Argument to SD card             - 32 R/W */
#define SDTOUT 0x08 /* Start value for timeout counter - 32 R/W */
#define SDCDIV 0x0c /* Start value for clock divider   - 11 R/W */
#define SDRSP0 0x10 /* SD card response (31:0)         - 32 R   */
#define SDRSP1 0x14 /* SD card response (63:32)        - 32 R   */
#define SDRSP2 0x18 /* SD card response (95:64)        - 32 R   */
#define SDRSP3 0x1c /* SD card response (127:96)       - 32 R   */
#define SDHSTS 0x20 /* SD host status                  - 11 R   */
#define SDVDD  0x30 /* SD card power control           -  1 R/W */
#define SDEDM  0x34 /* Emergency Debug Mode            - 13 R/W */
#define SDHCFG 0x38 /* Host configuration              -  2 R/W */
#define SDHBCT 0x3c /* Host byte count (debug)         - 32 R/W */
#define SDDATA 0x40 /* Data to/from SD card            - 32 R/W */
#define SDHBLC 0x50 /* Host block count (SDIO/SDHC)    -  9 R/W */

#define SDCMD_NEW_FLAG                  0x8000
#define SDCMD_FAIL_FLAG                 0x4000
#define SDCMD_BUSYWAIT                  0x800
#define SDCMD_NO_RESPONSE               0x400
#define SDCMD_LONG_RESPONSE             0x200
#define SDCMD_WRITE_CMD                 0x80
#define SDCMD_READ_CMD                  0x40
#define SDCMD_CMD_MASK                  0x3f

#define SDCDIV_MAX_CDIV                 0x7ff

#define SDHSTS_BUSY_IRPT                0x400
#define SDHSTS_BLOCK_IRPT               0x200
#define SDHSTS_SDIO_IRPT                0x100
#define SDHSTS_REW_TIME_OUT             0x80
#define SDHSTS_CMD_TIME_OUT             0x40
#define SDHSTS_CRC16_ERROR              0x20
#define SDHSTS_CRC7_ERROR               0x10
#define SDHSTS_FIFO_ERROR               0x08
/* Reserved */
/* Reserved */
#define SDHSTS_DATA_FLAG                0x01

#define SDHSTS_TRANSFER_ERROR_MASK      (SDHSTS_CRC7_ERROR|SDHSTS_CRC16_ERROR|SDHSTS_REW_TIME_OUT|SDHSTS_FIFO_ERROR)
#define SDHSTS_ERROR_MASK               (SDHSTS_CMD_TIME_OUT|SDHSTS_TRANSFER_ERROR_MASK)

#define SDHCFG_BUSY_IRPT_EN     (1<<10)
#define SDHCFG_BLOCK_IRPT_EN    (1<<8)
#define SDHCFG_SDIO_IRPT_EN     (1<<5)
#define SDHCFG_DATA_IRPT_EN     (1<<4)
#define SDHCFG_SLOW_CARD        (1<<3)
#define SDHCFG_WIDE_EXT_BUS     (1<<2)
#define SDHCFG_WIDE_INT_BUS     (1<<1)
#define SDHCFG_REL_CMD_LINE     (1<<0)

#define SDEDM_FORCE_DATA_MODE   (1<<19)
#define SDEDM_CLOCK_PULSE       (1<<20)
#define SDEDM_BYPASS            (1<<21)

#define SDEDM_WRITE_THRESHOLD_SHIFT 9
#define SDEDM_READ_THRESHOLD_SHIFT 14
#define SDEDM_THRESHOLD_MASK     0x1f

#define SDEDM_FSM_MASK           0xf
#define SDEDM_FSM_IDENTMODE      0x0
#define SDEDM_FSM_DATAMODE       0x1
#define SDEDM_FSM_READDATA       0x2
#define SDEDM_FSM_WRITEDATA      0x3
#define SDEDM_FSM_READWAIT       0x4
#define SDEDM_FSM_READCRC        0x5
#define SDEDM_FSM_WRITECRC       0x6
#define SDEDM_FSM_WRITEWAIT1     0x7
#define SDEDM_FSM_POWERDOWN      0x8
#define SDEDM_FSM_POWERUP        0x9
#define SDEDM_FSM_WRITESTART1    0xa
#define SDEDM_FSM_WRITESTART2    0xb
#define SDEDM_FSM_GENPULSES      0xc
#define SDEDM_FSM_WRITEWAIT2     0xd
#define SDEDM_FSM_STARTPOWDOWN   0xf

#define SDDATA_FIFO_WORDS        16

#define USE_CMD23_FLAGS          ((ALLOW_CMD23_READ * MMC_DATA_READ) | \
				  (ALLOW_CMD23_WRITE * MMC_DATA_WRITE))

#define MHZ 1000000


struct bcm2835_host {
	spinlock_t		lock;

	void __iomem		*ioaddr;
	u32			bus_addr;

	struct mmc_host		*mmc;

	u32			pio_timeout;	/* In jiffies */

	int			clock;		/* Current clock speed */

	bool			slow_card;	/* Force 11-bit divisor */

	unsigned int		max_clk;	/* Max possible freq */

	struct tasklet_struct	finish_tasklet;	/* Tasklet structures */

	struct work_struct	cmd_wait_wq;	/* Workqueue function */

	struct timer_list	timer;		/* Timer for timeouts */

	struct sg_mapping_iter	sg_miter;	/* SG state for PIO */
	unsigned int		blocks;		/* remaining PIO blocks */

	int			irq;		/* Device IRQ */

	u32			cmd_quick_poll_retries;
	u32			ns_per_fifo_word;

	/* cached registers */
	u32			hcfg;
	u32			cdiv;

	struct mmc_request		*mrq;			/* Current request */
	struct mmc_command		*cmd;			/* Current command */
	struct mmc_data			*data;			/* Current data request */
	unsigned int			data_complete:1;	/* Data finished before cmd */

	unsigned int			flush_fifo:1;		/* Drain the fifo when finishing */

	unsigned int			use_busy:1;		/* Wait for busy interrupt */

	unsigned int			use_sbc:1;		/* Send CMD23 */

	unsigned int			debug:1;		/* Enable debug output */
	unsigned int			firmware_sets_cdiv:1;	/* Let the firmware manage the clock */
	unsigned int			reset_clock:1;		/* Reset the clock fore the next request */

	/*DMA part*/
	struct dma_chan			*dma_chan_rxtx;		/* DMA channel for reads and writes */
	struct dma_chan			*dma_chan;		/* Channel in use */
	struct dma_slave_config		dma_cfg_rx;
	struct dma_slave_config		dma_cfg_tx;
	struct dma_async_tx_descriptor	*dma_desc;
	u32				dma_dir;
	u32				drain_words;
	struct page 			*drain_page;
	u32				drain_offset;

	bool				allow_dma;
	bool				use_dma;
	/*end of DMA part*/

	int				max_delay;	/* maximum length of time spent waiting */
	struct timeval			stop_time;	/* when the last stop was issued */
	u32				delay_after_stop; /* minimum time between stop and subsequent data transfer */
	u32				delay_after_this_stop; /* minimum time between this stop and subsequent data transfer */
	u32				user_overclock_50; /* User's preferred frequency to use when 50MHz is requested (in MHz) */
	u32				overclock_50;	/* frequency to use when 50MHz is requested (in MHz) */
	u32				overclock;	/* Current frequency if overclocked, else zero */
	u32				pio_limit;	/* Maximum block count for PIO (0 = always DMA) */

	u32				sectors;	/* Cached card size in sectors */
};

#if ENABLE_LOG

struct log_entry_struct {
	char event[4];
	u32 timestamp;
	u32 param1;
	u32 param2;
};

typedef struct log_entry_struct LOG_ENTRY_T;

LOG_ENTRY_T *sdhost_log_buf;
dma_addr_t sdhost_log_addr;
static u32 sdhost_log_idx;
static spinlock_t log_lock;
static void __iomem *timer_base;

#define LOG_ENTRIES (256*1)
#define LOG_SIZE (sizeof(LOG_ENTRY_T)*LOG_ENTRIES)

static void log_init(u32 bus_to_phys)
{
	spin_lock_init(&log_lock);
	sdhost_log_buf = dma_zalloc_coherent(NULL, LOG_SIZE, &sdhost_log_addr,
					     GFP_KERNEL);
	if (sdhost_log_buf) {
		pr_info("sdhost: log_buf @ %p (%x)\n",
			sdhost_log_buf, sdhost_log_addr);
		timer_base = ioremap_nocache(bus_to_phys + 0x7e003000, SZ_4K);
		if (!timer_base)
			pr_err("sdhost: failed to remap timer\n");
	}
	else
		pr_err("sdhost: failed to allocate log buf\n");
}

static void log_event_impl(const char *event, u32 param1, u32 param2)
{
	if (sdhost_log_buf) {
		LOG_ENTRY_T *entry;
		unsigned long flags;

		spin_lock_irqsave(&log_lock, flags);

		entry = sdhost_log_buf + sdhost_log_idx;
		memcpy(entry->event, event, 4);
		entry->timestamp = (readl(timer_base + 4) & 0x3fffffff) +
			(smp_processor_id()<<30);
		entry->param1 = param1;
		entry->param2 = param2;
		sdhost_log_idx = (sdhost_log_idx + 1) % LOG_ENTRIES;

		spin_unlock_irqrestore(&log_lock, flags);
	}
}

static void log_dump(void)
{
	if (sdhost_log_buf) {
		LOG_ENTRY_T *entry;
		unsigned long flags;
		int idx;

		spin_lock_irqsave(&log_lock, flags);

		idx = sdhost_log_idx;
		do {
			entry = sdhost_log_buf + idx;
			if (entry->event[0] != '\0')
				pr_info("[%08x] %.4s %x %x\n",
				       entry->timestamp,
				       entry->event,
				       entry->param1,
				       entry->param2);
			idx = (idx + 1) % LOG_ENTRIES;
		} while (idx != sdhost_log_idx);

		spin_unlock_irqrestore(&log_lock, flags);
	}
}

#define log_event(event, param1, param2) log_event_impl(event, param1, param2)

#else

#define log_init(x) (void)0
#define log_event(event, param1, param2) (void)0
#define log_dump() (void)0

#endif

static inline void bcm2835_sdhost_write(struct bcm2835_host *host, u32 val, int reg)
{
	writel(val, host->ioaddr + reg);
}

static inline u32 bcm2835_sdhost_read(struct bcm2835_host *host, int reg)
{
	return readl(host->ioaddr + reg);
}

static inline u32 bcm2835_sdhost_read_relaxed(struct bcm2835_host *host, int reg)
{
	return readl_relaxed(host->ioaddr + reg);
}

static void bcm2835_sdhost_dumpcmd(struct bcm2835_host *host,
				   struct mmc_command *cmd,
				   const char *label)
{
	if (cmd)
		pr_info("%s:%c%s op %d arg 0x%x flags 0x%x - resp %08x %08x %08x %08x, err %d\n",
			mmc_hostname(host->mmc),
			(cmd == host->cmd) ? '>' : ' ',
			label, cmd->opcode, cmd->arg, cmd->flags,
			cmd->resp[0], cmd->resp[1], cmd->resp[2], cmd->resp[3],
			cmd->error);
}

static void bcm2835_sdhost_dumpregs(struct bcm2835_host *host)
{
	if (host->mrq)
	{
		bcm2835_sdhost_dumpcmd(host, host->mrq->sbc, "sbc");
		bcm2835_sdhost_dumpcmd(host, host->mrq->cmd, "cmd");
		if (host->mrq->data)
			pr_info("%s: data blocks %x blksz %x - err %d\n",
			       mmc_hostname(host->mmc),
			       host->mrq->data->blocks,
			       host->mrq->data->blksz,
			       host->mrq->data->error);
		bcm2835_sdhost_dumpcmd(host, host->mrq->stop, "stop");
	}

	pr_info("%s: =========== REGISTER DUMP ===========\n",
		mmc_hostname(host->mmc));

	pr_info("%s: SDCMD  0x%08x\n",
		mmc_hostname(host->mmc),
		bcm2835_sdhost_read(host, SDCMD));
	pr_info("%s: SDARG  0x%08x\n",
		mmc_hostname(host->mmc),
		bcm2835_sdhost_read(host, SDARG));
	pr_info("%s: SDTOUT 0x%08x\n",
		mmc_hostname(host->mmc),
		bcm2835_sdhost_read(host, SDTOUT));
	pr_info("%s: SDCDIV 0x%08x\n",
		mmc_hostname(host->mmc),
		bcm2835_sdhost_read(host, SDCDIV));
	pr_info("%s: SDRSP0 0x%08x\n",
		mmc_hostname(host->mmc),
		bcm2835_sdhost_read(host, SDRSP0));
	pr_info("%s: SDRSP1 0x%08x\n",
		mmc_hostname(host->mmc),
		bcm2835_sdhost_read(host, SDRSP1));
	pr_info("%s: SDRSP2 0x%08x\n",
		mmc_hostname(host->mmc),
		bcm2835_sdhost_read(host, SDRSP2));
	pr_err("%s: SDRSP3 0x%08x\n",
		mmc_hostname(host->mmc),
		bcm2835_sdhost_read(host, SDRSP3));
	pr_info("%s: SDHSTS 0x%08x\n",
		mmc_hostname(host->mmc),
		bcm2835_sdhost_read(host, SDHSTS));
	pr_info("%s: SDVDD  0x%08x\n",
		mmc_hostname(host->mmc),
		bcm2835_sdhost_read(host, SDVDD));
	pr_info("%s: SDEDM  0x%08x\n",
		mmc_hostname(host->mmc),
		bcm2835_sdhost_read(host, SDEDM));
	pr_info("%s: SDHCFG 0x%08x\n",
		mmc_hostname(host->mmc),
		bcm2835_sdhost_read(host, SDHCFG));
	pr_info("%s: SDHBCT 0x%08x\n",
		mmc_hostname(host->mmc),
		bcm2835_sdhost_read(host, SDHBCT));
	pr_info("%s: SDHBLC 0x%08x\n",
		mmc_hostname(host->mmc),
		bcm2835_sdhost_read(host, SDHBLC));

	pr_info("%s: ===========================================\n",
		mmc_hostname(host->mmc));
}

static void bcm2835_sdhost_set_power(struct bcm2835_host *host, bool on)
{
	bcm2835_sdhost_write(host, on ? 1 : 0, SDVDD);
}

static void bcm2835_sdhost_reset_internal(struct bcm2835_host *host)
{
	u32 temp;

	if (host->debug)
		pr_info("%s: reset\n", mmc_hostname(host->mmc));

	bcm2835_sdhost_set_power(host, false);

	bcm2835_sdhost_write(host, 0, SDCMD);
	bcm2835_sdhost_write(host, 0, SDARG);
	bcm2835_sdhost_write(host, 0xf00000, SDTOUT);
	bcm2835_sdhost_write(host, 0, SDCDIV);
	bcm2835_sdhost_write(host, 0x7f8, SDHSTS); /* Write 1s to clear */
	bcm2835_sdhost_write(host, 0, SDHCFG);
	bcm2835_sdhost_write(host, 0, SDHBCT);
	bcm2835_sdhost_write(host, 0, SDHBLC);

	/* Limit fifo usage due to silicon bug */
	temp = bcm2835_sdhost_read(host, SDEDM);
	temp &= ~((SDEDM_THRESHOLD_MASK<<SDEDM_READ_THRESHOLD_SHIFT) |
		  (SDEDM_THRESHOLD_MASK<<SDEDM_WRITE_THRESHOLD_SHIFT));
	temp |= (FIFO_READ_THRESHOLD << SDEDM_READ_THRESHOLD_SHIFT) |
		(FIFO_WRITE_THRESHOLD << SDEDM_WRITE_THRESHOLD_SHIFT);
	bcm2835_sdhost_write(host, temp, SDEDM);
	mdelay(10);
	bcm2835_sdhost_set_power(host, true);
	mdelay(10);
	host->clock = 0;
	host->sectors = 0;
	bcm2835_sdhost_write(host, host->hcfg, SDHCFG);
	bcm2835_sdhost_write(host, SDCDIV_MAX_CDIV, SDCDIV);
	mmiowb();
}

static void bcm2835_sdhost_reset(struct mmc_host *mmc)
{
	struct bcm2835_host *host = mmc_priv(mmc);
	unsigned long flags;
	spin_lock_irqsave(&host->lock, flags);
	log_event("RST<", 0, 0);

	bcm2835_sdhost_reset_internal(host);

	spin_unlock_irqrestore(&host->lock, flags);
}

static void bcm2835_sdhost_set_ios(struct mmc_host *mmc, struct mmc_ios *ios);

static void bcm2835_sdhost_init(struct bcm2835_host *host, int soft)
{
	pr_debug("bcm2835_sdhost_init(%d)\n", soft);

	/* Set interrupt enables */
	host->hcfg = SDHCFG_BUSY_IRPT_EN;

	bcm2835_sdhost_reset_internal(host);

	if (soft) {
		/* force clock reconfiguration */
		host->clock = 0;
		bcm2835_sdhost_set_ios(host->mmc, &host->mmc->ios);
	}
}

static void bcm2835_sdhost_wait_transfer_complete(struct bcm2835_host *host)
{
	int timediff;
	u32 alternate_idle;
	u32 edm;

	alternate_idle = (host->mrq->data->flags & MMC_DATA_READ) ?
		SDEDM_FSM_READWAIT : SDEDM_FSM_WRITESTART1;

	edm = bcm2835_sdhost_read(host, SDEDM);

	log_event("WTC<", edm, 0);

	timediff = 0;

	while (1) {
		u32 fsm = edm & SDEDM_FSM_MASK;
		if ((fsm == SDEDM_FSM_IDENTMODE) ||
		    (fsm == SDEDM_FSM_DATAMODE))
			break;
		if (fsm == alternate_idle) {
			bcm2835_sdhost_write(host,
					     edm | SDEDM_FORCE_DATA_MODE,
					     SDEDM);
			break;
		}

		timediff++;
		if (timediff == 100000) {
			pr_err("%s: wait_transfer_complete - still waiting after %d retries\n",
			       mmc_hostname(host->mmc),
			       timediff);
			log_dump();
			bcm2835_sdhost_dumpregs(host);
			host->mrq->data->error = -ETIMEDOUT;
			log_event("WTC!", edm, 0);
			return;
		}
		cpu_relax();
		edm = bcm2835_sdhost_read(host, SDEDM);
	}
	log_event("WTC>", edm, 0);
}

static void bcm2835_sdhost_finish_data(struct bcm2835_host *host);

static void bcm2835_sdhost_dma_complete(void *param)
{
	struct bcm2835_host *host = param;
	struct mmc_data *data = host->data;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	log_event("DMA<", (u32)host->data, bcm2835_sdhost_read(host, SDHSTS));
	log_event("DMA ", bcm2835_sdhost_read(host, SDCMD),
		  bcm2835_sdhost_read(host, SDEDM));

	if (host->dma_chan) {
		dma_unmap_sg(host->dma_chan->device->dev,
			     data->sg, data->sg_len,
			     host->dma_dir);

		host->dma_chan = NULL;
	}

	if (host->drain_words) {
		void *page;
		u32 *buf;

		page = kmap_atomic(host->drain_page);
		buf = page + host->drain_offset;

		while (host->drain_words) {
			u32 edm = bcm2835_sdhost_read(host, SDEDM);
			if ((edm >> 4) & 0x1f)
				*(buf++) = bcm2835_sdhost_read(host,
							       SDDATA);
			host->drain_words--;
		}

		kunmap_atomic(page);
	}

	bcm2835_sdhost_finish_data(host);

	log_event("DMA>", (u32)host->data, 0);
	spin_unlock_irqrestore(&host->lock, flags);
}

static void bcm2835_sdhost_read_block_pio(struct bcm2835_host *host)
{
	unsigned long flags;
	size_t blksize, len;
	u32 *buf;
	unsigned long wait_max;

	blksize = host->data->blksz;

	wait_max = jiffies + msecs_to_jiffies(host->pio_timeout);

	local_irq_save(flags);

	while (blksize) {
		int copy_words;
		u32 hsts = 0;

		if (!sg_miter_next(&host->sg_miter)) {
			host->data->error = -EINVAL;
			break;
		}

		len = min(host->sg_miter.length, blksize);
		if (len % 4) {
			host->data->error = -EINVAL;
			break;
		}

		blksize -= len;
		host->sg_miter.consumed = len;

		buf = (u32 *)host->sg_miter.addr;

		copy_words = len/4;

		while (copy_words) {
			int burst_words, words;
			u32 edm;

			burst_words = SDDATA_FIFO_PIO_BURST;
			if (burst_words > copy_words)
				burst_words = copy_words;
			edm = bcm2835_sdhost_read(host, SDEDM);
			words = ((edm >> 4) & 0x1f);

			if (words < burst_words) {
				int fsm_state = (edm & SDEDM_FSM_MASK);
				if ((fsm_state != SDEDM_FSM_READDATA) &&
				    (fsm_state != SDEDM_FSM_READWAIT) &&
				    (fsm_state != SDEDM_FSM_READCRC)) {
					hsts = bcm2835_sdhost_read(host,
								   SDHSTS);
					pr_info("%s: fsm %x, hsts %x\n",
					       mmc_hostname(host->mmc),
					       fsm_state, hsts);
					if (hsts & SDHSTS_ERROR_MASK)
						break;
				}

				if (time_after(jiffies, wait_max)) {
					pr_err("%s: PIO read timeout - EDM %x\n",
					       mmc_hostname(host->mmc),
					       edm);
					hsts = SDHSTS_REW_TIME_OUT;
					break;
				}
				ndelay((burst_words - words) *
				       host->ns_per_fifo_word);
				continue;
			} else if (words > copy_words) {
				words = copy_words;
			}

			copy_words -= words;

			while (words) {
				*(buf++) = bcm2835_sdhost_read(host, SDDATA);
				words--;
			}
		}

		if (hsts & SDHSTS_ERROR_MASK)
			break;
	}

	sg_miter_stop(&host->sg_miter);

	local_irq_restore(flags);
}

static void bcm2835_sdhost_write_block_pio(struct bcm2835_host *host)
{
	unsigned long flags;
	size_t blksize, len;
	u32 *buf;
	unsigned long wait_max;

	blksize = host->data->blksz;

	wait_max = jiffies + msecs_to_jiffies(host->pio_timeout);

	local_irq_save(flags);

	while (blksize) {
		int copy_words;
		u32 hsts = 0;

		if (!sg_miter_next(&host->sg_miter)) {
			host->data->error = -EINVAL;
			break;
		}

		len = min(host->sg_miter.length, blksize);
		if (len % 4) {
			host->data->error = -EINVAL;
			break;
		}

		blksize -= len;
		host->sg_miter.consumed = len;

		buf = (u32 *)host->sg_miter.addr;

		copy_words = len/4;

		while (copy_words) {
			int burst_words, words;
			u32 edm;

			burst_words = SDDATA_FIFO_PIO_BURST;
			if (burst_words > copy_words)
				burst_words = copy_words;
			edm = bcm2835_sdhost_read(host, SDEDM);
			words = SDDATA_FIFO_WORDS - ((edm >> 4) & 0x1f);

			if (words < burst_words) {
				int fsm_state = (edm & SDEDM_FSM_MASK);
				if ((fsm_state != SDEDM_FSM_WRITEDATA) &&
				    (fsm_state != SDEDM_FSM_WRITESTART1) &&
				    (fsm_state != SDEDM_FSM_WRITESTART2)) {
					hsts = bcm2835_sdhost_read(host,
								   SDHSTS);
					pr_info("%s: fsm %x, hsts %x\n",
					       mmc_hostname(host->mmc),
					       fsm_state, hsts);
					if (hsts & SDHSTS_ERROR_MASK)
						break;
				}

				if (time_after(jiffies, wait_max)) {
					pr_err("%s: PIO write timeout - EDM %x\n",
					       mmc_hostname(host->mmc),
					       edm);
					hsts = SDHSTS_REW_TIME_OUT;
					break;
				}
				ndelay((burst_words - words) *
				       host->ns_per_fifo_word);
				continue;
			} else if (words > copy_words) {
				words = copy_words;
			}

			copy_words -= words;

			while (words) {
				bcm2835_sdhost_write(host, *(buf++), SDDATA);
				words--;
			}
		}

		if (hsts & SDHSTS_ERROR_MASK)
			break;
	}

	sg_miter_stop(&host->sg_miter);

	local_irq_restore(flags);
}

static void bcm2835_sdhost_transfer_pio(struct bcm2835_host *host)
{
	u32 sdhsts;
	bool is_read;
	BUG_ON(!host->data);
	log_event("XFP<", (u32)host->data, host->blocks);

	is_read = (host->data->flags & MMC_DATA_READ) != 0;
	if (is_read)
		bcm2835_sdhost_read_block_pio(host);
	else
		bcm2835_sdhost_write_block_pio(host);

	sdhsts = bcm2835_sdhost_read(host, SDHSTS);
	if (sdhsts & (SDHSTS_CRC16_ERROR |
		      SDHSTS_CRC7_ERROR |
		      SDHSTS_FIFO_ERROR)) {
		pr_err("%s: %s transfer error - HSTS %x\n",
		       mmc_hostname(host->mmc),
		       is_read ? "read" : "write",
		       sdhsts);
		host->data->error = -EILSEQ;
	} else if ((sdhsts & (SDHSTS_CMD_TIME_OUT |
			      SDHSTS_REW_TIME_OUT))) {
		pr_err("%s: %s timeout error - HSTS %x\n",
		       mmc_hostname(host->mmc),
		       is_read ? "read" : "write",
		       sdhsts);
		host->data->error = -ETIMEDOUT;
	}
	log_event("XFP>", (u32)host->data, host->blocks);
}

static void bcm2835_sdhost_prepare_dma(struct bcm2835_host *host,
	struct mmc_data *data)
{
	int len, dir_data, dir_slave;
	struct dma_async_tx_descriptor *desc = NULL;
	struct dma_chan *dma_chan;

	log_event("PRD<", (u32)data, 0);
	pr_debug("bcm2835_sdhost_prepare_dma()\n");

	dma_chan = host->dma_chan_rxtx;
	if (data->flags & MMC_DATA_READ) {
		dir_data = DMA_FROM_DEVICE;
		dir_slave = DMA_DEV_TO_MEM;
	} else {
		dir_data = DMA_TO_DEVICE;
		dir_slave = DMA_MEM_TO_DEV;
	}
	log_event("PRD1", (u32)dma_chan, 0);

	BUG_ON(!dma_chan->device);
	BUG_ON(!dma_chan->device->dev);
	BUG_ON(!data->sg);

	/* The block doesn't manage the FIFO DREQs properly for multi-block
	   transfers, so don't attempt to DMA the final few words.
	   Unfortunately this requires the final sg entry to be trimmed.
	   N.B. This code demands that the overspill is contained in
	   a single sg entry.
	*/

	host->drain_words = 0;
	if ((data->blocks > 1) && (dir_data == DMA_FROM_DEVICE)) {
		struct scatterlist *sg;
		u32 len;
		int i;

		len = min((u32)(FIFO_READ_THRESHOLD - 1) * 4,
			  (u32)data->blocks * data->blksz);

		for_each_sg(data->sg, sg, data->sg_len, i) {
			if (sg_is_last(sg)) {
				BUG_ON(sg->length < len);
				sg->length -= len;
				host->drain_page = (struct page *)sg->page_link;
				host->drain_offset = sg->offset + sg->length;
			}
		}
		host->drain_words = len/4;
	}

	/* The parameters have already been validated, so this will not fail */
	(void)dmaengine_slave_config(dma_chan,
				     (dir_data == DMA_FROM_DEVICE) ?
				     &host->dma_cfg_rx :
				     &host->dma_cfg_tx);

	len = dma_map_sg(dma_chan->device->dev, data->sg, data->sg_len,
			 dir_data);

	log_event("PRD2", len, 0);
	if (len > 0)
		desc = dmaengine_prep_slave_sg(dma_chan, data->sg,
					       len, dir_slave,
					       DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	log_event("PRD3", (u32)desc, 0);

	if (desc) {
		desc->callback = bcm2835_sdhost_dma_complete;
		desc->callback_param = host;
		host->dma_desc = desc;
		host->dma_chan = dma_chan;
		host->dma_dir = dir_data;
	}
	log_event("PDM>", (u32)data, 0);
}

static void bcm2835_sdhost_start_dma(struct bcm2835_host *host)
{
	log_event("SDMA", (u32)host->data, (u32)host->dma_chan);
	dmaengine_submit(host->dma_desc);
	dma_async_issue_pending(host->dma_chan);
}

static void bcm2835_sdhost_set_transfer_irqs(struct bcm2835_host *host)
{
	u32 all_irqs = SDHCFG_DATA_IRPT_EN | SDHCFG_BLOCK_IRPT_EN |
		SDHCFG_BUSY_IRPT_EN;
	if (host->dma_desc)
		host->hcfg = (host->hcfg & ~all_irqs) |
			SDHCFG_BUSY_IRPT_EN;
	else
		host->hcfg = (host->hcfg & ~all_irqs) |
			SDHCFG_DATA_IRPT_EN |
			SDHCFG_BUSY_IRPT_EN;

	bcm2835_sdhost_write(host, host->hcfg, SDHCFG);
}

static void bcm2835_sdhost_prepare_data(struct bcm2835_host *host, struct mmc_command *cmd)
{
	struct mmc_data *data = cmd->data;

	WARN_ON(host->data);

	host->data = data;
	if (!data)
		return;

	/* Sanity checks */
	BUG_ON(data->blksz * data->blocks > 524288);
	BUG_ON(data->blksz > host->mmc->max_blk_size);
	BUG_ON(data->blocks > 65535);

	host->data_complete = 0;
	host->flush_fifo = 0;
	host->data->bytes_xfered = 0;

	if (!host->sectors && host->mmc->card) {
		struct mmc_card *card = host->mmc->card;
		if (!mmc_card_sd(card) && mmc_card_blockaddr(card)) {
			/*
			 * The EXT_CSD sector count is in number of 512 byte
			 * sectors.
			 */
			host->sectors = card->ext_csd.sectors;
		} else {
			/*
			 * The CSD capacity field is in units of read_blkbits.
			 * set_capacity takes units of 512 bytes.
			 */
			host->sectors = card->csd.capacity <<
				(card->csd.read_blkbits - 9);
		}
	}

	if (!host->dma_desc) {
		/* Use PIO */
		int flags = SG_MITER_ATOMIC;

		if (data->flags & MMC_DATA_READ)
			flags |= SG_MITER_TO_SG;
		else
			flags |= SG_MITER_FROM_SG;
		sg_miter_start(&host->sg_miter, data->sg, data->sg_len, flags);
		host->blocks = data->blocks;
	}

	bcm2835_sdhost_set_transfer_irqs(host);

	bcm2835_sdhost_write(host, data->blksz, SDHBCT);
	bcm2835_sdhost_write(host, data->blocks, SDHBLC);

	BUG_ON(!host->data);
}

bool bcm2835_sdhost_send_command(struct bcm2835_host *host,
				 struct mmc_command *cmd)
{
	u32 sdcmd, sdhsts;
	unsigned long timeout;
	int delay;

	WARN_ON(host->cmd);
	log_event("CMD<", cmd->opcode, cmd->arg);

	if (cmd->data)
		pr_debug("%s: send_command %d 0x%x "
			 "(flags 0x%x) - %s %d*%d\n",
			 mmc_hostname(host->mmc),
			 cmd->opcode, cmd->arg, cmd->flags,
			 (cmd->data->flags & MMC_DATA_READ) ?
			 "read" : "write", cmd->data->blocks,
			 cmd->data->blksz);
	else
		pr_debug("%s: send_command %d 0x%x (flags 0x%x)\n",
			 mmc_hostname(host->mmc),
			 cmd->opcode, cmd->arg, cmd->flags);

	/* Wait max 100 ms */
	timeout = 10000;

	while (bcm2835_sdhost_read(host, SDCMD) & SDCMD_NEW_FLAG) {
		if (timeout == 0) {
			pr_warn("%s: previous command never completed.\n",
				mmc_hostname(host->mmc));
			if (host->debug)
				bcm2835_sdhost_dumpregs(host);
			cmd->error = -EILSEQ;
			tasklet_schedule(&host->finish_tasklet);
			return false;
		}
		timeout--;
		udelay(10);
	}

	delay = (10000 - timeout)/100;
	if (delay > host->max_delay) {
		host->max_delay = delay;
		pr_warning("%s: controller hung for %d ms\n",
			   mmc_hostname(host->mmc),
			   host->max_delay);
	}

	timeout = jiffies;
	if (!cmd->data && cmd->busy_timeout > 9000)
		timeout += DIV_ROUND_UP(cmd->busy_timeout, 1000) * HZ + HZ;
	else
		timeout += 10 * HZ;
	mod_timer(&host->timer, timeout);

	host->cmd = cmd;

	/* Clear any error flags */
	sdhsts = bcm2835_sdhost_read(host, SDHSTS);
	if (sdhsts & SDHSTS_ERROR_MASK)
		bcm2835_sdhost_write(host, sdhsts, SDHSTS);

	if ((cmd->flags & MMC_RSP_136) && (cmd->flags & MMC_RSP_BUSY)) {
		pr_err("%s: unsupported response type!\n",
			mmc_hostname(host->mmc));
		cmd->error = -EINVAL;
		tasklet_schedule(&host->finish_tasklet);
		return false;
	}

	bcm2835_sdhost_prepare_data(host, cmd);

	bcm2835_sdhost_write(host, cmd->arg, SDARG);

	sdcmd = cmd->opcode & SDCMD_CMD_MASK;

	host->use_busy = 0;
	if (!(cmd->flags & MMC_RSP_PRESENT)) {
		sdcmd |= SDCMD_NO_RESPONSE;
	} else {
		if (cmd->flags & MMC_RSP_136)
			sdcmd |= SDCMD_LONG_RESPONSE;
		if (cmd->flags & MMC_RSP_BUSY) {
			sdcmd |= SDCMD_BUSYWAIT;
			host->use_busy = 1;
		}
	}

	if (cmd->data) {
		log_event("CMDD", cmd->data->blocks, cmd->data->blksz);
		if (host->delay_after_this_stop) {
			struct timeval now;
			int time_since_stop;
			do_gettimeofday(&now);
			time_since_stop = (now.tv_sec - host->stop_time.tv_sec);
			if (time_since_stop < 2) {
				/* Possibly less than one second */
				time_since_stop = time_since_stop * 1000000 +
					(now.tv_usec - host->stop_time.tv_usec);
				if (time_since_stop <
				    host->delay_after_this_stop)
					udelay(host->delay_after_this_stop -
					       time_since_stop);
			}
		}

		host->delay_after_this_stop = host->delay_after_stop;
		if ((cmd->data->flags & MMC_DATA_READ) && !host->use_sbc) {
			/* See if read crosses one of the hazardous sectors */
			u32 first_blk, last_blk;

			/* Intentionally include the following sector because
			   without CMD23/SBC the read may run on. */
			first_blk = host->mrq->cmd->arg;
			last_blk = first_blk + cmd->data->blocks;

			if (((last_blk >= (host->sectors - 64)) &&
			     (first_blk <= (host->sectors - 64))) ||
			    ((last_blk >= (host->sectors - 32)) &&
			     (first_blk <= (host->sectors - 32)))) {
				host->delay_after_this_stop =
					max(250u, host->delay_after_stop);
			}
		}

		if (cmd->data->flags & MMC_DATA_WRITE)
			sdcmd |= SDCMD_WRITE_CMD;
		if (cmd->data->flags & MMC_DATA_READ)
			sdcmd |= SDCMD_READ_CMD;
	}

	bcm2835_sdhost_write(host, sdcmd | SDCMD_NEW_FLAG, SDCMD);

	return true;
}

static void bcm2835_sdhost_finish_command(struct bcm2835_host *host,
					  unsigned long *irq_flags);
static void bcm2835_sdhost_transfer_complete(struct bcm2835_host *host);

static void bcm2835_sdhost_finish_data(struct bcm2835_host *host)
{
	struct mmc_data *data;

	data = host->data;
	BUG_ON(!data);

	log_event("FDA<", (u32)host->mrq, (u32)host->cmd);
	pr_debug("finish_data(error %d, stop %d, sbc %d)\n",
	       data->error, data->stop ? 1 : 0,
	       host->mrq->sbc ? 1 : 0);

	host->hcfg &= ~(SDHCFG_DATA_IRPT_EN | SDHCFG_BLOCK_IRPT_EN);
	bcm2835_sdhost_write(host, host->hcfg, SDHCFG);

	data->bytes_xfered = data->error ? 0 : (data->blksz * data->blocks);

	host->data_complete = 1;

	if (host->cmd) {
		/*
		 * Data managed to finish before the
		 * command completed. Make sure we do
		 * things in the proper order.
		 */
		pr_debug("Finished early - HSTS %x\n",
			 bcm2835_sdhost_read(host, SDHSTS));
	}
	else
		bcm2835_sdhost_transfer_complete(host);
	log_event("FDA>", (u32)host->mrq, (u32)host->cmd);
}

static void bcm2835_sdhost_transfer_complete(struct bcm2835_host *host)
{
	struct mmc_data *data;

	BUG_ON(host->cmd);
	BUG_ON(!host->data);
	BUG_ON(!host->data_complete);

	data = host->data;
	host->data = NULL;

	log_event("TCM<", (u32)data, data->error);
	pr_debug("transfer_complete(error %d, stop %d)\n",
	       data->error, data->stop ? 1 : 0);

	/*
	 * Need to send CMD12 if -
	 * a) open-ended multiblock transfer (no CMD23)
	 * b) error in multiblock transfer
	 */
	if (host->mrq->stop && (data->error || !host->use_sbc)) {
		if (bcm2835_sdhost_send_command(host, host->mrq->stop)) {
			/* No busy, so poll for completion */
			if (!host->use_busy)
				bcm2835_sdhost_finish_command(host, NULL);

			if (host->delay_after_this_stop)
				do_gettimeofday(&host->stop_time);
		}
	} else {
		bcm2835_sdhost_wait_transfer_complete(host);
		tasklet_schedule(&host->finish_tasklet);
	}
	log_event("TCM>", (u32)data, 0);
}

/* If irq_flags is valid, the caller is in a thread context and is allowed
   to sleep */
static void bcm2835_sdhost_finish_command(struct bcm2835_host *host,
					  unsigned long *irq_flags)
{
	u32 sdcmd;
	u32 retries;
#ifdef DEBUG
	struct timeval before, after;
	int timediff = 0;
#endif

	log_event("FCM<", (u32)host->mrq, (u32)host->cmd);
	pr_debug("finish_command(%x)\n", bcm2835_sdhost_read(host, SDCMD));

	BUG_ON(!host->cmd || !host->mrq);

	/* Poll quickly at first */

	retries = host->cmd_quick_poll_retries;
	if (!retries) {
		/* Work out how many polls take 1us by timing 10us */
		struct timeval start, now;
		int us_diff;

		retries = 1;
		do {
			int i;

			retries *= 2;

			do_gettimeofday(&start);

			for (i = 0; i < retries; i++) {
				cpu_relax();
				sdcmd = bcm2835_sdhost_read(host, SDCMD);
			}

			do_gettimeofday(&now);
			us_diff = (now.tv_sec - start.tv_sec) * 1000000 +
				(now.tv_usec - start.tv_usec);
		} while (us_diff < 10);

		host->cmd_quick_poll_retries = ((retries * us_diff + 9)*CMD_DALLY_US)/10 + 1;
		retries = 1; // We've already waited long enough this time
	}

	retries = host->cmd_quick_poll_retries;
	for (sdcmd = bcm2835_sdhost_read(host, SDCMD);
	     (sdcmd & SDCMD_NEW_FLAG) && !(sdcmd & SDCMD_FAIL_FLAG) && retries;
	     retries--) {
		cpu_relax();
		sdcmd = bcm2835_sdhost_read(host, SDCMD);
	}

	if (!retries) {
		unsigned long wait_max;

		if (!irq_flags) {
			/* Schedule the work */
			log_event("CWWQ", 0, 0);
			schedule_work(&host->cmd_wait_wq);
			return;
		}

		/* Wait max 100 ms */
		wait_max = jiffies + msecs_to_jiffies(100);
		while (time_before(jiffies, wait_max)) {
			spin_unlock_irqrestore(&host->lock, *irq_flags);
			usleep_range(1, 10);
			spin_lock_irqsave(&host->lock, *irq_flags);
			sdcmd = bcm2835_sdhost_read(host, SDCMD);
			if (!(sdcmd & SDCMD_NEW_FLAG) ||
			    (sdcmd & SDCMD_FAIL_FLAG))
				break;
		}
	}

	/* Check for errors */
	if (sdcmd & SDCMD_NEW_FLAG) {
		if (host->debug) {
			pr_err("%s: command %d never completed.\n",
			       mmc_hostname(host->mmc), host->cmd->opcode);
			bcm2835_sdhost_dumpregs(host);
		}
		host->cmd->error = -EILSEQ;
		tasklet_schedule(&host->finish_tasklet);
		return;
	} else if (sdcmd & SDCMD_FAIL_FLAG) {
		u32 sdhsts = bcm2835_sdhost_read(host, SDHSTS);

		/* Clear the errors */
		bcm2835_sdhost_write(host, SDHSTS_ERROR_MASK, SDHSTS);

		if (host->debug)
			pr_info("%s: error detected - CMD %x, HSTS %03x, EDM %x\n",
				mmc_hostname(host->mmc), sdcmd, sdhsts,
				bcm2835_sdhost_read(host, SDEDM));

		if ((sdhsts & SDHSTS_CRC7_ERROR) &&
		    (host->cmd->opcode == 1)) {
			if (host->debug)
				pr_info("%s: ignoring CRC7 error for CMD1\n",
					mmc_hostname(host->mmc));
		} else {
			if (sdhsts & SDHSTS_CMD_TIME_OUT) {
				if (host->debug)
					pr_warn("%s: command %d timeout\n",
					       mmc_hostname(host->mmc),
					       host->cmd->opcode);
				host->cmd->error = -ETIMEDOUT;
			} else {
				pr_warn("%s: unexpected command %d error\n",
				       mmc_hostname(host->mmc),
				       host->cmd->opcode);
				host->cmd->error = -EILSEQ;
			}
			tasklet_schedule(&host->finish_tasklet);
			return;
		}
	}

	if (host->cmd->flags & MMC_RSP_PRESENT) {
		if (host->cmd->flags & MMC_RSP_136) {
			int i;
			for (i = 0; i < 4; i++)
				host->cmd->resp[3 - i] = bcm2835_sdhost_read(host, SDRSP0 + i*4);
			pr_debug("%s: finish_command %08x %08x %08x %08x\n",
				 mmc_hostname(host->mmc),
				 host->cmd->resp[0], host->cmd->resp[1], host->cmd->resp[2], host->cmd->resp[3]);
			log_event("RSP ", host->cmd->resp[0], host->cmd->resp[1]);
		} else {
			host->cmd->resp[0] = bcm2835_sdhost_read(host, SDRSP0);
			pr_debug("%s: finish_command %08x\n",
				 mmc_hostname(host->mmc),
				 host->cmd->resp[0]);
			log_event("RSP ", host->cmd->resp[0], 0);
		}
	}

	if (host->cmd == host->mrq->sbc) {
		/* Finished CMD23, now send actual command. */
		host->cmd = NULL;
		if (bcm2835_sdhost_send_command(host, host->mrq->cmd)) {
			if (host->data && host->dma_desc)
				/* DMA transfer starts now, PIO starts after irq */
				bcm2835_sdhost_start_dma(host);

			if (!host->use_busy)
				bcm2835_sdhost_finish_command(host, NULL);
		}
	} else if (host->cmd == host->mrq->stop) {
		/* Finished CMD12 */
		tasklet_schedule(&host->finish_tasklet);
	} else {
		/* Processed actual command. */
		host->cmd = NULL;
		if (!host->data)
			tasklet_schedule(&host->finish_tasklet);
		else if (host->data_complete)
			bcm2835_sdhost_transfer_complete(host);
	}
	log_event("FCM>", (u32)host->mrq, (u32)host->cmd);
}

static void bcm2835_sdhost_timeout(unsigned long data)
{
	struct bcm2835_host *host;
	unsigned long flags;

	host = (struct bcm2835_host *)data;

	spin_lock_irqsave(&host->lock, flags);
	log_event("TIM<", 0, 0);

	if (host->mrq) {
		pr_err("%s: timeout waiting for hardware interrupt.\n",
			mmc_hostname(host->mmc));
		log_dump();
		bcm2835_sdhost_dumpregs(host);

		if (host->data) {
			host->data->error = -ETIMEDOUT;
			bcm2835_sdhost_finish_data(host);
		} else {
			if (host->cmd)
				host->cmd->error = -ETIMEDOUT;
			else
				host->mrq->cmd->error = -ETIMEDOUT;

			pr_debug("timeout_timer tasklet_schedule\n");
			tasklet_schedule(&host->finish_tasklet);
		}
	}

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
}

static void bcm2835_sdhost_busy_irq(struct bcm2835_host *host, u32 intmask)
{
	log_event("IRQB", (u32)host->cmd, intmask);
	if (!host->cmd) {
		pr_err("%s: got command busy interrupt 0x%08x even "
			"though no command operation was in progress.\n",
			mmc_hostname(host->mmc), (unsigned)intmask);
		bcm2835_sdhost_dumpregs(host);
		return;
	}

	if (!host->use_busy) {
		pr_err("%s: got command busy interrupt 0x%08x even "
			"though not expecting one.\n",
			mmc_hostname(host->mmc), (unsigned)intmask);
		bcm2835_sdhost_dumpregs(host);
		return;
	}
	host->use_busy = 0;

	if (intmask & SDHSTS_ERROR_MASK)
	{
		pr_err("sdhost_busy_irq: intmask %x, data %p\n", intmask, host->mrq->data);
		if (intmask & SDHSTS_CRC7_ERROR)
			host->cmd->error = -EILSEQ;
		else if (intmask & (SDHSTS_CRC16_ERROR |
				    SDHSTS_FIFO_ERROR)) {
			if (host->mrq->data)
				host->mrq->data->error = -EILSEQ;
			else
				host->cmd->error = -EILSEQ;
		} else if (intmask & SDHSTS_REW_TIME_OUT) {
			if (host->mrq->data)
				host->mrq->data->error = -ETIMEDOUT;
			else
				host->cmd->error = -ETIMEDOUT;
		} else if (intmask & SDHSTS_CMD_TIME_OUT)
			host->cmd->error = -ETIMEDOUT;

		if (host->debug) {
			log_dump();
			bcm2835_sdhost_dumpregs(host);
		}
	}
	else
		bcm2835_sdhost_finish_command(host, NULL);
}

static void bcm2835_sdhost_data_irq(struct bcm2835_host *host, u32 intmask)
{
	/* There are no dedicated data/space available interrupt
	   status bits, so it is necessary to use the single shared
	   data/space available FIFO status bits. It is therefore not
	   an error to get here when there is no data transfer in
	   progress. */
	log_event("IRQD", (u32)host->data, intmask);
	if (!host->data)
		return;

	if (intmask & (SDHSTS_CRC16_ERROR |
		       SDHSTS_FIFO_ERROR |
		       SDHSTS_REW_TIME_OUT)) {
		if (intmask & (SDHSTS_CRC16_ERROR |
			       SDHSTS_FIFO_ERROR))
			host->data->error = -EILSEQ;
		else
			host->data->error = -ETIMEDOUT;

		if (host->debug) {
			log_dump();
			bcm2835_sdhost_dumpregs(host);
		}
	}

	if (host->data->error) {
		bcm2835_sdhost_finish_data(host);
	} else if (host->data->flags & MMC_DATA_WRITE) {
		/* Use the block interrupt for writes after the first block */
		host->hcfg &= ~(SDHCFG_DATA_IRPT_EN);
		host->hcfg |= SDHCFG_BLOCK_IRPT_EN;
		bcm2835_sdhost_write(host, host->hcfg, SDHCFG);
		bcm2835_sdhost_transfer_pio(host);
	} else {
		bcm2835_sdhost_transfer_pio(host);
		host->blocks--;
		if ((host->blocks == 0) || host->data->error)
			bcm2835_sdhost_finish_data(host);
	}
}

static void bcm2835_sdhost_block_irq(struct bcm2835_host *host, u32 intmask)
{
	log_event("IRQK", (u32)host->data, intmask);
	if (!host->data) {
		pr_err("%s: got block interrupt 0x%08x even "
			"though no data operation was in progress.\n",
			mmc_hostname(host->mmc), (unsigned)intmask);
		bcm2835_sdhost_dumpregs(host);
		return;
	}

	if (intmask & (SDHSTS_CRC16_ERROR |
		       SDHSTS_FIFO_ERROR |
		       SDHSTS_REW_TIME_OUT)) {
		if (intmask & (SDHSTS_CRC16_ERROR |
			       SDHSTS_FIFO_ERROR))
			host->data->error = -EILSEQ;
		else
			host->data->error = -ETIMEDOUT;

		if (host->debug) {
			log_dump();
			bcm2835_sdhost_dumpregs(host);
		}
	}

	if (!host->dma_desc) {
		BUG_ON(!host->blocks);
		if (host->data->error || (--host->blocks == 0)) {
			bcm2835_sdhost_finish_data(host);
		} else {
			bcm2835_sdhost_transfer_pio(host);
		}
	} else if (host->data->flags & MMC_DATA_WRITE) {
		bcm2835_sdhost_finish_data(host);
	}
}

static irqreturn_t bcm2835_sdhost_irq(int irq, void *dev_id)
{
	irqreturn_t result = IRQ_NONE;
	struct bcm2835_host *host = dev_id;
	u32 intmask;

	spin_lock(&host->lock);

	intmask = bcm2835_sdhost_read(host, SDHSTS);
	log_event("IRQ<", intmask, 0);

	bcm2835_sdhost_write(host,
			     SDHSTS_BUSY_IRPT |
			     SDHSTS_BLOCK_IRPT |
			     SDHSTS_SDIO_IRPT |
			     SDHSTS_DATA_FLAG,
			     SDHSTS);

	if (intmask & SDHSTS_BLOCK_IRPT) {
		bcm2835_sdhost_block_irq(host, intmask);
		result = IRQ_HANDLED;
	}

	if (intmask & SDHSTS_BUSY_IRPT) {
		bcm2835_sdhost_busy_irq(host, intmask);
		result = IRQ_HANDLED;
	}

	/* There is no true data interrupt status bit, so it is
	   necessary to qualify the data flag with the interrupt
	   enable bit */
	if ((intmask & SDHSTS_DATA_FLAG) &&
	    (host->hcfg & SDHCFG_DATA_IRPT_EN)) {
		bcm2835_sdhost_data_irq(host, intmask);
		result = IRQ_HANDLED;
	}

	mmiowb();

	log_event("IRQ>", bcm2835_sdhost_read(host, SDHSTS), 0);
	spin_unlock(&host->lock);

	return result;
}

void bcm2835_sdhost_set_clock(struct bcm2835_host *host, unsigned int clock)
{
	int div = 0; /* Initialized for compiler warning */
	unsigned int input_clock = clock;
	unsigned long flags;

	if (host->debug)
		pr_info("%s: set_clock(%d)\n", mmc_hostname(host->mmc), clock);

	if ((host->overclock_50 > 50) &&
	    (clock == 50*MHZ))
		clock = host->overclock_50 * MHZ + (MHZ - 1);

	/* The SDCDIV register has 11 bits, and holds (div - 2).
	   But in data mode the max is 50MHz wihout a minimum, and only the
	   bottom 3 bits are used. Since the switch over is automatic (unless
	   we have marked the card as slow...), chosen values have to make
	   sense in both modes.
	   Ident mode must be 100-400KHz, so can range check the requested
	   clock. CMD15 must be used to return to data mode, so this can be
	   monitored.

	   clock 250MHz -> 0->125MHz, 1->83.3MHz, 2->62.5MHz, 3->50.0MHz
                           4->41.7MHz, 5->35.7MHz, 6->31.3MHz, 7->27.8MHz

			 623->400KHz/27.8MHz
			 reset value (507)->491159/50MHz

	   BUT, the 3-bit clock divisor in data mode is too small if the
	   core clock is higher than 250MHz, so instead use the SLOW_CARD
	   configuration bit to force the use of the ident clock divisor
	   at all times.
	*/

	host->mmc->actual_clock = 0;

	if (host->firmware_sets_cdiv) {
		u32 msg[3] = { clock, 0, 0 };

		rpi_firmware_property(rpi_firmware_get(NULL),
				      RPI_FIRMWARE_SET_SDHOST_CLOCK,
				      &msg, sizeof(msg));

		clock = max(msg[1], msg[2]);
		spin_lock_irqsave(&host->lock, flags);
	} else {
		spin_lock_irqsave(&host->lock, flags);
		if (clock < 100000) {
			/* Can't stop the clock, but make it as slow as
			 * possible to show willing
			 */
			host->cdiv = SDCDIV_MAX_CDIV;
			bcm2835_sdhost_write(host, host->cdiv, SDCDIV);
			mmiowb();
			spin_unlock_irqrestore(&host->lock, flags);
			return;
		}

		div = host->max_clk / clock;
		if (div < 2)
			div = 2;
		if ((host->max_clk / div) > clock)
			div++;
		div -= 2;

		if (div > SDCDIV_MAX_CDIV)
			div = SDCDIV_MAX_CDIV;

		clock = host->max_clk / (div + 2);

		host->cdiv = div;
		bcm2835_sdhost_write(host, host->cdiv, SDCDIV);

		if (host->debug)
			pr_info("%s: clock=%d -> max_clk=%d, cdiv=%x "
				"(actual clock %d)\n",
				mmc_hostname(host->mmc), input_clock,
				host->max_clk, host->cdiv,
				clock);
	}

	/* Calibrate some delays */

	host->ns_per_fifo_word = (1000000000/clock) *
		((host->mmc->caps & MMC_CAP_4_BIT_DATA) ? 8 : 32);

	if (input_clock == 50 * MHZ) {
		if (clock > input_clock) {
			/* Save the closest value, to make it easier
			   to reduce in the event of error */
			host->overclock_50 = (clock/MHZ);

			if (clock != host->overclock) {
				pr_info("%s: overclocking to %dHz\n",
					mmc_hostname(host->mmc), clock);
				host->overclock = clock;
			}
		} else if (host->overclock) {
			host->overclock = 0;
			if (clock == 50 * MHZ)
				pr_warn("%s: cancelling overclock\n",
					mmc_hostname(host->mmc));
		}
	} else if (input_clock == 0) {
		/* Reset the preferred overclock when the clock is stopped.
		 * This always happens during initialisation. */
		host->overclock_50 = host->user_overclock_50;
		host->overclock = 0;
	}

	/* Set the timeout to 500ms */
	bcm2835_sdhost_write(host, clock/2, SDTOUT);

	host->mmc->actual_clock = clock;
	host->clock = input_clock;
	host->reset_clock = 0;

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
}

static void bcm2835_sdhost_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct bcm2835_host *host;
	unsigned long flags;
	u32 edm, fsm;

	host = mmc_priv(mmc);

	if (host->debug) {
		struct mmc_command *cmd = mrq->cmd;
		BUG_ON(!cmd);
		if (cmd->data)
			pr_info("%s: cmd %d 0x%x (flags 0x%x) - %s %d*%d\n",
				mmc_hostname(mmc),
				cmd->opcode, cmd->arg, cmd->flags,
				(cmd->data->flags & MMC_DATA_READ) ?
				"read" : "write", cmd->data->blocks,
				cmd->data->blksz);
		else
			pr_info("%s: cmd %d 0x%x (flags 0x%x)\n",
				mmc_hostname(mmc),
				cmd->opcode, cmd->arg, cmd->flags);
	}

	/* Reset the error statuses in case this is a retry */
	if (mrq->sbc)
		mrq->sbc->error = 0;
	if (mrq->cmd)
		mrq->cmd->error = 0;
	if (mrq->data)
		mrq->data->error = 0;
	if (mrq->stop)
		mrq->stop->error = 0;

	if (mrq->data && !is_power_of_2(mrq->data->blksz)) {
		pr_err("%s: unsupported block size (%d bytes)\n",
		       mmc_hostname(mmc), mrq->data->blksz);
		mrq->cmd->error = -EINVAL;
		mmc_request_done(mmc, mrq);
		return;
	}

	if (host->use_dma && mrq->data &&
	    (mrq->data->blocks > host->pio_limit))
		bcm2835_sdhost_prepare_dma(host, mrq->data);

	if (host->reset_clock)
	    bcm2835_sdhost_set_clock(host, host->clock);

	spin_lock_irqsave(&host->lock, flags);

	WARN_ON(host->mrq != NULL);
	host->mrq = mrq;

	edm = bcm2835_sdhost_read(host, SDEDM);
	fsm = edm & SDEDM_FSM_MASK;

	log_event("REQ<", (u32)mrq, edm);
	if ((fsm != SDEDM_FSM_IDENTMODE) &&
	    (fsm != SDEDM_FSM_DATAMODE)) {
		log_event("REQ!", (u32)mrq, edm);
		if (host->debug) {
			pr_warn("%s: previous command (%d) not complete (EDM %x)\n",
			       mmc_hostname(host->mmc),
			       bcm2835_sdhost_read(host, SDCMD) & SDCMD_CMD_MASK,
			       edm);
			log_dump();
			bcm2835_sdhost_dumpregs(host);
		}
		mrq->cmd->error = -EILSEQ;
		tasklet_schedule(&host->finish_tasklet);
		mmiowb();
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}

	host->use_sbc = !!mrq->sbc &&
		(host->mrq->data->flags & USE_CMD23_FLAGS);
	if (host->use_sbc) {
		if (bcm2835_sdhost_send_command(host, mrq->sbc)) {
			if (!host->use_busy)
				bcm2835_sdhost_finish_command(host, &flags);
		}
	} else if (bcm2835_sdhost_send_command(host, mrq->cmd)) {
		if (host->data && host->dma_desc)
			/* DMA transfer starts now, PIO starts after irq */
			bcm2835_sdhost_start_dma(host);

		if (!host->use_busy)
			bcm2835_sdhost_finish_command(host, &flags);
	}

	log_event("CMD ", (u32)mrq->cmd->opcode,
		   mrq->data ? (u32)mrq->data->blksz : 0);
	mmiowb();

	log_event("REQ>", (u32)mrq, 0);
	spin_unlock_irqrestore(&host->lock, flags);
}

static void bcm2835_sdhost_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{

	struct bcm2835_host *host = mmc_priv(mmc);
	unsigned long flags;

	if (host->debug)
		pr_info("%s: ios clock %d, pwr %d, bus_width %d, "
			"timing %d, vdd %d, drv_type %d\n",
			mmc_hostname(mmc),
			ios->clock, ios->power_mode, ios->bus_width,
			ios->timing, ios->signal_voltage, ios->drv_type);

	spin_lock_irqsave(&host->lock, flags);

	log_event("IOS<", ios->clock, 0);

	/* set bus width */
	host->hcfg &= ~SDHCFG_WIDE_EXT_BUS;
	if (ios->bus_width == MMC_BUS_WIDTH_4)
		host->hcfg |= SDHCFG_WIDE_EXT_BUS;

	host->hcfg |= SDHCFG_WIDE_INT_BUS;

	/* Disable clever clock switching, to cope with fast core clocks */
	host->hcfg |= SDHCFG_SLOW_CARD;

	bcm2835_sdhost_write(host, host->hcfg, SDHCFG);

	mmiowb();

	spin_unlock_irqrestore(&host->lock, flags);

	if (!ios->clock || ios->clock != host->clock)
		bcm2835_sdhost_set_clock(host, ios->clock);
}

static struct mmc_host_ops bcm2835_sdhost_ops = {
	.request = bcm2835_sdhost_request,
	.set_ios = bcm2835_sdhost_set_ios,
	.hw_reset = bcm2835_sdhost_reset,
};

static void bcm2835_sdhost_cmd_wait_work(struct work_struct *work)
{
	struct bcm2835_host *host;
	unsigned long flags;

	host = container_of(work, struct bcm2835_host, cmd_wait_wq);

	spin_lock_irqsave(&host->lock, flags);

	log_event("CWK<", (u32)host->cmd, (u32)host->mrq);

	/*
	 * If this tasklet gets rescheduled while running, it will
	 * be run again afterwards but without any active request.
	 */
	if (!host->mrq) {
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}

	bcm2835_sdhost_finish_command(host, &flags);

	mmiowb();

	log_event("CWK>", (u32)host->cmd, 0);

	spin_unlock_irqrestore(&host->lock, flags);
}

static void bcm2835_sdhost_tasklet_finish(unsigned long param)
{
	struct bcm2835_host *host;
	unsigned long flags;
	struct mmc_request *mrq;
	struct dma_chan *terminate_chan = NULL;

	host = (struct bcm2835_host *)param;

	spin_lock_irqsave(&host->lock, flags);

	log_event("TSK<", (u32)host->mrq, 0);
	/*
	 * If this tasklet gets rescheduled while running, it will
	 * be run again afterwards but without any active request.
	 */
	if (!host->mrq) {
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}

	del_timer(&host->timer);

	mrq = host->mrq;

	/* Drop the overclock after any data corruption, or after any
	 * error while overclocked. Ignore errors for status commands,
	 * as they are likely when a card is ejected. */
	if (host->overclock) {
		if ((mrq->cmd && mrq->cmd->error &&
		     (mrq->cmd->opcode != MMC_SEND_STATUS)) ||
		    (mrq->data && mrq->data->error) ||
		    (mrq->stop && mrq->stop->error) ||
		    (mrq->sbc && mrq->sbc->error)) {
			host->overclock_50--;
			pr_warn("%s: reducing overclock due to errors\n",
				mmc_hostname(host->mmc));
			host->reset_clock = 1;
			mrq->cmd->error = -ETIMEDOUT;
			mrq->cmd->retries = 1;
		}
	}

	host->mrq = NULL;
	host->cmd = NULL;
	host->data = NULL;

	mmiowb();

	host->dma_desc = NULL;
	terminate_chan = host->dma_chan;
	host->dma_chan = NULL;

	spin_unlock_irqrestore(&host->lock, flags);

	if (terminate_chan)
	{
		int err = dmaengine_terminate_all(terminate_chan);
		if (err)
			pr_err("%s: failed to terminate DMA (%d)\n",
			       mmc_hostname(host->mmc), err);
	}

	/* The SDHOST block doesn't report any errors for a disconnected
	   interface. All cards and SDIO devices should report some supported
	   voltage range, so a zero response to SEND_OP_COND, IO_SEND_OP_COND
	   or APP_SEND_OP_COND can be treated as an error. */
	if (((mrq->cmd->opcode == MMC_SEND_OP_COND) ||
	     (mrq->cmd->opcode == SD_IO_SEND_OP_COND) ||
	     (mrq->cmd->opcode == SD_APP_OP_COND)) &&
	    (mrq->cmd->error == 0) &&
	    (mrq->cmd->resp[0] == 0)) {
		mrq->cmd->error = -ETIMEDOUT;
		if (host->debug)
			pr_info("%s: faking timeout due to zero OCR\n",
				mmc_hostname(host->mmc));
	}

	mmc_request_done(host->mmc, mrq);
	log_event("TSK>", (u32)mrq, 0);
}

int bcm2835_sdhost_add_host(struct bcm2835_host *host)
{
	struct mmc_host *mmc;
	struct dma_slave_config cfg;
	char pio_limit_string[20];
	int ret;

	mmc = host->mmc;

	bcm2835_sdhost_reset_internal(host);

	mmc->f_max = host->max_clk;
	mmc->f_min = host->max_clk / SDCDIV_MAX_CDIV;

	mmc->max_busy_timeout =  (~(unsigned int)0)/(mmc->f_max/1000);

	pr_debug("f_max %d, f_min %d, max_busy_timeout %d\n",
		 mmc->f_max, mmc->f_min, mmc->max_busy_timeout);

	/* host controller capabilities */
	mmc->caps |=
		MMC_CAP_SD_HIGHSPEED | MMC_CAP_MMC_HIGHSPEED |
		MMC_CAP_NEEDS_POLL | MMC_CAP_HW_RESET | MMC_CAP_ERASE |
		((ALLOW_CMD23_READ|ALLOW_CMD23_WRITE) * MMC_CAP_CMD23);

	spin_lock_init(&host->lock);

	if (host->allow_dma) {
		if (IS_ERR_OR_NULL(host->dma_chan_rxtx)) {
			pr_err("%s: unable to initialise DMA channel. "
			       "Falling back to PIO\n",
			       mmc_hostname(mmc));
			host->use_dma = false;
		} else {
			cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
			cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
			cfg.slave_id = 13;		/* DREQ channel */

			/* Validate the slave configurations */

			cfg.direction = DMA_MEM_TO_DEV;
			cfg.src_addr = 0;
			cfg.dst_addr = host->bus_addr + SDDATA;

			ret = dmaengine_slave_config(host->dma_chan_rxtx, &cfg);

			if (ret == 0) {
				host->dma_cfg_tx = cfg;

				cfg.direction = DMA_DEV_TO_MEM;
				cfg.src_addr = host->bus_addr + SDDATA;
				cfg.dst_addr = 0;

				ret = dmaengine_slave_config(host->dma_chan_rxtx, &cfg);
			}

			if (ret == 0) {
				host->dma_cfg_rx = cfg;

				host->use_dma = true;
			} else {
				pr_err("%s: unable to configure DMA channel. "
				       "Falling back to PIO\n",
				       mmc_hostname(mmc));
				dma_release_channel(host->dma_chan_rxtx);
				host->dma_chan_rxtx = NULL;
				host->use_dma = false;
			}
		}
	} else {
		host->use_dma = false;
	}

	mmc->max_segs = 128;
	mmc->max_req_size = 524288;
	mmc->max_seg_size = mmc->max_req_size;
	mmc->max_blk_size = 512;
	mmc->max_blk_count =  65535;

	/* report supported voltage ranges */
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;

	tasklet_init(&host->finish_tasklet,
		bcm2835_sdhost_tasklet_finish, (unsigned long)host);

	INIT_WORK(&host->cmd_wait_wq, bcm2835_sdhost_cmd_wait_work);

	setup_timer(&host->timer, bcm2835_sdhost_timeout,
		    (unsigned long)host);

	bcm2835_sdhost_init(host, 0);

	ret = request_irq(host->irq, bcm2835_sdhost_irq, 0 /*IRQF_SHARED*/,
				  mmc_hostname(mmc), host);
	if (ret) {
		pr_err("%s: failed to request IRQ %d: %d\n",
		       mmc_hostname(mmc), host->irq, ret);
		goto untasklet;
	}

	mmiowb();
	mmc_add_host(mmc);

	pio_limit_string[0] = '\0';
	if (host->use_dma && (host->pio_limit > 0))
		sprintf(pio_limit_string, " (>%d)", host->pio_limit);
	pr_info("%s: %s loaded - DMA %s%s\n",
		mmc_hostname(mmc), DRIVER_NAME,
		host->use_dma ? "enabled" : "disabled",
		pio_limit_string);

	return 0;

untasklet:
	tasklet_kill(&host->finish_tasklet);

	return ret;
}

static int bcm2835_sdhost_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct clk *clk;
	struct resource *iomem;
	struct bcm2835_host *host;
	struct mmc_host *mmc;
	const __be32 *addr;
	u32 msg[3];
	int ret;

	pr_debug("bcm2835_sdhost_probe\n");
	mmc = mmc_alloc_host(sizeof(*host), dev);
	if (!mmc)
		return -ENOMEM;

	mmc->ops = &bcm2835_sdhost_ops;
	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->pio_timeout = msecs_to_jiffies(500);
	host->pio_limit = 1;
	host->max_delay = 1; /* Warn if over 1ms */
	host->allow_dma = 1;
	spin_lock_init(&host->lock);

	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	host->ioaddr = devm_ioremap_resource(dev, iomem);
	if (IS_ERR(host->ioaddr)) {
		ret = PTR_ERR(host->ioaddr);
		goto err;
	}

	addr = of_get_address(node, 0, NULL, NULL);
	if (!addr) {
		dev_err(dev, "could not get DMA-register address\n");
		return -ENODEV;
	}
	host->bus_addr = be32_to_cpup(addr);
	log_init(iomem->start - host->bus_addr);
	pr_debug(" - ioaddr %lx, iomem->start %lx, bus_addr %lx\n",
		 (unsigned long)host->ioaddr,
		 (unsigned long)iomem->start,
		 (unsigned long)host->bus_addr);

	if (node) {
		/* Read any custom properties */
		of_property_read_u32(node,
				     "brcm,delay-after-stop",
				     &host->delay_after_stop);
		of_property_read_u32(node,
				     "brcm,overclock-50",
				     &host->user_overclock_50);
		of_property_read_u32(node,
				     "brcm,pio-limit",
				     &host->pio_limit);
		host->allow_dma =
			!of_property_read_bool(node, "brcm,force-pio");
		host->debug = of_property_read_bool(node, "brcm,debug");
	}

	host->dma_chan = NULL;
	host->dma_desc = NULL;

	/* Formally recognise the other way of disabling DMA */
	if (host->pio_limit == 0x7fffffff)
		host->allow_dma = false;

	if (host->allow_dma) {
		if (node) {
			host->dma_chan_rxtx =
				dma_request_slave_channel(dev, "rx-tx");
			if (!host->dma_chan_rxtx)
				host->dma_chan_rxtx =
					dma_request_slave_channel(dev, "tx");
			if (!host->dma_chan_rxtx)
				host->dma_chan_rxtx =
					dma_request_slave_channel(dev, "rx");
		} else {
			dma_cap_mask_t mask;

			dma_cap_zero(mask);
			/* we don't care about the channel, any would work */
			dma_cap_set(DMA_SLAVE, mask);
			host->dma_chan_rxtx =
				dma_request_channel(mask, NULL, NULL);
		}
	}

	clk = devm_clk_get(dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(dev, "could not get clk\n");
		ret = PTR_ERR(clk);
		goto err;
	}

	host->max_clk = clk_get_rate(clk);

	host->irq = platform_get_irq(pdev, 0);
	if (host->irq <= 0) {
		dev_err(dev, "get IRQ failed\n");
		ret = -EINVAL;
		goto err;
	}

	pr_debug(" - max_clk %lx, irq %d\n",
		 (unsigned long)host->max_clk,
		 (int)host->irq);

	if (node)
		mmc_of_parse(mmc);
	else
		mmc->caps |= MMC_CAP_4_BIT_DATA;

	msg[0] = 0;
	msg[1] = ~0;
	msg[2] = ~0;

	rpi_firmware_property(rpi_firmware_get(NULL),
			      RPI_FIRMWARE_SET_SDHOST_CLOCK,
			      &msg, sizeof(msg));

	host->firmware_sets_cdiv = (msg[1] != ~0);

	ret = bcm2835_sdhost_add_host(host);
	if (ret)
		goto err;

	platform_set_drvdata(pdev, host);

	pr_debug("bcm2835_sdhost_probe -> OK\n");

	return 0;

err:
	pr_debug("bcm2835_sdhost_probe -> err %d\n", ret);
	mmc_free_host(mmc);

	return ret;
}

static int bcm2835_sdhost_remove(struct platform_device *pdev)
{
	struct bcm2835_host *host = platform_get_drvdata(pdev);

	pr_debug("bcm2835_sdhost_remove\n");

	mmc_remove_host(host->mmc);

	bcm2835_sdhost_set_power(host, false);

	free_irq(host->irq, host);

	del_timer_sync(&host->timer);

	tasklet_kill(&host->finish_tasklet);

	mmc_free_host(host->mmc);
	platform_set_drvdata(pdev, NULL);

	pr_debug("bcm2835_sdhost_remove - OK\n");
	return 0;
}

static const struct of_device_id bcm2835_sdhost_match[] = {
	{ .compatible = "brcm,bcm2835-sdhost" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm2835_sdhost_match);

static struct platform_driver bcm2835_sdhost_driver = {
	.probe      = bcm2835_sdhost_probe,
	.remove     = bcm2835_sdhost_remove,
	.driver     = {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= bcm2835_sdhost_match,
	},
};
module_platform_driver(bcm2835_sdhost_driver);

MODULE_ALIAS("platform:sdhost-bcm2835");
MODULE_DESCRIPTION("BCM2835 SDHost driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Phil Elwell");
