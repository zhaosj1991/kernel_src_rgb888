/*
 * Tegra Graphics Host Interrupt Management
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>

#include "bus_client_t186.h"
#include "nvhost_intr.h"
#include "nvhost_ktime.h"
#include "dev.h"
#include "../../../../../../kernel-4.4/drivers/media/platform/tegra/camera/vi/vi4_registers.h"


/* Spacing between sync registers */
#define REGISTER_STRIDE 4

extern struct tegra_channel_buffer *dequeue_buffer(struct tegra_channel *chan);
extern void tegra_channel_surface_setup(struct tegra_channel *chan, struct tegra_channel_buffer *buf, int index);
extern void vi4_channel_write(struct tegra_channel *chan,
		unsigned int index, unsigned int addr, u32 val);

static void intr_syncpt_intr_ack(struct nvhost_intr_syncpt *syncpt,
				     bool disable_intr);
static void intr_enable_syncpt_intr(struct nvhost_intr *intr, u32 id);
static void intr_set_syncpt_threshold(struct nvhost_intr *intr,
					  u32 id, u32 thresh);


static void vi_chan_capture(struct tegra_channel *chan, struct nvhost_intr *intr,
			     struct nvhost_intr_syncpt *syncpt,
			     u32 threshold)
{
	bool is_streaming = atomic_read(&chan->is_streaming);
	struct tegra_channel_buffer *buf;

	if (!is_streaming){
		printk("### vi_chan_capture is_streaming is false !\n");
		return;
	}
	
	/* Put buffer into the release queue */
	spin_lock(&chan->release_lock);
	list_add_tail(&chan->cur_buf->queue, &chan->release);
	spin_unlock(&chan->release_lock);
	
	buf = dequeue_buffer(chan);
	if (!buf) {
		printk("### vi_chan_capture chan->capture is NULL !\n");
		return;
	}

	chan->cur_buf = buf;
	tegra_channel_surface_setup(chan, buf, 0);

	vi4_channel_write(chan, chan->vnc_id[0], CHANNEL_COMMAND, LOAD);
	vi4_channel_write(chan, chan->vnc_id[0],
				CONTROL, SINGLESHOT | MATCH_STATE_EN);
	
	/* take lock on syncpt list */
	spin_lock(&syncpt->lock);

	// increase thresh & enable interrupt
	intr_set_syncpt_threshold(intr, syncpt->id, (threshold + 1) & 0x3fffff);
	intr_enable_syncpt_intr(intr, syncpt->id);

	/* release syncpt lock */
	spin_unlock(&syncpt->lock);
		
	// schedule tasklet to release buffer
	tasklet_schedule(&chan->tasklet_vi4);
}
static void vi_syncpt_thresh_fn(void *dev_id, struct tegra_channel *chan)
{
	struct nvhost_intr_syncpt *syncpt = dev_id;
	unsigned int id = syncpt->id;
	struct nvhost_intr *intr = intr_syncpt_to_intr(syncpt);
	struct nvhost_master *dev = intr_to_dev(intr);
	int err;

	/* make sure host1x is powered */
	err = nvhost_module_busy(dev->dev);
	if (err) {
		WARN(1, "failed to powerON host1x.");
		return;
	}

	vi_chan_capture(chan, intr, syncpt,
				nvhost_syncpt_update_min(&dev->syncpt, id));

	nvhost_module_idle(dev->dev);
}

static irqreturn_t syncpt_thresh_cascade_isr(int irq, void *dev_id)
{
	struct nvhost_master *dev = dev_id;
	struct nvhost_intr *intr = &dev->intr;
	unsigned long reg;
	int i, id;

	for (i = 0; i < DIV_ROUND_UP(nvhost_syncpt_nb_hw_pts(&dev->syncpt), 32);
			i++) {
		reg = host1x_readl(dev->dev,
				host1x_sync_syncpt_thresh_cpu0_int_status_r() +
				i * REGISTER_STRIDE);

		for_each_set_bit(id, &reg, 32) {
			struct nvhost_intr_syncpt *sp;
			int sp_id = i * 32 + id;
			int graphics_host_sp =
				nvhost_syncpt_graphics_host_sp(&dev->syncpt);

			if (unlikely(!nvhost_syncpt_is_valid_hw_pt(&dev->syncpt,
					sp_id))) {
				dev_err(&dev->dev->dev, "%s(): syncpoint id %d is beyond the number of syncpoints (%d)\n",
					__func__, sp_id,
					nvhost_syncpt_nb_hw_pts(&dev->syncpt));
				goto out;
			}

			sp = intr->syncpt + sp_id;
			nvhost_ktime_get_ts(&sp->isr_recv);

			/* handle graphics host syncpoint increments
			 * immediately
			 */
			if (sp_id == graphics_host_sp) {
				dev_warn(&dev->dev->dev, "%s(): syncpoint id %d incremented\n",
					 __func__, graphics_host_sp);
				nvhost_syncpt_patch_check(&dev->syncpt);
				intr_syncpt_intr_ack(sp, false);
			} if (sp_id == 23){
				struct tegra_mc_vi *mc_vi = dev->mc_vi;
				struct tegra_channel *chan = list_first_entry(&(mc_vi->vi_chans), struct tegra_channel, list);
				
				if (chan == NULL){
					printk("### host1x_intr_t186.c : syncpt_thresh_cascade_isr chan is NULL!\n");
				}else{
					// clear & disable interrupt
					intr_syncpt_intr_ack(sp, true);
					vi_syncpt_thresh_fn(sp, chan);
				}
			}else {
				intr_syncpt_intr_ack(sp, true);
				nvhost_syncpt_thresh_fn(sp);
			}
		}
	}

out:
	return IRQ_HANDLED;
}

static void intr_set_host_clocks_per_usec(struct nvhost_intr *intr, u32 cpm)
{
}

static void intr_set_syncpt_threshold(struct nvhost_intr *intr,
	u32 id, u32 thresh)
{
	struct nvhost_master *dev = intr_to_dev(intr);
	host1x_writel(dev->dev,
		(host1x_sync_syncpt_int_thresh_0_r() + id * REGISTER_STRIDE),
		thresh);
}

static void intr_enable_syncpt_intr(struct nvhost_intr *intr, u32 id)
{
	struct nvhost_master *dev = intr_to_dev(intr);
	u32 thresh;

	host1x_writel(dev->dev,
			host1x_sync_syncpt_thresh_int_enable_cpu0_r() +
			bit_word(id) * REGISTER_STRIDE,
			bit_mask(id));

	/* set thershold again to make sure
	 * interrupt doesn't miss
	 */
	thresh = host1x_readl(dev->dev,
		(host1x_sync_syncpt_int_thresh_0_r() + id * REGISTER_STRIDE));
	host1x_writel(dev->dev,
		(host1x_sync_syncpt_int_thresh_0_r() + id * REGISTER_STRIDE),
		thresh);
}

static void intr_disable_syncpt_intr(struct nvhost_intr *intr, u32 id)
{
	struct nvhost_master *dev = intr_to_dev(intr);

	host1x_writel(dev->dev,
			host1x_sync_syncpt_thresh_int_disable_r() +
			bit_word(id) * REGISTER_STRIDE,
			bit_mask(id));

	/* clear status for both cpu's */
	host1x_writel(dev->dev,
		host1x_sync_syncpt_thresh_cpu0_int_status_r() +
		bit_word(id) * REGISTER_STRIDE,
		bit_mask(id));
}

static void intr_disable_all_syncpt_intrs(struct nvhost_intr *intr)
{
	struct nvhost_master *dev = intr_to_dev(intr);
	u32 reg;

	for (reg = 0; reg < bit_word(nvhost_syncpt_nb_hw_pts(&dev->syncpt))
			* REGISTER_STRIDE; reg += REGISTER_STRIDE) {
		/* disable interrupts for both cpu's */
		host1x_writel(dev->dev,
				host1x_sync_syncpt_thresh_int_disable_r() +
				reg, 0xffffffffu);

		/* clear status for both cpu's */
		host1x_writel(dev->dev,
			host1x_sync_syncpt_thresh_cpu0_int_status_r() + reg,
			0xffffffffu);
	}
}

/*
 * Acknowledge that the syncpoint interrupt is handled. If disable_intr is set,
 * the syncpoint interrupt is also disabled.
 */
static void intr_syncpt_intr_ack(struct nvhost_intr_syncpt *syncpt,
				     bool disable_intr)
{
	unsigned int id = syncpt->id;
	struct nvhost_intr *intr = intr_syncpt_to_intr(syncpt);
	struct nvhost_master *dev = intr_to_dev(intr);

	u32 reg = bit_word(id) * REGISTER_STRIDE;

	if (disable_intr)
		host1x_writel(dev->dev,
		       host1x_sync_syncpt_thresh_int_disable_r() + reg,
			bit_mask(id));

	host1x_writel(dev->dev,
		host1x_sync_syncpt_thresh_cpu0_int_status_r() + reg,
		bit_mask(id));
}

/**
 * Host general interrupt service function
 * Handles read / write failures
 */
static irqreturn_t intr_host1x_isr(int irq, void *dev_id)
{
	struct nvhost_intr *intr = dev_id;
	struct nvhost_master *dev = intr_to_dev(intr);
	u32 addr, i;
	unsigned long intstat;

	intstat = host1x_hypervisor_readl(dev->dev,
			host1x_sync_intstatus_r());
	intr->intstatus = intstat;

	for_each_set_bit(i, &intstat, 32) {
		if (intr->host_isr[i])
			intr->host_isr[i](intstat, intr->host_isr_priv[i]);
	}

	if (host1x_sync_intstatus_ip_read_int_v(intstat)) {
		addr = host1x_hypervisor_readl(dev->dev,
				host1x_sync_ip_read_timeout_addr_r());
		pr_err("Host read timeout at address %x\n", addr);
	}

	if (host1x_sync_intstatus_ip_write_int_v(intstat)) {
		addr = host1x_hypervisor_readl(dev->dev,
				host1x_sync_ip_write_timeout_addr_r());
		pr_err("Host write timeout at address %x\n", addr);
	}

	if (host1x_sync_intstatus_illegal_pb_access_v(intstat)) {
		u32 stat = host1x_hypervisor_readl(dev->dev,
			host1x_sync_illegal_syncpt_access_frm_pb_r());
		u32 ch = host1x_sync_illegal_syncpt_access_frm_pb_ch_v(stat);
		u32 id = host1x_sync_illegal_syncpt_access_frm_pb_syncpt_v(stat);

		pr_err("Illegal syncpoint pb access (ch=%u, id=%u)\n", ch, id);
	}

	if (host1x_sync_intstatus_illegal_client_access_v(intstat)) {
		u32 stat = host1x_hypervisor_readl(dev->dev,
			host1x_sync_illegal_syncpt_access_frm_client_r());
		u32 id = host1x_sync_illegal_syncpt_access_frm_client_syncpt_v(stat);
		u32 ch = host1x_sync_illegal_syncpt_access_frm_client_ch_v(stat);

		pr_err("Illegal syncpoint client access (ch=%u, id=%u)\n",
		       ch, id);
	}

	host1x_hypervisor_writel(dev->dev, host1x_sync_intstatus_r(), intstat);

	return IRQ_HANDLED;
}

static void intr_enable_host_irq(struct nvhost_intr *intr, int irq)
{
	struct nvhost_master *dev = intr_to_dev(intr);
	unsigned long val;

	val = host1x_hypervisor_readl(dev->dev, host1x_sync_intmask_r());
	val |= BIT(irq);
	host1x_hypervisor_writel(dev->dev, host1x_sync_intmask_r(), val);
}

static void intr_disable_host_irq(struct nvhost_intr *intr, int irq)
{
	struct nvhost_master *dev = intr_to_dev(intr);
	unsigned long val;

	val = host1x_hypervisor_readl(dev->dev, host1x_sync_intmask_r());
	val &= ~BIT(irq);
	host1x_hypervisor_writel(dev->dev, host1x_sync_intmask_r(), val);
}

static void intr_enable_module_intr(struct nvhost_intr *intr, int irq)
{
	struct nvhost_master *dev = intr_to_dev(intr);
	unsigned long val;

	val = host1x_hypervisor_readl(dev->dev, host1x_sync_intc0mask_r());
	val |= BIT(irq);
	host1x_hypervisor_writel(dev->dev, host1x_sync_intc0mask_r(), val);
}

static void intr_disable_module_intr(struct nvhost_intr *intr, int irq)
{
	struct nvhost_master *dev = intr_to_dev(intr);
	unsigned long val;

	val = host1x_hypervisor_readl(dev->dev, host1x_sync_intc0mask_r());
	val &= ~BIT(irq);
	host1x_hypervisor_writel(dev->dev, host1x_sync_intc0mask_r(), val);
}

static int intr_debug_dump(struct nvhost_intr *intr, struct output *o)
{
	struct nvhost_master *dev = intr_to_dev(intr);
	int i;

	nvhost_debug_output(o, "\n---- host general irq ----\n\n");
	nvhost_debug_output(o, "sync_intc0mask = 0x%08x\n",
		host1x_hypervisor_readl(dev->dev, host1x_sync_intc0mask_r()));
	nvhost_debug_output(o, "sync_intmask = 0x%08x\n",
		host1x_hypervisor_readl(dev->dev, host1x_sync_intmask_r()));

	nvhost_debug_output(o, "\n---- host syncpt irq mask ----\n\n");

	nvhost_debug_output(o, "\n---- host syncpt irq status ----\n\n");
	for (i = 0; i < DIV_ROUND_UP(nvhost_syncpt_nb_hw_pts(&dev->syncpt), 32);
			i++)
		nvhost_debug_output(o, "syncpt_thresh_cpu0_int_status(%d) = 0x%08x\n",
			i, host1x_readl(dev->dev,
				host1x_sync_syncpt_thresh_cpu0_int_status_r() +
				i * REGISTER_STRIDE));

	return 0;
}

static void intr_resume(struct nvhost_intr *intr)
{
	struct nvhost_master *dev = intr_to_dev(intr);

	/* increase the auto-ack timout to the maximum value. 2d will hang
	 * otherwise on ap20.
	 */
	host1x_hypervisor_writel(dev->dev,
			host1x_sync_ctxsw_timeout_cfg_r(), 0xff);

	/* enable graphics host syncpoint interrupt */
	intr_set_syncpt_threshold(intr,
			nvhost_syncpt_graphics_host_sp(&dev->syncpt),
			1);
	intr_enable_syncpt_intr(intr,
			nvhost_syncpt_graphics_host_sp(&dev->syncpt));

	/* enable host module interrupt to CPU0 */
	host1x_hypervisor_writel(dev->dev, host1x_sync_intc0mask_r(), BIT(0));
	host1x_hypervisor_writel(dev->dev, host1x_sync_intgmask_r(), BIT(0));
	/* enable syncpoint interrupts */
	host1x_hypervisor_writel(dev->dev, host1x_sync_syncpt_intgmask_r(),
				/* Camera CPUs 2 and 3 */
				BIT(2) | BIT(3) |
				/* VM1..VM8 */
				(0xff << 8));

	/* master enable for general (not syncpt) host interrupts
	 * (AXIREAD, AXIWRITE, Syncpoint protection) */
	host1x_hypervisor_writel(dev->dev, host1x_sync_intmask_r(),
				 BIT(0) | BIT(1) | BIT(30) | BIT(28));
}

static void intr_suspend(struct nvhost_intr *intr)
{
	struct nvhost_master *dev = intr_to_dev(intr);

	/* master disable for general (not syncpt) host interrupts */
	host1x_hypervisor_writel(dev->dev, host1x_sync_intmask_r(), 0);
	host1x_hypervisor_writel(dev->dev, host1x_sync_syncpt_intgmask_r(), 0);

	/* disable graphics host syncpoint interrupt */
	intr_disable_syncpt_intr(intr,
			nvhost_syncpt_graphics_host_sp(&dev->syncpt));
}

static int intr_init(struct nvhost_intr *intr)
{
	struct nvhost_master *dev = intr_to_dev(intr);
	int err;

	intr_op().disable_all_syncpt_intrs(intr);

	err = request_threaded_irq(intr->syncpt_irq, NULL,
				syncpt_thresh_cascade_isr,
				IRQF_ONESHOT, "host_syncpt", dev);
	if (err)
		return err;

	/* master disable for general (not syncpt) host interrupts */
	host1x_hypervisor_writel(dev->dev, host1x_sync_intc0mask_r(), 0);
	host1x_hypervisor_writel(dev->dev, host1x_sync_intgmask_r(), 0);
	host1x_hypervisor_writel(dev->dev, host1x_sync_intmask_r(), 0);

	err = request_threaded_irq(intr->general_irq, NULL,
				intr_host1x_isr,
				IRQF_ONESHOT, "host_status", intr);
	if (err)
		dev_warn(&dev->dev->dev,
		         "general irq request failed, but continuing\n");

	return 0;
}

static void intr_deinit(struct nvhost_intr *intr)
{
	struct nvhost_master *dev = intr_to_dev(intr);

	free_irq(intr->general_irq, intr);
	free_irq(intr->syncpt_irq, dev);
}

static const struct nvhost_intr_ops host1x_intr_ops = {
	.init = intr_init,
	.deinit = intr_deinit,
	.resume = intr_resume,
	.suspend = intr_suspend,
	.set_host_clocks_per_usec = intr_set_host_clocks_per_usec,
	.set_syncpt_threshold = intr_set_syncpt_threshold,
	.enable_syncpt_intr = intr_enable_syncpt_intr,
	.disable_syncpt_intr = intr_disable_syncpt_intr,
	.disable_all_syncpt_intrs = intr_disable_all_syncpt_intrs,
	.debug_dump = intr_debug_dump,
	.disable_host_irq = intr_disable_host_irq,
	.enable_host_irq = intr_enable_host_irq,
	.disable_module_intr = intr_disable_module_intr,
	.enable_module_intr = intr_enable_module_intr,
};
