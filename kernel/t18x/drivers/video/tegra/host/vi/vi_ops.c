#include <linux/debugfs.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/tegra_pm_domains.h>
#include <linux/uaccess.h>
#include <linux/module.h>

#include "dev.h"
#include "nvhost_acm.h"
#include "vi_notify.h"
#include "vi4.h"
#include "t186/t186.h"
#include <linux/nvhost_vi_ioctl.h>
#include <linux/platform/tegra/latency_allowance.h>
#include "camera/vi/mc_common.h"
#include "camera/vi/vi4_fops.h"

#define VI_CFG_INTERRUPT_STATUS_0		0x0044
#define VI_CFG_INTERRUPT_MASK_0			0x0048
#define VI_ISPBUFA_ERROR_0			0x1000
#define VI_FMLITE_ERROR_0			0x313C
#define VI_NOTIFY_TAG_CLASSIFY_SAFETY_0		0x6008
#define VI_NOTIFY_TAG_CLASSIFY_SAFETY_ERROR_0	0x600C
#define VI_NOTIFY_TAG_CLASSIFY_SAFETY_TEST_0	0x6010
#define VI_NOTIFY_ERROR_0			0x6020

#define VI_HOST_PKTINJECT_STALL_ERR_MASK	0x00000080
#define VI_CSIMUX_FIFO_OVFL_ERR_MASK		0x00000040
#define VI_ATOMP_PACKER_OVFL_ERR_MASK		0x00000020
#define VI_FMLITE_BUF_OVFL_ERR_MASK		0x00000010
#define VI_NOTIFY_FIFO_OVFL_ERR_MASK		0x00000008
#define VI_ISPBUFA_ERR_MASK			0x00000001


/* NV host device */
int nvhost_vi4_prepare_poweroff(struct platform_device *pdev)
{
	struct nvhost_vi_dev *vi = nvhost_get_private_data(pdev);

	host1x_writel(pdev, VI_CFG_INTERRUPT_MASK_0, 0x00000000);
	if (!IS_ERR_VALUE(vi->error_irq))
		disable_irq(vi->error_irq);
	return 0;
}

int nvhost_vi4_finalize_poweron(struct platform_device *pdev)
{
	struct nvhost_vi_dev *vi = nvhost_get_private_data(pdev);

	host1x_writel(pdev, VI_CFG_INTERRUPT_MASK_0,
			VI_HOST_PKTINJECT_STALL_ERR_MASK |
			VI_CSIMUX_FIFO_OVFL_ERR_MASK |
			VI_ATOMP_PACKER_OVFL_ERR_MASK |
			VI_FMLITE_BUF_OVFL_ERR_MASK |
			VI_NOTIFY_FIFO_OVFL_ERR_MASK |
			VI_ISPBUFA_ERR_MASK);
	if (!IS_ERR_VALUE(vi->error_irq))
		enable_irq(vi->error_irq);
	return 0;
}

int nvhost_vi4_aggregate_constraints(struct platform_device *dev,
				int clk_index,
				unsigned long floor_rate,
				unsigned long pixelrate,
				unsigned long bw_constraint)
{
	struct nvhost_device_data *pdata = nvhost_get_devdata(dev);

	if (!pdata) {
		dev_err(&dev->dev,
			"No platform data, fall back to default policy\n");
		return 0;
	}
	if (!pixelrate || clk_index != 0)
		return 0;
	/* SCF send request using NVHOST_CLK, which is calculated
	 * in floor_rate, so we need to aggregate its request
	 * with V4L2 pixelrate request
	 */
	if (floor_rate)
		return floor_rate + (pixelrate / pdata->num_ppc);

	return pixelrate / pdata->num_ppc;
}

void nvhost_vi4_idle(struct platform_device *pdev)
{
	struct nvhost_vi_dev *vi = nvhost_get_private_data(pdev);

	vi->busy = false;
}

void nvhost_vi4_busy(struct platform_device *pdev)
{
	struct nvhost_vi_dev *vi = nvhost_get_private_data(pdev);

	vi->busy = true;
}

void nvhost_vi4_reset(struct platform_device *pdev)
{
	struct nvhost_vi_dev *vi = nvhost_get_private_data(pdev);

	if (vi->busy)
		return;
	if (!IS_ERR(vi->vi_reset))
		reset_control_reset(vi->vi_reset);
	if (!IS_ERR(vi->vi_tsc_reset))
		reset_control_reset(vi->vi_tsc_reset);
}

static int vi_set_la(u32 vi_bw, struct platform_device *pdev)
{
	int ret = 0;

	ret = tegra_set_camera_ptsa(TEGRA_LA_VI_W, vi_bw, 1);
	if (ret) {
		dev_err(&pdev->dev, "%s: set ptsa failed: %d\n",
			__func__, ret);
		return ret;
	}

	/*  T186 ISP/VI LA programming is changed.
	 *  Check tegra18x_la.c
	 */

	return ret;
}

int vi4_v4l2_set_la(struct platform_device *pdev,
			u32 vi_bypass_bw, u32 is_ioctl)
{
	int ret = 0;
	unsigned long total_bw;
	struct nvhost_vi_dev *vi = nvhost_get_private_data(pdev);

	mutex_lock(&vi->update_la_lock);
	/* SCF has already aggregated bw number */
	if (is_ioctl)
		vi->vi_bypass_bw = vi_bypass_bw;

	total_bw = vi->mc_vi.aggregated_kbyteps + vi->vi_bypass_bw;
	ret = vi_set_la(total_bw, pdev);
	mutex_unlock(&vi->update_la_lock);
	return ret;
}
EXPORT_SYMBOL(vi4_v4l2_set_la);

static long nvhost_vi4_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct platform_device *pdev = file->private_data;

	switch (cmd) {
	case NVHOST_VI_IOCTL_SET_VI_CLK: {
		long rate;

		if (!(file->f_mode & FMODE_WRITE))
			return -EINVAL;
		if (get_user(rate, (long __user *)arg))
			return -EFAULT;

		return nvhost_module_set_rate(pdev, file, rate, 0,
						NVHOST_CLOCK);
	}
	case NVHOST_VI_IOCTL_SET_VI_LA_BW: {
		int ret = 0;
		u32 vi_la_bw;

		if (!(file->f_mode & FMODE_WRITE))
			return -EINVAL;
		if (get_user(vi_la_bw, (u32 __user *)arg))
			return -EFAULT;

		/* Set latency allowance for VI, BW is in MBps */
		ret = vi4_v4l2_set_la(pdev, vi_la_bw, 1);
		if (ret) {
			dev_err(&pdev->dev, "%s: failed to set la vi_bw %u MBps\n",
				__func__, vi_la_bw);
			return -ENOMEM;
		}
		return 0;
	}
	}
	return -ENOIOCTLCMD;
}

static int nvhost_vi4_open(struct inode *inode, struct file *file)
{
	struct nvhost_device_data *pdata = container_of(inode->i_cdev,
					struct nvhost_device_data, ctrl_cdev);
	struct platform_device *pdev = pdata->pdev;
	int err;

	if ((file->f_flags & O_ACCMODE) != O_WRONLY)
		return -EACCES;

	err = nvhost_module_add_client(pdev, file);
	if (err)
		return err;

	file->private_data = pdev;
	return nonseekable_open(inode, file);
}

static int nvhost_vi4_release(struct inode *inode, struct file *file)
{
	struct platform_device *pdev = file->private_data;

	nvhost_module_remove_client(pdev, file);
	return 0;
}

const struct file_operations nvhost_vi4_ctrl_ops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = nvhost_vi4_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = nvhost_vi4_ioctl,
#endif
	.open = nvhost_vi4_open,
	.release = nvhost_vi4_release,
};
