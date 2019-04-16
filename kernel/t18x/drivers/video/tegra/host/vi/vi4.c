/*
 * VI driver for T186
 *
 * Copyright (c) 2015-2017 NVIDIA Corporation.  All rights reserved.
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
#define DEBUG 1
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

/* Interrupt handler */
/* NOTE: VI4 has three interrupt lines. This handler is for the master/error
 * line. The other lines are dedicated to VI NOTIFY and handled elsewhere.
 */
static irqreturn_t nvhost_vi4_error_isr(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct nvhost_vi_dev *vi = nvhost_get_private_data(pdev);
	u32 r;

	r = host1x_readl(pdev, VI_NOTIFY_ERROR_0);
	if (r) {
		host1x_writel(pdev, VI_NOTIFY_ERROR_0, 1);
		dev_err(&pdev->dev, "notify buffer overflow\n");
		atomic_inc(&vi->notify_overflow);

		/* FIXME: does not work with RTCPU-based VI Notify */
		if (vi->hvnd != NULL)
			nvhost_vi_notify_error(pdev);
	}

	r = host1x_readl(pdev, VI_NOTIFY_TAG_CLASSIFY_SAFETY_ERROR_0);
	if (r) {
		host1x_writel(pdev, VI_NOTIFY_TAG_CLASSIFY_SAFETY_ERROR_0, r);
		dev_err(&pdev->dev, "safety error mask 0x%08X\n", r);
	}

	r = host1x_readl(pdev, VI_FMLITE_ERROR_0);
	if (r) {
		host1x_writel(pdev, VI_FMLITE_ERROR_0, 1);
		dev_err(&pdev->dev, "FM-Lite buffer overflow\n");
		atomic_inc(&vi->fmlite_overflow);
	}

	r = host1x_readl(pdev, VI_CFG_INTERRUPT_STATUS_0);
	if (r) {
		host1x_writel(pdev, VI_CFG_INTERRUPT_STATUS_0, 1);
		dev_err(&pdev->dev, "master error\n");
		atomic_inc(&vi->overflow);
	}

	return IRQ_HANDLED;
}

struct tegra_vi_data t18_vi_data = {
	.info = (struct nvhost_device_data *)&t18_vi_info,
	.vi_fops = &vi4_fops,
};

/* Platform device */
static struct of_device_id tegra_vi4_of_match[] = {
	{
		.compatible = "nvidia,tegra186-vi",
		.data = &t18_vi_data
	},
	{ },
};

static bool tegra_camera_rtcpu_available(void)
{
	struct device_node *dn;

	dn = of_find_node_by_path("tegra-camera-rtcpu");
	/* Note: false if dn is NULL */
	return of_device_is_available(dn);
}

static int tegra_vi4_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct nvhost_device_data *pdata;
	struct nvhost_vi_dev *vi;
	struct tegra_vi_data *data = NULL;
	int err;

	match = of_match_device(tegra_vi4_of_match, &pdev->dev);
	BUG_ON(match == NULL);
	data = (struct tegra_vi_data *) match->data;

	pdata = (struct nvhost_device_data *)data->info;
	BUG_ON(pdata == NULL);

	pdata->pdev = pdev;
	mutex_init(&pdata->lock);
	platform_set_drvdata(pdev, pdata);

	vi = devm_kzalloc(&pdev->dev, sizeof(*vi), GFP_KERNEL);
	if (unlikely(vi == NULL))
		return -ENOMEM;

	vi->vi_reset = devm_reset_control_get(&pdev->dev, "vi");
	vi->vi_tsc_reset = devm_reset_control_get(&pdev->dev, "tsctnvi");
	vi->debug_dir = debugfs_create_dir("tegra_vi", NULL);

	if (vi->debug_dir != NULL) {
		debugfs_create_atomic_t("overflow", S_IRUGO, vi->debug_dir,
					&vi->overflow);
		debugfs_create_atomic_t("notify-overflow", S_IRUGO,
					vi->debug_dir, &vi->notify_overflow);
		debugfs_create_atomic_t("fmlite-overflow", S_IRUGO,
					vi->debug_dir, &vi->fmlite_overflow);
	}

	nvhost_set_private_data(pdev, vi);
	err = nvhost_client_device_get_resources(pdev);
	if (err)
		return err;

	nvhost_module_init(pdev);
	err = nvhost_client_device_init(pdev);
	if (err) {
		nvhost_module_deinit(pdev);
		return err;
	}

	vi->error_irq = platform_get_irq(pdev, 0);
	if (!IS_ERR_VALUE(vi->error_irq)) {
		err = devm_request_threaded_irq(&pdev->dev, vi->error_irq,
						NULL, nvhost_vi4_error_isr,
						IRQF_ONESHOT,
						dev_name(&pdev->dev), pdev);
		if (err) {
			dev_err(&pdev->dev, "cannot get master IRQ %d: %d\n",
				vi->error_irq, err);
			vi->error_irq = -ENXIO;
		} else
			disable_irq(vi->error_irq);
	} else
		dev_warn(&pdev->dev, "missing master IRQ\n");

	if (!tegra_camera_rtcpu_available()) {
		err = vi_notify_register(&nvhost_vi_notify_driver, &pdev->dev,
						12);
		if (err) {
			nvhost_client_device_release(pdev);
			return err;
		}
	}

	mutex_init(&vi->update_la_lock);
	vi->mc_vi.ndev = pdev;
	vi->mc_vi.fops = data->vi_fops;
	err = tegra_vi_media_controller_init(&vi->mc_vi, pdev);
	if (err) {
		if (vi->hvnd != NULL)
			vi_notify_unregister(&nvhost_vi_notify_driver,
						&pdev->dev);
		nvhost_client_device_release(pdev);
		return err;
	}

	return 0;
}

static int tegra_vi4_remove(struct platform_device *pdev)
{
	struct nvhost_vi_dev *vi = nvhost_get_private_data(pdev);

	tegra_vi_media_controller_cleanup(&vi->mc_vi);
	if (vi->hvnd != NULL)
		vi_notify_unregister(&nvhost_vi_notify_driver, &pdev->dev);
	nvhost_client_device_release(pdev);
	/* ^ includes call to nvhost_module_deinit() */
#ifdef CONFIG_PM_GENERIC_DOMAINS
	tegra_pd_remove_device(&pdev->dev);
#endif
	debugfs_remove_recursive(vi->debug_dir);
	return 0;
}

static struct platform_driver tegra_vi4_driver = {
	.probe = tegra_vi4_probe,
	.remove = tegra_vi4_remove,
	.driver = {
		.name = "tegra-vi4",
		.owner = THIS_MODULE,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
#ifdef CONFIG_OF
		.of_match_table = tegra_vi4_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
		.suppress_bind_attrs = true,
	},
};

static struct of_device_id tegra_vi4_domain_match[] = {
	{ .compatible = "nvidia,tegra186-ve-pd",
		.data = (struct nvhost_device_data *)&t18_vi_info },
	{ },
};

static int __init tegra_vi4_init(void)
{
	int err;

	err = nvhost_domain_init(tegra_vi4_domain_match);
	if (err)
		return err;

	return platform_driver_register(&tegra_vi4_driver);
}

static void __exit tegra_vi4_exit(void)
{
	platform_driver_unregister(&tegra_vi4_driver);
}

late_initcall(tegra_vi4_init);
module_exit(tegra_vi4_exit);
MODULE_LICENSE("GPL v2");
