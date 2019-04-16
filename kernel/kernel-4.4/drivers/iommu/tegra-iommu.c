/*
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
 */

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/tegra_smmu.h>
#include <linux/dma-contiguous.h>
#include <soc/tegra/chip-id.h>

#include <soc/tegra/fuse.h>

#include <asm/dma-iommu.h>

#include <dt-bindings/memory/tegra-swgroup.h>

#include <soc/tegra/memory-carveout.h>
#include <linux/platform/tegra/common.h>
#include <soc/tegra/common.h>

static struct iommu_linear_map tegra_fb_linear_map[16]; /* Terminated with 0 */

#define LINEAR_MAP_ADD(n) \
do { \
	if (n##_start && n##_size) { \
		map[i].is_mapped = false; \
		map[i].start = n##_start; \
		map[i++].size = n##_size; \
	} \
} while (0)

#if defined(CONFIG_DMA_CMA) && defined(CONFIG_TEGRA_NVMAP)
static void carveout_linear_set(struct device *cma_dev)
{
	struct dma_contiguous_stats stats;
	struct iommu_linear_map *map = &tegra_fb_linear_map[0];

	if (dma_get_contiguous_stats(cma_dev, &stats))
		return;

	/* get the free slot at end and add carveout entry */
	while (map && map->size)
		map++;
	map->start = stats.base;
	map->size = stats.size;
}
#endif

static void cma_carveout_linear_set(void)
{
#if defined(CONFIG_DMA_CMA) && defined(CONFIG_TEGRA_NVMAP)
	if (tegra_vpr_resize) {
		carveout_linear_set(&tegra_generic_cma_dev);
		carveout_linear_set(&tegra_vpr_cma_dev);
	}
#endif
}

void tegra_fb_linear_set(struct iommu_linear_map *map)
{
	int i = 0;

	map = tegra_fb_linear_map;

	LINEAR_MAP_ADD(tegra_fb);
	LINEAR_MAP_ADD(tegra_fb2);
	LINEAR_MAP_ADD(tegra_fb3);
	LINEAR_MAP_ADD(tegra_bootloader_fb);
	LINEAR_MAP_ADD(tegra_bootloader_fb2);
	LINEAR_MAP_ADD(tegra_bootloader_fb3);
	LINEAR_MAP_ADD(tegra_bootloader_lut);
	LINEAR_MAP_ADD(tegra_bootloader_lut2);
	LINEAR_MAP_ADD(tegra_bootloader_lut3);
#ifdef CONFIG_TEGRA_NVMAP
	if (!tegra_vpr_resize) {
		LINEAR_MAP_ADD(tegra_vpr);
		LINEAR_MAP_ADD(tegra_carveout);
	}
#endif
}
EXPORT_SYMBOL(tegra_fb_linear_set);

struct swgid_fixup {
	const char * const name;
	u64 swgids;
	struct iommu_linear_map *linear_map;
};

#ifdef CONFIG_PLATFORM_ENABLE_IOMMU
#define DUMMY_DEV_NAME "dummy_dev"
#define DUMMY_DEV_MAX_NAME_SIZE 100
static char dummy_name[DUMMY_DEV_MAX_NAME_SIZE] = DUMMY_DEV_NAME;
#endif

/*
 * FIXME: They should have a DT entry with swgroup IDs.
 */
static struct swgid_fixup tegra_swgid_fixup_t124[] = {
	{ .name = "nvavp",	.swgids = TEGRA_SWGROUP_BIT(AVPC) |
					  TEGRA_SWGROUP_BIT(A9AVP), },
	{ .name = "sdhci-tegra.2",	.swgids = TEGRA_SWGROUP_BIT(SDMMC3A) },
	{ .name = "serial8250",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "dtv",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "snd-soc-dummy",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "spdif-dit",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra12-se",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra30-ahub",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra30-dam",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra30-hda",	.swgids = TEGRA_SWGROUP_BIT(HDA), },
	{ .name = "tegra30-i2s",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra30-spdif",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra30-avp-audio",	.swgids = TEGRA_SWGROUP_BIT(AVPC) |
						  TEGRA_SWGROUP_BIT(A9AVP), },
	{ .name = "tegradc.0", .swgids = TEGRA_SWGROUP_BIT(DC) |
					 TEGRA_SWGROUP_BIT(DC12),
	  .linear_map = tegra_fb_linear_map, },
	{ .name = "tegradc.1", .swgids = TEGRA_SWGROUP_BIT(DCB),
	  .linear_map = tegra_fb_linear_map, },
	{ .name = "tegra-ehci",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra-fuse",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	/*
	 * PPCS1 selection for USB2 needs AHB_ARBC register program
	 * in warm boot and cold boot paths in BL as it needs
	 * secure write.
	 */
	{ .name = "tegra-otg",	.swgids = TEGRA_SWGROUP_BIT(PPCS1), },
	{ .name = "tegra-snd",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "tegra-udc",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
	{ .name = "vic",	.swgids = SWGIDS_ERROR_CODE, },
	{ .name = "vi",	.swgids = TEGRA_SWGROUP_BIT(VI), },
	{ .name = "therm_est",	.swgids = TEGRA_SWGROUP_BIT(PPCS), },
#ifdef CONFIG_PLATFORM_ENABLE_IOMMU
	{ .name = dummy_name,	.swgids = TEGRA_SWGROUP_BIT(PPCS) },
#endif
	{ .name = "tegra-xhci",	.swgids = TEGRA_SWGROUP_BIT(XUSB_HOST), },
	{},
};

static struct swgid_fixup tegra_swgid_fixup_t210[] = {
	{
		.name = "bpmp",
		.swgids = TEGRA_SWGROUP_BIT(AVPC),
	},
	{ .name = "serial8250",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "snd-soc-dummy",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
						  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "spdif-dit",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegra21-se",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(SE) |
	  TEGRA_SWGROUP_BIT(SE1), },
	{ .name = "tegra30-hda",	.swgids = TEGRA_SWGROUP_BIT(HDA), },
	{ .name = "tegra30-spdif",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
						  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegradc.0", .swgids = TEGRA_SWGROUP_BIT(DC) |
	 TEGRA_SWGROUP_BIT(DC12), .linear_map = tegra_fb_linear_map, },
	{ .name = "tegradc.1", .swgids = TEGRA_SWGROUP_BIT(DCB),
				.linear_map = tegra_fb_linear_map, },
	{ .name = "54200000.dc", .swgids = TEGRA_SWGROUP_BIT(DC) |
	 TEGRA_SWGROUP_BIT(DC12), .linear_map = tegra_fb_linear_map, },
	{ .name = "54240000.dc", .swgids = TEGRA_SWGROUP_BIT(DCB),
				.linear_map = tegra_fb_linear_map, },
	{ .name = "tegra-fuse",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegra-otg",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegra-se",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
	{ .name = "tegra-udc",	.swgids = TEGRA_SWGROUP_BIT(PPCS) |
					  TEGRA_SWGROUP_BIT(PPCS1) |
	  TEGRA_SWGROUP_BIT(PPCS2), },
#ifdef CONFIG_PLATFORM_ENABLE_IOMMU
	{ .name = dummy_name,	.swgids = TEGRA_SWGROUP_BIT(PPCS) },
#endif
	{},
};

u64 tegra_smmu_fixup_swgids(struct device *dev, struct iommu_linear_map **map)
{
	const char *s;
	struct swgid_fixup *table;

	if (!dev)
		return SWGIDS_ERROR_CODE;

	switch (tegra_get_chip_id()) {
	case TEGRA124:
	case TEGRA132:
		table = tegra_swgid_fixup_t124;
		break;
	case TEGRA210:
		table = tegra_swgid_fixup_t210;
		break;
	default:
		return SWGIDS_ERROR_CODE;
	}

	while ((s = table->name) != NULL) {
		if (strncmp(s, dev_name(dev), strlen(s))) {
			table++;
			continue;
		}

		if (map)
			*map = table->linear_map;

		if (dev->of_node)
			break;

		pr_info("No Device Node present for smmu client: %s !!\n",
			dev_name(dev));
		break;
	}

	return table->name ? table->swgids : SWGIDS_ERROR_CODE;
}
EXPORT_SYMBOL(tegra_smmu_fixup_swgids);

static int __init tegra_smmu_init(void)
{
	tegra_fb_linear_set(NULL);
	cma_carveout_linear_set();
	return 0;
}
pure_initcall(tegra_smmu_init);

struct iommu_linear_map_mapping {
	const char * const name;
	struct iommu_linear_map *map;
};

static struct iommu_linear_map_mapping t186_linear_map[] = {
	{
		.name = "15200000.nvdisplay",
		.map = tegra_fb_linear_map,
	},
	{
		.name = "15210000.nvdisplay",
		.map = tegra_fb_linear_map,
	},
	{
		.name = "15220000.nvdisplay",
		.map = tegra_fb_linear_map,
	},
	{},
};

int iommu_get_linear_map(struct device *dev, struct iommu_linear_map **map)
{
	const char *s;
	struct iommu_linear_map_mapping *table;

	if (!dev)
		return 0;

	switch (tegra_get_chip_id()) {
	case TEGRA186:
		table = t186_linear_map;
		break;
	default:
		return 0;
	}

	while ((s = table->name) != NULL) {
		if (!strncmp(s, dev_name(dev), strlen(s))) {
			*map = table->map;
			return 1;
		}
		table++;
	}
	return 0;
}
EXPORT_SYMBOL(iommu_get_linear_map);
