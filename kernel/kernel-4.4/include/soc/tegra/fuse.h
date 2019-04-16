/*
 * Copyright (c) 2012-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __SOC_TEGRA_FUSE_H__
#define __SOC_TEGRA_FUSE_H__

#include <soc/tegra/chip-id.h>

#define TEGRA_FUSE_PRODUCTION_MODE	0x0
#define FUSE_FUSEBYPASS_0		0x24
#define FUSE_WRITE_ACCESS_SW_0		0x30

#define FUSE_SKU_INFO			0x10
#define FUSE_SKU_MSB_MASK		0xFF00
#define FUSE_SKU_MSB_SHIFT		8

#define FUSE_SKU_USB_CALIB_0		0xf0
#define TEGRA_FUSE_SKU_CALIB_0		0xf0

#define FUSE_OPT_VENDOR_CODE		0x100
#define FUSE_OPT_VENDOR_CODE_MASK	0xf
#define FUSE_OPT_FAB_CODE		0x104
#define FUSE_OPT_FAB_CODE_MASK		0x3f
#define FUSE_OPT_LOT_CODE_0		0x108
#define FUSE_OPT_LOT_CODE_1		0x10c
#define FUSE_OPT_WAFER_ID		0x118
#define FUSE_OPT_WAFER_ID_MASK		0x3f
#define FUSE_OPT_X_COORDINATE		0x114
#define FUSE_OPT_X_COORDINATE_MASK	0x1ff
#define FUSE_OPT_Y_COORDINATE		0x118
#define FUSE_OPT_Y_COORDINATE_MASK	0x1ff

#define TEGRA30_FUSE_SATA_CALIB		0x124

#define FUSE_OPT_SUBREVISION		0x148
#define FUSE_OPT_SUBREVISION_MASK	0xF

#define FUSE_OPT_PRIV_SEC_DIS_0		0x164
#define FUSE_OPT_PRIV_SEC_EN_0		0x164
#define FUSE_GCPLEX_CONFIG_FUSE_0	0x1c8

#define FUSE_TSENSOR8_CALIB		0x180
#define FUSE_SPARE_REALIGNMENT_REG_0	0x1fc

#define FUSE_OPT_GPU_TPC0_DISABLE_0	0x30c
#define FUSE_OPT_GPU_TPC1_DISABLE_0	0x33c

#define FUSE_USB_CALIB_EXT_0		0x250
#define TEGRA_FUSE_USB_CALIB_EXT_0	0x250

#define FUSE_OPT_ECC_EN			0x258

u32 tegra_read_straps(void);
u32 tegra_read_ram_code(void);
u32 tegra_read_chipid(void);
enum tegra_chipid tegra_get_chipid(void);
unsigned long long tegra_chip_uid(void);
enum tegra_revision tegra_chip_get_revision(void);
u32 tegra_fuse_get_subrevision(void);

int tegra_fuse_readl(unsigned long offset, u32 *value);
void tegra_fuse_writel(u32 val, unsigned long offset);

#if defined(CONFIG_TEGRA_FUSE)
bool tegra_spare_fuse(int bit);
int tegra_get_sku_override(void);
u32 tegra_get_sku_id(void);
#else
static inline bool tegra_spare_fuse(int bit)
{ return 0; }
static inline int tegra_get_sku_override(void)
{ return 0; }
static inline u32 tegra_get_sku_id(void)
{ return 0; }
#endif


extern int (*tegra_fuse_regulator_en)(int);
int tegra_soc_speedo_id(void);
void tegra_init_speedo_data(void);
int tegra_cpu_process_id(void);
int tegra_core_process_id(void);
int tegra_gpu_process_id(void);
int tegra_get_age(void);
static inline bool tegra_is_soc_automotive_speedo(void)
{
	return 0;
}

int tegra_package_id(void);
int tegra_cpu_speedo_id(void);
int tegra_cpu_speedo_mv(void);
int tegra_cpu_speedo_value(void);
int tegra_core_speedo_mv(void);
int tegra_core_speedo_min_mv(void);
int tegra_gpu_speedo_id(void);
int tegra_fuse_get_cpu_iddq(void);
int tegra_get_chip_personality(void);

int tegra_fuse_get_soc_iddq(void);
int tegra_fuse_get_gpu_iddq(void);

int tegra_fuse_get_tsensor_calib(int index, u32 *calib);
int tegra_fuse_calib_base_get_cp(u32 *base_cp, s32 *shifted_cp);
int tegra_fuse_calib_base_get_ft(u32 *base_ft, s32 *shifted_ft);

int tegra_fuse_control_read(unsigned long offset, u32 *value);
void tegra_fuse_control_write(u32 value, unsigned long offset);
void tegra_pmc_fuse_disable_mirroring(void);
void tegra_pmc_fuse_enable_mirroring(void);

#endif /* __SOC_TEGRA_FUSE_H__ */
