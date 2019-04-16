/*
 * tegra_asoc_machine_virt_alt.c - Tegra xbar dai link for machine drivers
 *
 * Copyright (c) 2017 NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/module.h>
#include <linux/of.h>
#include <linux/export.h>
#include <sound/soc.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/tegra_pm_domains.h>


#include "tegra_asoc_machine_virt_alt.h"

#define CODEC_NAME		NULL

#define DAI_NAME(i)		"AUDIO" #i
#define STREAM_NAME		"playback"
#define LINK_CPU_NAME		DRV_NAME
#define CPU_DAI_NAME(i)		"ADMAIF" #i
#define CODEC_DAI_NAME		"dit-hifi"
#define PLATFORM_NAME		LINK_CPU_NAME

static unsigned int num_dai_links;
static const struct snd_soc_pcm_stream default_params = {
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};

static const struct snd_soc_pcm_stream adsp_default_params = {
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};


struct snd_soc_dai_link tegra_virt_t186ref_pcm_links[] = {
	{
		/* 0 */
		.name = DAI_NAME(1),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(1),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 1 */
		.name = DAI_NAME(2),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(2),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 2 */
		.name = DAI_NAME(3),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(3),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 3 */
		.name = DAI_NAME(4),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(4),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 4 */
		.name = DAI_NAME(5),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(5),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 5 */
		.name = DAI_NAME(6),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(6),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 6 */
		.name = DAI_NAME(7),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(7),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 7 */
		.name = DAI_NAME(8),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(8),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 8 */
		.name = DAI_NAME(9),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(9),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 9 */
		.name = DAI_NAME(10),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(10),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 10 */
		.name = DAI_NAME(11),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(11),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 11 */
		.name = DAI_NAME(12),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(12),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 12 */
		.name = DAI_NAME(13),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(13),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 13 */
		.name = DAI_NAME(14),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(14),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 14 */
		.name = DAI_NAME(15),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(15),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 15 */
		.name = DAI_NAME(16),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(16),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 16 */
		.name = DAI_NAME(17),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(17),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 17 */
		.name = DAI_NAME(18),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(18),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 18 */
		.name = DAI_NAME(19),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(19),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 19 */
		.name = DAI_NAME(20),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(20),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 20 */
		.name = "ADSP ADMAIF1",
		.stream_name = "ADSP AFMAIF1",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF1",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 21 */
		.name = "ADSP ADMAIF2",
		.stream_name = "ADSP AFMAIF2",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF2",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 22 */
		.name = "ADSP ADMAIF3",
		.stream_name = "ADSP AFMAIF3",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF3",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 23 */
		.name = "ADSP ADMAIF4",
		.stream_name = "ADSP AFMAIF4",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF4",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 24 */
		.name = "ADSP ADMAIF5",
		.stream_name = "ADSP AFMAIF5",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF5",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 25 */
		.name = "ADSP ADMAIF6",
		.stream_name = "ADSP AFMAIF6",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF6",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 26 */
		.name = "ADSP ADMAIF7",
		.stream_name = "ADSP AFMAIF7",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF7",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 27 */
		.name = "ADSP ADMAIF8",
		.stream_name = "ADSP AFMAIF8",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF8",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 28 */
		.name = "ADSP ADMAIF9",
		.stream_name = "ADSP AFMAIF9",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF9",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 29 */
		.name = "ADSP ADMAIF10",
		.stream_name = "ADSP AFMAIF10",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF10",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 30 */
		.name = "ADSP ADMAIF11",
		.stream_name = "ADSP AFMAIF11",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF11",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 31 */
		.name = "ADSP ADMAIF12",
		.stream_name = "ADSP AFMAIF12",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF12",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 32 */
		.name = "ADSP ADMAIF13",
		.stream_name = "ADSP AFMAIF13",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF13",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 33 */
		.name = "ADSP ADMAIF14",
		.stream_name = "ADSP AFMAIF14",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF14",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 34 */
		.name = "ADSP ADMAIF15",
		.stream_name = "ADSP AFMAIF15",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF15",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 35 */
		.name = "ADSP ADMAIF16",
		.stream_name = "ADSP AFMAIF16",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF16",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 36 */
		.name = "ADSP ADMAIF17",
		.stream_name = "ADSP AFMAIF17",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF17",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 37 */
		.name = "ADSP ADMAIF18",
		.stream_name = "ADSP AFMAIF18",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF18",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 38 */
		.name = "ADSP ADMAIF19",
		.stream_name = "ADSP AFMAIF19",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF19",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 39 */
		.name = "ADSP ADMAIF20",
		.stream_name = "ADSP AFMAIF20",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF20",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 40 */
		.name = "ADSP PCM1",
		.stream_name = "ADSP PCM1",
		.codec_name = "adsp_audio",
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP PCM1",
		.codec_dai_name = "ADSP-FE1",
		.platform_name = "adsp_audio",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 41 */
		.name = "ADSP PCM2",
		.stream_name = "ADSP PCM2",
		.codec_name = "adsp_audio",
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP PCM2",
		.codec_dai_name = "ADSP-FE2",
		.platform_name = "adsp_audio",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
};

struct snd_soc_dai_link tegra_virt_t210ref_pcm_links[] = {
	{
		/* 0 */
		.name = DAI_NAME(1),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(1),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 1 */
		.name = DAI_NAME(2),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(2),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 2 */
		.name = DAI_NAME(3),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(3),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 3 */
		.name = DAI_NAME(4),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(4),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 4 */
		.name = DAI_NAME(5),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(5),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 5 */
		.name = DAI_NAME(6),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(6),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 6 */
		.name = DAI_NAME(7),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(7),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 7 */
		.name = DAI_NAME(8),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(8),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 8 */
		.name = DAI_NAME(9),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(9),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 9 */
		.name = DAI_NAME(10),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(10),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
};

void tegra_virt_machine_set_num_dai_links(unsigned int val)
{
	num_dai_links = val;
}
EXPORT_SYMBOL(tegra_virt_machine_set_num_dai_links);

unsigned int tegra_virt_machine_get_num_dai_links(void)
{
	return num_dai_links;
}
EXPORT_SYMBOL(tegra_virt_machine_get_num_dai_links);

struct snd_soc_dai_link *tegra_virt_machine_get_dai_link(void)
{
	struct snd_soc_dai_link *link = tegra_virt_t186ref_pcm_links;
	unsigned int size = TEGRA186_XBAR_DAI_LINKS;

	if (of_machine_is_compatible("nvidia,tegra210")) {
		link = tegra_virt_t210ref_pcm_links;
		size = TEGRA210_XBAR_DAI_LINKS;
	}

	tegra_virt_machine_set_num_dai_links(size);
	return link;
}
EXPORT_SYMBOL(tegra_virt_machine_get_dai_link);

MODULE_AUTHOR("Dipesh Gandhi <dipeshg@nvidia.com>");
MODULE_DESCRIPTION("Tegra Virt ASoC machine code");
MODULE_LICENSE("GPL");
